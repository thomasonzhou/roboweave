#!/usr/bin/env python3
"""
Rudimentary Gait Generator for Unitree Go2
A simple, stable (but stupid-looking) walking gait implementation.

This implements a basic alternating leg gait that prioritizes stability over elegance.
The robot will walk in a "duck walk" style - very stable but not graceful.
"""

import time
import sys
import numpy as np
import math
import threading
import termios
import tty
import select

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

class RudimentaryGait:
    def __init__(self):
        # Robot control parameters
        self.dt = 0.02  # Control loop frequency (50Hz)
        self.crc = CRC()
        
        # Movement commands (high-level inputs)
        self.cmd_vx = 0.0      # Forward velocity command
        self.cmd_vy = 0.0      # Lateral velocity command
        self.cmd_vyaw = 0.0    # Yaw rate command
        
        # Gait parameters
        self.gait_frequency = 0.5  # Steps per second (very slow for stability)
        self.step_height = 0.05    # How high to lift feet (conservative)
        self.step_length = 0.08    # Maximum step length (small steps)
        self.body_height = 0.30    # Fixed body height
        
        # Current gait state
        self.gait_time = 0.0
        self.gait_cycle = 2.0 / self.gait_frequency  # Full cycle time
        self.is_walking = False
        self.is_standing = True
        
        # Joint limits and defaults (12 joints: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf)
        self.default_pos = np.array([
            0.0, 0.9, -1.8,   # Front Left:  hip, thigh, calf
            0.0, 0.9, -1.8,   # Front Right: hip, thigh, calf  
            0.0, 0.9, -1.8,   # Rear Left:   hip, thigh, calf
            0.0, 0.9, -1.8    # Rear Right:  hip, thigh, calf
        ])
        
        # Sit position
        self.sit_pos = np.array([
            0.0, 1.3, -2.6,   # Front Left
            0.0, 1.3, -2.6,   # Front Right
            0.0, 1.3, -2.6,   # Rear Left
            0.0, 1.3, -2.6    # Rear Right
        ])
        
        self.current_pos = self.sit_pos.copy()
        self.target_pos = self.sit_pos.copy()
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        self.running = True
        
        # Publisher
        self.cmd_pub = None
        
    def setup_terminal(self):
        """Setup terminal for raw input"""
        tty.setraw(sys.stdin.fileno())
        
    def restore_terminal(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
    def get_key(self):
        """Get a single keypress without Enter"""
        if select.select([sys.stdin], [], [], 0.01) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def print_help(self):
        """Print control instructions"""
        print("\n" + "="*60)
        print("RUDIMENTARY GAIT CONTROLLER")
        print("="*60)
        print("Robot Control:")
        print("  SPACE : Stand up/sit down")
        print("  m     : Toggle walking mode")
        print("  0     : Stop movement")
        print("")
        print("Walking Commands (when in walk mode):")
        print("  w/s   : Walk forward/backward")
        print("  a/d   : Turn left/right")
        print("  q/e   : Strafe left/right")
        print("")
        print("Other:")
        print("  H     : Show this help")
        print("  ESC   : Exit")
        print("="*60)
        print(f"Status: Standing={self.is_standing}, Walking={self.is_walking}")
        print()
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == ' ':
            # Toggle stand/sit
            self.is_standing = not self.is_standing
            if self.is_standing:
                print("\nStanding up...")
                self.is_walking = False
            else:
                print("\nSitting down...")
                self.is_walking = False
                self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
                
        elif key == 'm':
            # Toggle walking mode
            if self.is_standing:
                self.is_walking = not self.is_walking
                if self.is_walking:
                    print("\nWalking mode ON - use WASD to move")
                    self.gait_time = 0.0  # Reset gait cycle
                else:
                    print("\nWalking mode OFF")
                    self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
            else:
                print("\nMust be standing to walk!")
                
        elif key == '0':
            # Stop all movement
            self.cmd_vx = self.cmd_vy = self.cmd_vyaw = 0.0
            if self.is_walking:
                print("\nStopped movement")
                
        elif key == '\x1b':  # ESC
            self.running = False
            
        # Movement commands (only when walking)
        elif self.is_walking:
            if key == 'w':
                self.cmd_vx = min(self.cmd_vx + 0.1, 0.3)
                print(f"\nForward velocity: {self.cmd_vx:.2f}")
            elif key == 's':
                self.cmd_vx = max(self.cmd_vx - 0.1, -0.3)
                print(f"\nForward velocity: {self.cmd_vx:.2f}")
            elif key == 'a':
                self.cmd_vyaw = min(self.cmd_vyaw + 0.2, 0.5)
                print(f"\nTurn rate: {self.cmd_vyaw:.2f}")
            elif key == 'd':
                self.cmd_vyaw = max(self.cmd_vyaw - 0.2, -0.5)
                print(f"\nTurn rate: {self.cmd_vyaw:.2f}")
            elif key == 'q':
                self.cmd_vy = min(self.cmd_vy + 0.1, 0.2)
                print(f"\nStrafe velocity: {self.cmd_vy:.2f}")
            elif key == 'e':
                self.cmd_vy = max(self.cmd_vy - 0.1, -0.2)
                print(f"\nStrafe velocity: {self.cmd_vy:.2f}")
    
    def generate_stupid_but_stable_gait(self):
        """
        Generate a very simple, stable gait.
        Strategy: Only move one leg at a time, keep the other 3 planted.
        This looks stupid but is very stable.
        """
        if not self.is_walking or (abs(self.cmd_vx) < 0.01 and abs(self.cmd_vy) < 0.01 and abs(self.cmd_vyaw) < 0.01):
            # Not walking or no command - just stand
            return self.default_pos.copy()
        
        # Update gait time
        self.gait_time += self.dt
        if self.gait_time >= self.gait_cycle:
            self.gait_time = 0.0
        
        # Determine which leg is moving (one at a time for maximum stability)
        cycle_phase = self.gait_time / self.gait_cycle
        
        # 4 phases: FL -> FR -> RL -> RR (each leg gets 25% of cycle)
        if cycle_phase < 0.25:
            moving_leg = 0  # Front Left
        elif cycle_phase < 0.5:
            moving_leg = 1  # Front Right
        elif cycle_phase < 0.75:
            moving_leg = 2  # Rear Left
        else:
            moving_leg = 3  # Rear Right
        
        # Calculate step for the moving leg
        leg_phase = (cycle_phase % 0.25) / 0.25  # 0-1 within leg's phase
        
        # Base positions
        positions = self.default_pos.copy()
        
        # Calculate desired foot position based on velocity commands
        step_x = self.cmd_vx * self.step_length
        step_y = self.cmd_vy * self.step_length
        step_yaw = self.cmd_vyaw * 0.1  # Small turning steps
        
        # Apply step to moving leg
        leg_idx = moving_leg * 3  # Each leg has 3 joints
        
        if leg_phase < 0.5:
            # Lift phase - lift foot up
            lift_height = self.step_height * math.sin(leg_phase * 2 * math.pi)
            positions[leg_idx + 1] -= lift_height * 0.5  # Thigh compensates
            positions[leg_idx + 2] += lift_height  # Calf lifts
        else:
            # Step phase - move foot forward/back
            step_progress = (leg_phase - 0.5) * 2  # 0-1 for step phase
            
            # Hip adjustment for forward/back and turning
            if moving_leg in [0, 2]:  # Left legs
                positions[leg_idx] += step_yaw * 0.5 + step_y * 0.3
            else:  # Right legs
                positions[leg_idx] -= step_yaw * 0.5 - step_y * 0.3
            
            # Thigh adjustment for forward/back motion
            if moving_leg in [0, 1]:  # Front legs
                positions[leg_idx + 1] += step_x * 0.3
            else:  # Rear legs
                positions[leg_idx + 1] -= step_x * 0.3
        
        # Apply small body lean for stability (duck walk style)
        body_lean = 0.1
        if moving_leg in [0, 2]:  # Left leg moving, lean right
            positions[1] += body_lean  # FR thigh
            positions[4] += body_lean  # FL thigh  
            positions[7] += body_lean  # RL thigh
            positions[10] += body_lean # RR thigh
        else:  # Right leg moving, lean left
            positions[1] -= body_lean
            positions[4] -= body_lean  
            positions[7] -= body_lean
            positions[10] -= body_lean
        
        return positions
    
    def update_robot_pose(self):
        """Update target joint positions based on robot state"""
        if not self.is_standing:
            # Sitting
            self.target_pos = self.sit_pos.copy()
        elif not self.is_walking:
            # Standing still
            self.target_pos = self.default_pos.copy()
        else:
            # Walking with rudimentary gait
            self.target_pos = self.generate_stupid_but_stable_gait()
        
        # Smooth interpolation to target (for stability)
        alpha = 0.1  # Low value for smooth motion
        self.current_pos = (1 - alpha) * self.current_pos + alpha * self.target_pos
    
    def create_low_cmd(self):
        """Create low-level command message"""
        cmd = unitree_go_msg_dds__LowCmd_()
        
        # Set motor commands for 12 joints
        for i in range(12):
            cmd.motor_cmd[i].mode = 0x01  # Position control mode
            cmd.motor_cmd[i].q = self.current_pos[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 50.0  # Higher gain for stability
            cmd.motor_cmd[i].kd = 2.0   # Higher damping for stability
        
        # Add CRC
        cmd.crc = self.crc.Crc(cmd)
        return cmd
    
    def print_status(self):
        """Print current status"""
        print(f"\rStand: {self.is_standing} | Walk: {self.is_walking} | "
              f"Vx: {self.cmd_vx:+.2f} | Vy: {self.cmd_vy:+.2f} | "
              f"Vyaw: {self.cmd_vyaw:+.2f} | Phase: {self.gait_time/self.gait_cycle:.2f}", 
              end='', flush=True)
    
    def run(self):
        """Main control loop"""
        # Initialize SDK2
        print("Initializing SDK for simulation...")
        ChannelFactoryInitialize(1, "lo")  # Simulation
        
        # Initialize publisher
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        
        self.setup_terminal()
        self.print_help()
        
        print("Starting rudimentary gait controller...")
        print("Note: This gait prioritizes stability over grace - it will look silly!")
        
        try:
            while self.running:
                step_start = time.perf_counter()
                
                # Handle input
                key = self.get_key()
                if key:
                    if key.lower() == 'h' and key.isupper():
                        self.print_help()
                    else:
                        self.handle_input(key.lower())
                
                # Update gait
                self.update_robot_pose()
                
                # Send command
                cmd = self.create_low_cmd()
                self.cmd_pub.Write(cmd)
                
                # Print status
                self.print_status()
                
                # Maintain loop frequency
                elapsed = time.perf_counter() - step_start
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Exiting...")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Stop robot
            if self.cmd_pub:
                stop_cmd = self.create_low_cmd()
                for i in range(12):
                    stop_cmd.motor_cmd[i].q = self.sit_pos[i]
                    stop_cmd.motor_cmd[i].kp = 20.0
                    stop_cmd.motor_cmd[i].kd = 1.0
                self.cmd_pub.Write(stop_cmd)
            
            self.restore_terminal()
            print("\nRudimentary gait controller stopped.")

if __name__ == '__main__':
    print("="*60)
    print("RUDIMENTARY GAIT GENERATOR")
    print("A stable but stupid-looking walking implementation")
    print("="*60)
    print()
    
    gait = RudimentaryGait()
    gait.run()
