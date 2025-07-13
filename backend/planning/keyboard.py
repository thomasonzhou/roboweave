#!/usr/bin/env python3
"""
Fixed Keyboard teleoperation for Unitree robots using Low-Level Commands.
This follows the official unitree_mujoco approach using LowCmd_ messages.

Joint Order (confirmed from apt68.xml):
0: FR_hip, 1: FR_thigh, 2: FR_calf
3: FL_hip, 4: FL_thigh, 5: FL_calf  
6: RR_hip, 7: RR_thigh, 8: RR_calf
9: RL_hip, 10: RL_thigh, 11: RL_calf

Based on the working implementation from unitree_mujoco repository.
"""

import time
import sys
import numpy as np
import threading
import termios
import tty
import select

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

class KeyboardTeleopLowLevel:
    def __init__(self):
        # Robot control parameters
        self.dt = 0.02  # Control loop frequency (50Hz)
        self.crc = CRC()
        
        # Movement parameters
        self.linear_vel = 0.0      # Forward/backward velocity
        self.angular_vel = 0.0     # Turning velocity
        self.strafe_vel = 0.0      # Side-to-side velocity
        self.body_height = 0.0     # Body height adjustment
        self.body_pitch = 0.0      # Body pitch (forward/backward tilt)
        self.body_roll = 0.0       # Body roll (left/right tilt)
        
        # Control limits
        self.max_linear_vel = 0.5   # m/s
        self.max_angular_vel = 1.0  # rad/s
        self.max_strafe_vel = 0.3   # m/s
        self.max_body_height = 0.1  # m
        self.max_body_tilt = 0.2    # rad
        
        # Velocity increments
        self.vel_increment = 0.1
        self.ang_increment = 0.2
        self.height_increment = 0.02
        self.tilt_increment = 0.05
        
        # Robot state
        self.is_standing = False
        self.current_gait = 1
        self.running = True
        
        # Stand positions (12 joints for Go2)
        # Order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, 
        #        RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
        self.stand_up_pos = np.array([
            0.0,  0.8, -1.6,  # FR (Front Right)
            0.0,  0.8, -1.6,  # FL (Front Left) 
            0.0,  0.8, -1.6,  # RR (Rear Right)
            0.0,  0.8, -1.6   # RL (Rear Left)
        ], dtype=float)
        
        self.stand_down_pos = np.array([
            0.0,  1.4, -2.7,  # FR (Front Right)
            0.0,  1.4, -2.7,  # FL (Front Left)
            0.0,  1.4, -2.7,  # RR (Rear Right) 
            0.0,  1.4, -2.7   # RL (Rear Left)
        ], dtype=float)
        
        # Current joint positions
        self.current_pos = self.stand_down_pos.copy()
        
        # Terminal settings for raw input
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Initialize publisher
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
        print("UNITREE ROBOT LOW-LEVEL KEYBOARD TELEOPERATION")
        print("="*60)
        print("Robot Control:")
        print("  SPACE : Stand up/down toggle")
        print("  1-4   : Switch gaits (not implemented)")
        print("  0     : Stop all movement")
        print("")
        print("Movement Controls (when standing):")
        print("  w/s   : Move forward/backward")
        print("  a/d   : Turn left/right")
        print("  q/e   : Strafe left/right")
        print("")
        print("Body Pose Controls (when standing):")
        print("  r/f   : Increase/decrease body height")
        print("  t/g   : Tilt body forward/backward")
        print("  y/h   : Roll body left/right")
        print("")
        print("Other:")
        print("  H     : Show this help")
        print("  ESC   : Exit")
        print("="*60)
        print("Current Status: Standing={}, Gait={}".format(self.is_standing, self.current_gait))
        print()
        
    def update_velocities(self, key):
        """Update velocities based on key press"""
        if key == ' ':
            # Toggle stand up/down
            self.is_standing = not self.is_standing
            if self.is_standing:
                self.current_pos = self.stand_up_pos.copy()
                print("\\nStanding up...")
            else:
                self.current_pos = self.stand_down_pos.copy()
                print("\\nSitting down...")
                # Reset velocities when sitting
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                self.strafe_vel = 0.0
                
        elif key == '0':
            # Stop all movement
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.strafe_vel = 0.0
            print("\\nStopped all movement")
            
        elif key == '1':
            self.current_gait = 1
            print(f"\\nSwitched to gait {self.current_gait}")
        elif key == '2':
            self.current_gait = 2
            print(f"\\nSwitched to gait {self.current_gait}")
        elif key == '3':
            self.current_gait = 3
            print(f"\\nSwitched to gait {self.current_gait}")
        elif key == '4':
            self.current_gait = 4
            print(f"\\nSwitched to gait {self.current_gait}")
            
        elif key == '\\x1b':  # ESC key
            self.running = False
            
        # Movement controls (only when standing)
        elif self.is_standing:
            if key == 'w':
                self.linear_vel = min(self.linear_vel + self.vel_increment, self.max_linear_vel)
            elif key == 's':
                self.linear_vel = max(self.linear_vel - self.vel_increment, -self.max_linear_vel)
            elif key == 'a':
                self.angular_vel = min(self.angular_vel + self.ang_increment, self.max_angular_vel)
            elif key == 'd':
                self.angular_vel = max(self.angular_vel - self.ang_increment, -self.max_angular_vel)
            elif key == 'q':
                self.strafe_vel = min(self.strafe_vel + self.vel_increment, self.max_strafe_vel)
            elif key == 'e':
                self.strafe_vel = max(self.strafe_vel - self.vel_increment, -self.max_strafe_vel)
                
            # Body pose controls
            elif key == 'r':
                self.body_height = min(self.body_height + self.height_increment, self.max_body_height)
                print(f"\\nBody height: {self.body_height:.2f}")
            elif key == 'f':
                self.body_height = max(self.body_height - self.height_increment, -self.max_body_height)
                print(f"\\nBody height: {self.body_height:.2f}")
            elif key == 't':
                self.body_pitch = min(self.body_pitch + self.tilt_increment, self.max_body_tilt)
                print(f"\\nBody pitch: {self.body_pitch:.2f}")
            elif key == 'g':
                self.body_pitch = max(self.body_pitch - self.tilt_increment, -self.max_body_tilt)
                print(f"\\nBody pitch: {self.body_pitch:.2f}")
            elif key == 'y':
                self.body_roll = min(self.body_roll + self.tilt_increment, self.max_body_tilt)
                print(f"\\nBody roll: {self.body_roll:.2f}")
            elif key == 'h':
                self.body_roll = max(self.body_roll - self.tilt_increment, -self.max_body_tilt)
                print(f"\\nBody roll: {self.body_roll:.2f}")
                
    def create_low_cmd(self):
        """Create low-level command message"""
        cmd = unitree_go_msg_dds__LowCmd_()
        
        # Set motor commands for 12 joints
        for i in range(12):
            cmd.motor_cmd[i].mode = 0x01  # Position control mode
            cmd.motor_cmd[i].q = self.current_pos[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
            cmd.motor_cmd[i].kp = 60.0  # Higher position gain for better control
            cmd.motor_cmd[i].kd = 2.0   # Higher damping gain to prevent oscillation
            
        # Apply movement adjustments when standing
        if self.is_standing:
            # Simple leg adjustments for movement (very basic implementation)
            leg_adjustment = 0.05 * self.linear_vel  # Reduced for stability
            height_adjustment = self.body_height
            
            # Apply adjustments to all legs
            # FR leg (joints 0,1,2)
            cmd.motor_cmd[0].q += 0.05 * self.angular_vel  # Hip turn
            cmd.motor_cmd[1].q += leg_adjustment + self.body_pitch  # Thigh forward/pitch
            cmd.motor_cmd[2].q -= 2 * leg_adjustment + height_adjustment - self.body_pitch  # Calf
            
            # FL leg (joints 3,4,5) 
            cmd.motor_cmd[3].q -= 0.05 * self.angular_vel  # Hip turn (opposite)
            cmd.motor_cmd[4].q += leg_adjustment + self.body_pitch  # Thigh forward/pitch
            cmd.motor_cmd[5].q -= 2 * leg_adjustment + height_adjustment - self.body_pitch  # Calf
            
            # RR leg (joints 6,7,8)
            cmd.motor_cmd[6].q += 0.05 * self.angular_vel  # Hip turn
            cmd.motor_cmd[7].q -= leg_adjustment - self.body_pitch  # Thigh back/pitch
            cmd.motor_cmd[8].q += 2 * leg_adjustment + height_adjustment + self.body_pitch  # Calf
            
            # RL leg (joints 9,10,11)
            cmd.motor_cmd[9].q -= 0.05 * self.angular_vel  # Hip turn (opposite)
            cmd.motor_cmd[10].q -= leg_adjustment - self.body_pitch  # Thigh back/pitch  
            cmd.motor_cmd[11].q += 2 * leg_adjustment + height_adjustment + self.body_pitch  # Calf
            
            # Apply strafe (side-to-side movement)
            strafe_adjustment = 0.03 * self.strafe_vel
            # Adjust hip joints for strafing
            cmd.motor_cmd[0].q += strafe_adjustment  # FR hip
            cmd.motor_cmd[3].q += strafe_adjustment  # FL hip  
            cmd.motor_cmd[6].q += strafe_adjustment  # RR hip
            cmd.motor_cmd[9].q += strafe_adjustment  # RL hip
            
            # Apply body roll
            roll_adjustment = 0.1 * self.body_roll
            # Left legs get positive roll, right legs get negative roll
            cmd.motor_cmd[1].q += roll_adjustment   # FR thigh
            cmd.motor_cmd[4].q -= roll_adjustment   # FL thigh
            cmd.motor_cmd[7].q += roll_adjustment   # RR thigh  
            cmd.motor_cmd[10].q -= roll_adjustment  # RL thigh
        
        # Add CRC
        cmd.crc = self.crc.Crc(cmd)
        return cmd
        
    def print_status(self):
        """Print current status"""
        print(f"\\rStand: {self.is_standing} | Gait: {self.current_gait} | "
              f"Lin: {self.linear_vel:+.2f} | Ang: {self.angular_vel:+.2f} | "
              f"Str: {self.strafe_vel:+.2f} | H: {self.body_height:+.2f} | "
              f"P: {self.body_pitch:+.2f} | R: {self.body_roll:+.2f}", end='', flush=True)
              
    def run(self):
        """Main control loop"""
        # Initialize SDK2
        if len(sys.argv) < 2:
            print("Using simulation interface...")
            ChannelFactoryInitialize(1, "lo")  # Simulation
        else:
            print(f"Using real robot interface: {sys.argv[1]}")
            ChannelFactoryInitialize(0, sys.argv[1])  # Real robot
            
        # Initialize publisher
        self.cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.cmd_pub.Init()
        
        self.setup_terminal()
        self.print_help()
        
        try:
            while self.running:
                step_start = time.perf_counter()
                
                # Get keyboard input
                key = self.get_key()
                if key:
                    if key.lower() == 'h' and key.isupper():
                        self.print_help()
                    else:
                        self.update_velocities(key.lower())
                
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
            print("\\nKeyboard interrupt received. Exiting...")
        except Exception as e:
            print(f"\\nError: {e}")
        finally:
            self.restore_terminal()
            print("\\nTeleoperation stopped.")

if __name__ == '__main__':
    teleop = KeyboardTeleopLowLevel()
    teleop.run()
