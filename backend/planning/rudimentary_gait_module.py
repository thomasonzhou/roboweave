#!/usr/bin/env python3
"""
Sport Mode Bridge with Rudimentary Gait Generator

This implements a basic sport mode interface that converts high-level movement
commands to low-level joint commands using a simple but stable gait.
"""

import time
import math
import numpy as np
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread

# Simple sport mode command structure
class SportModeCmd:
    def __init__(self):
        self.vx = 0.0      # Forward velocity command
        self.vy = 0.0      # Lateral velocity command  
        self.vyaw = 0.0    # Yaw rate command
        self.mode = 0      # 0=idle, 1=balance_stand, 2=walk

class RudimentaryGaitGenerator:
    """
    A simple gait generator that creates stable (but ungraceful) walking motion.
    Uses a "one leg at a time" strategy for maximum stability.
    """
    
    def __init__(self, dt=0.002):
        self.dt = dt
        
        # Gait parameters  
        self.gait_frequency = 0.4  # Very slow for stability
        self.step_height = 0.04    # Conservative step height
        self.step_length = 0.06    # Small steps
        self.body_height = 0.30    # Fixed body height
        
        # Gait state
        self.gait_time = 0.0
        self.gait_cycle = 4.0 / self.gait_frequency  # 4 phases (one per leg)
        
        # Default joint positions (12 joints total)
        # Order: FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, 
        #        RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
        self.stand_pos = np.array([
            0.0, 0.8, -1.6,   # Front Left
            0.0, 0.8, -1.6,   # Front Right  
            0.0, 0.8, -1.6,   # Rear Left
            0.0, 0.8, -1.6    # Rear Right
        ])
        
        self.sit_pos = np.array([
            0.0, 1.3, -2.6,   # Front Left
            0.0, 1.3, -2.6,   # Front Right
            0.0, 1.3, -2.6,   # Rear Left  
            0.0, 1.3, -2.6    # Rear Right
        ])
        
        # Current command
        self.current_cmd = SportModeCmd()
        
    def update_command(self, vx=0.0, vy=0.0, vyaw=0.0, mode=0):
        """Update the high-level movement command"""
        self.current_cmd.vx = vx
        self.current_cmd.vy = vy
        self.current_cmd.vyaw = vyaw
        self.current_cmd.mode = mode
        
        # Reset gait time when starting to walk
        if mode == 2 and (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vyaw) > 0.01):
            if not hasattr(self, '_was_walking') or not self._was_walking:
                self.gait_time = 0.0
            self._was_walking = True
        else:
            self._was_walking = False
    
    def generate_joint_positions(self):
        """Generate joint positions based on current command"""
        
        if self.current_cmd.mode == 0:  # Idle/sit
            return self.sit_pos.copy()
            
        elif self.current_cmd.mode == 1:  # Balance stand
            return self.stand_pos.copy()
            
        elif self.current_cmd.mode == 2:  # Walk
            return self._generate_walking_gait()
        
        else:
            return self.stand_pos.copy()
    
    def _generate_walking_gait(self):
        """Generate the stupidly stable walking gait"""
        # If no movement command, just stand
        if (abs(self.current_cmd.vx) < 0.01 and 
            abs(self.current_cmd.vy) < 0.01 and 
            abs(self.current_cmd.vyaw) < 0.01):
            return self.stand_pos.copy()
        
        # Update gait time
        self.gait_time += self.dt
        if self.gait_time >= self.gait_cycle:
            self.gait_time = 0.0
        
        # Determine which leg is moving (cycle through legs for stability)
        cycle_phase = self.gait_time / self.gait_cycle
        
        # 4 phases: each leg gets 25% of the cycle
        if cycle_phase < 0.25:
            moving_leg = 0  # Front Left
            leg_phase = cycle_phase / 0.25
        elif cycle_phase < 0.5:
            moving_leg = 1  # Front Right
            leg_phase = (cycle_phase - 0.25) / 0.25
        elif cycle_phase < 0.75:
            moving_leg = 2  # Rear Left  
            leg_phase = (cycle_phase - 0.5) / 0.25
        else:
            moving_leg = 3  # Rear Right
            leg_phase = (cycle_phase - 0.75) / 0.25
        
        # Start with standing position
        positions = self.stand_pos.copy()
        
        # Calculate step parameters
        step_x = self.current_cmd.vx * self.step_length
        step_y = self.current_cmd.vy * self.step_length  
        step_yaw = self.current_cmd.vyaw * 0.05  # Small yaw steps
        
        # Apply motion to the moving leg
        leg_start_idx = moving_leg * 3
        
        if leg_phase < 0.5:
            # Lift phase: lift the foot up
            lift_height = self.step_height * math.sin(leg_phase * 2 * math.pi)
            positions[leg_start_idx + 1] -= lift_height * 0.4  # Thigh adjustment
            positions[leg_start_idx + 2] += lift_height * 0.8  # Calf lift
            
        else:
            # Step phase: move foot to new position  
            step_progress = (leg_phase - 0.5) * 2  # 0 to 1
            
            # Hip adjustment for lateral motion and turning
            if moving_leg in [0, 2]:  # Left legs
                positions[leg_start_idx] += step_yaw + step_y * 0.2
            else:  # Right legs  
                positions[leg_start_idx] -= step_yaw - step_y * 0.2
            
            # Thigh adjustment for forward/backward motion
            if moving_leg in [0, 1]:  # Front legs
                positions[leg_start_idx + 1] += step_x * 0.2
            else:  # Rear legs
                positions[leg_start_idx + 1] -= step_x * 0.2
        
        # Add body stability adjustments (the "stupid" part that makes it stable)
        # Lean body away from moving leg for stability
        stability_lean = 0.08
        if moving_leg in [0, 2]:  # Left leg moving, lean right
            # Right legs provide stability
            positions[3 + 1] += stability_lean  # FR thigh
            positions[9 + 1] += stability_lean  # RR thigh
        else:  # Right leg moving, lean left
            # Left legs provide stability  
            positions[0 + 1] += stability_lean  # FL thigh
            positions[6 + 1] += stability_lean  # RL thigh
        
        return positions

# Global gait generator instance
_gait_generator = RudimentaryGaitGenerator()

def sport_mode_move(vx, vy, vyaw):
    """High-level move command - converts to gait"""
    _gait_generator.update_command(vx, vy, vyaw, mode=2)
    return _gait_generator.generate_joint_positions()

def sport_mode_balance_stand():
    """High-level balance stand command"""
    _gait_generator.update_command(0, 0, 0, mode=1)
    return _gait_generator.generate_joint_positions()

def sport_mode_stop():
    """High-level stop command"""
    _gait_generator.update_command(0, 0, 0, mode=0)
    return _gait_generator.generate_joint_positions()

if __name__ == '__main__':
    # Test the gait generator
    print("Testing Rudimentary Gait Generator...")
    
    gait = RudimentaryGaitGenerator()
    
    # Test idle
    print("Idle positions:", gait.generate_joint_positions())
    
    # Test standing
    gait.update_command(mode=1)
    print("Standing positions:", gait.generate_joint_positions())
    
    # Test walking
    gait.update_command(vx=0.1, mode=2)
    print("Walking positions (t=0):", gait.generate_joint_positions())
    
    # Simulate some time steps
    for i in range(10):
        time.sleep(0.1)
        pos = gait.generate_joint_positions()
        print(f"Walking positions (t={i*0.1:.1f}):", pos[:3])  # Just first leg
    
    print("Gait generator test complete!")
