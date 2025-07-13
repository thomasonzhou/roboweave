#!/usr/bin/env python3
"""
Keyboard teleoperation for Unitree robots using the Python SDK.
This script allows you to control the robot using keyboard commands.

Controls:
  w/s: Move forward/backward
  a/d: Turn left/right
  q/e: Strafe left/right
  r/f: Increase/decrease body height
  t/g: Tilt body forward/backward
  y/h: Roll body left/right
  space: Stand up/down toggle
  1-4: Switch between different gaits
  ESC: Exit

Make sure to run the simulator first:
  cd simulate_python
  python3 unitree_mujoco.py
"""

import time
import sys
import threading
import termios
import tty
import select

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

class KeyboardTeleop:
    def __init__(self):
        # Robot control parameters
        self.dt = 0.02  # Control loop frequency (50Hz)
        
        # Robot state
        self.is_standing = False
        self.sport_mode_active = False
        
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
        self.running = True
        
        # Sport client
        self.sport_client = None
        
        # Terminal settings for raw input
        self.old_settings = termios.tcgetattr(sys.stdin)
        
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
        print("UNITREE ROBOT KEYBOARD TELEOPERATION (SPORT MODE)")
        print("="*60)
        print("Basic Controls:")
        print("  SPACE : Stand up/down toggle")
        print("  m/M   : Enter/exit sport mode")
        print("  0     : Stop all movement")
        print("  ESC   : Exit")
        print("  h/H   : Show this help")
        print()
        print("Movement Controls (when standing and in sport mode):")
        print("  w/s   : Move forward/backward")
        print("  a/d   : Turn left/right") 
        print("  q/e   : Strafe left/right")
        print()
        print("Body Pose Controls (when standing):")
        print("  r/f   : Increase/decrease body height")
        print("  t/g   : Tilt body forward/backward")
        print("  y/h   : Roll body left/right")
        print()
        print("Robot State Controls:")
        print("  1     : Balance stand")
        print("  2     : Recovery stand") 
        print("  3     : Damping mode")
        print()
        print("Current Status:")
        print(f"  Standing: {self.is_standing}")
        print(f"  Sport Mode: {self.sport_mode_active}")
        print("="*60)
        
    def update_velocities(self, key):
        """Update robot controls based on key input"""
        if key == ' ':
            # Toggle standing
            if not self.is_standing:
                print("\nStanding up...")
                result = self.sport_client.StandUp()
                print(f"StandUp result: {result}")
                self.is_standing = True
            else:
                print("\nStanding down...")
                result = self.sport_client.StandDown()
                print(f"StandDown result: {result}")
                self.is_standing = False
                self.sport_mode_active = False
                
        elif key == 'm':
            # Toggle sport mode (only when standing)
            if self.is_standing:
                if not self.sport_mode_active:
                    print("\nEntering sport mode...")
                    result = self.sport_client.BalanceStand()
                    print(f"BalanceStand result: {result}")
                    self.sport_mode_active = True
                else:
                    print("\nExiting sport mode...")
                    result = self.sport_client.StopMove()
                    print(f"StopMove result: {result}")
                    self.sport_mode_active = False
            else:
                print("\nCannot enter sport mode - robot must be standing first!")
                
        elif key == '0':
            # Stop all movement
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            self.strafe_vel = 0.0
            if self.sport_mode_active:
                result = self.sport_client.StopMove()
                print(f"\nStop movement result: {result}")
                
        elif key == '1':
            # Balance stand
            if self.is_standing:
                result = self.sport_client.BalanceStand()
                print(f"\nBalanceStand result: {result}")
                self.sport_mode_active = True
                
        elif key == '2':
            # Recovery stand
            result = self.sport_client.RecoveryStand()
            print(f"\nRecoveryStand result: {result}")
            self.is_standing = True
            
        elif key == '3':
            # Damping mode
            result = self.sport_client.Damp()
            print(f"\nDamp result: {result}")
            self.sport_mode_active = False
            
        elif key == '\x1b':  # ESC key
            self.running = False
            
        # Movement controls (only when in sport mode)
        elif self.sport_mode_active:
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
                
        # Body pose controls (only when standing)
        elif self.is_standing:
            if key == 'r':
                self.body_height = min(self.body_height + self.height_increment, self.max_body_height)
                # Note: body height may not be available in all SDK versions
                print(f"\nBody height: {self.body_height:.2f}")
            elif key == 'f':
                self.body_height = max(self.body_height - self.height_increment, -self.max_body_height)
                print(f"\nBody height: {self.body_height:.2f}")
            elif key == 't':
                self.body_pitch = min(self.body_pitch + self.tilt_increment, self.max_body_tilt)
                result = self.sport_client.Euler(self.body_roll, self.body_pitch, 0.0)
                print(f"\nEuler result: {result}")
            elif key == 'g':
                self.body_pitch = max(self.body_pitch - self.tilt_increment, -self.max_body_tilt)
                result = self.sport_client.Euler(self.body_roll, self.body_pitch, 0.0)
                print(f"\nEuler result: {result}")
            elif key == 'y':
                self.body_roll = min(self.body_roll + self.tilt_increment, self.max_body_tilt)
                result = self.sport_client.Euler(self.body_roll, self.body_pitch, 0.0)
                print(f"\nEuler result: {result}")
            elif key == 'h':
                self.body_roll = max(self.body_roll - self.tilt_increment, -self.max_body_tilt)
                result = self.sport_client.Euler(self.body_roll, self.body_pitch, 0.0)
                print(f"\nEuler result: {result}")
                
    def send_movement_command(self):
        """Send movement command to robot"""
        if self.sport_mode_active and (self.linear_vel != 0 or self.strafe_vel != 0 or self.angular_vel != 0):
            # Send move command with current velocities
            result = self.sport_client.Move(self.linear_vel, self.strafe_vel, self.angular_vel)
            
    def print_status(self):
        """Print current status"""
        print(f"\rStand: {self.is_standing} | Sport: {self.sport_mode_active} | "
              f"Lin: {self.linear_vel:+.2f} | Str: {self.strafe_vel:+.2f} | "
              f"Ang: {self.angular_vel:+.2f} | H: {self.body_height:+.2f} | "
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
            
        # Initialize sport client
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        
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
                
                # Send movement commands continuously when in sport mode
                self.send_movement_command()
                
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
            # Stop robot before exiting
            if self.sport_client and self.sport_mode_active:
                try:
                    self.sport_client.StopMove()
                    print("Stopped robot movement")
                except:
                    pass
            self.restore_terminal()
            print("\nTeleoperation stopped.")

def main():
    print("Starting Unitree Go2 Sport Mode Keyboard Teleoperation...")
    print("Make sure the simulator is running!")
    print("Usage: python3 keyboard_teleop.py [network_interface]")
    print("  Without arguments: Uses simulation mode")
    print("  With interface: Uses real robot (e.g., enp3s0)")
    print("Press H for help during operation.")
    
    teleop = KeyboardTeleop()
    teleop.run()

if __name__ == '__main__':
    main()
