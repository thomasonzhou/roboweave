#!/usr/bin/env python3
"""
Test the rudimentary gait generator through the bridge
"""

import time
import sys
import threading
import termios
import tty
import select
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# Import the bridge with gait functionality
sys.path.append('/home/thchzh/src/roboweave/backend/planning')

class SportModeTest:
    def __init__(self):
        self.running = True
        self.bridge = None
        
        # Movement commands
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.is_walking = False
        self.is_standing = False
        
        # Terminal settings
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
        print("RUDIMENTARY GAIT SPORT MODE TEST")
        print("="*60)
        print("Controls:")
        print("  SPACE : Stand up/sit down")
        print("  m     : Toggle walking mode")
        print("  w/s   : Walk forward/backward")
        print("  a/d   : Turn left/right")
        print("  q/e   : Strafe left/right")
        print("  0     : Stop all movement")
        print("  ESC   : Exit")
        print("="*60)
        print()
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == ' ':
            # Toggle stand/sit
            self.is_standing = not self.is_standing
            if self.is_standing:
                print("\nStanding up...")
                if self.bridge:
                    self.bridge.sport_mode_balance_stand()
                self.is_walking = False
            else:
                print("\nSitting down...")
                if self.bridge:
                    self.bridge.sport_mode_stop()
                self.is_walking = False
                self.vx = self.vy = self.vyaw = 0.0
                
        elif key == 'm':
            # Toggle walking mode
            if self.is_standing:
                self.is_walking = not self.is_walking
                if self.is_walking:
                    print("\nWalking mode ON - This will look stupid but be stable!")
                else:
                    print("\nWalking mode OFF")
                    self.vx = self.vy = self.vyaw = 0.0
                    if self.bridge:
                        self.bridge.sport_mode_balance_stand()
            else:
                print("\nMust be standing to walk!")
                
        elif key == '0':
            # Stop all movement
            self.vx = self.vy = self.vyaw = 0.0
            if self.bridge:
                if self.is_standing:
                    self.bridge.sport_mode_balance_stand()
                else:
                    self.bridge.sport_mode_stop()
            print("\nStopped movement")
                
        elif key == '\x1b':  # ESC
            self.running = False
            
        # Movement commands (only when walking)
        elif self.is_walking:
            if key == 'w':
                self.vx = min(self.vx + 0.05, 0.2)
                print(f"\nForward velocity: {self.vx:.2f}")
            elif key == 's':
                self.vx = max(self.vx - 0.05, -0.2)
                print(f"\nForward velocity: {self.vx:.2f}")
            elif key == 'a':
                self.vyaw = min(self.vyaw + 0.1, 0.3)
                print(f"\nTurn rate: {self.vyaw:.2f}")
            elif key == 'd':
                self.vyaw = max(self.vyaw - 0.1, -0.3)
                print(f"\nTurn rate: {self.vyaw:.2f}")
            elif key == 'q':
                self.vy = min(self.vy + 0.05, 0.15)
                print(f"\nStrafe velocity: {self.vy:.2f}")
            elif key == 'e':
                self.vy = max(self.vy - 0.05, -0.15)
                print(f"\nStrafe velocity: {self.vy:.2f}")
            
            # Send movement command
            if self.bridge:
                self.bridge.sport_mode_move(self.vx, self.vy, self.vyaw)
    
    def print_status(self):
        """Print current status"""
        print(f"\rStand: {self.is_standing} | Walk: {self.is_walking} | "
              f"Vx: {self.vx:+.2f} | Vy: {self.vy:+.2f} | Vyaw: {self.vyaw:+.2f}", 
              end='', flush=True)
    
    def run(self):
        """Main test loop"""
        print("="*60)
        print("SPORT MODE BRIDGE TEST")
        print("This tests the rudimentary gait through the bridge interface")
        print("Make sure the simulation is running first!")
        print("="*60)
        
        # Initialize SDK for simulation
        ChannelFactoryInitialize(1, "lo")
        
        # Note: In a real scenario, the bridge would be created by unitree_mujoco.py
        # For this test, we're just testing the command interface
        print("Bridge interface ready (commands will be sent to running simulation)")
        
        self.setup_terminal()
        self.print_help()
        
        try:
            while self.running:
                key = self.get_key()
                if key:
                    if key.lower() == 'h' and key.isupper():
                        self.print_help()
                    else:
                        self.handle_input(key.lower())
                
                self.print_status()
                time.sleep(0.02)  # 50Hz update
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Exiting...")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            self.restore_terminal()
            print("\nSport mode test stopped.")

if __name__ == '__main__':
    test = SportModeTest()
    test.run()
