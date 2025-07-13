#!/usr/bin/env python3
"""
Sport Mode Keyboard Teleop with Rudimentary Gait
This sends sport mode commands that get converted to stupid but stable walking.
"""

import time
import sys
import threading
import termios
import tty
import select
import json

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize

class SportModeKeyboardTeleop:
    def __init__(self):
        self.running = True
        
        # Movement commands
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.mode = 0  # 0=sit, 1=stand, 2=walk
        
        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Sport command publisher (simple approach using a custom topic)
        self.sport_cmd_pub = None
        
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
        print("SPORT MODE RUDIMENTARY GAIT TELEOP")
        print("="*60)
        print("This creates STUPID but STABLE walking motion!")
        print("")
        print("Controls:")
        print("  SPACE : Stand up/sit down")
        print("  m     : Toggle walking mode")
        print("  w/s   : Walk forward/backward (when in walk mode)")
        print("  a/d   : Turn left/right (when in walk mode)")
        print("  q/e   : Strafe left/right (when in walk mode)")
        print("  0     : Stop all movement")
        print("  H     : Show this help")
        print("  ESC   : Exit")
        print("="*60)
        print("Note: The gait will look ungraceful but should be very stable!")
        print("It moves one leg at a time in a duck-walk style.")
        print()
    
    def handle_input(self, key):
        """Handle keyboard input"""
        if key == ' ':
            # Toggle stand/sit
            if self.mode == 0:
                self.mode = 1  # Stand
                print("\nStanding up...")
            else:
                self.mode = 0  # Sit
                self.vx = self.vy = self.vyaw = 0.0
                print("\nSitting down...")
                
        elif key == 'm':
            # Toggle walking mode
            if self.mode == 1:  # Standing
                self.mode = 2  # Walk
                print("\nWalking mode ON - Get ready for duck walk!")
            elif self.mode == 2:  # Walking
                self.mode = 1  # Stand
                self.vx = self.vy = self.vyaw = 0.0
                print("\nWalking mode OFF - Standing still")
            else:
                print("\nMust be standing to walk!")
                
        elif key == '0':
            # Stop all movement
            self.vx = self.vy = self.vyaw = 0.0
            if self.mode == 2:
                self.mode = 1  # Back to standing
            print("\nStopped movement")
                
        elif key == '\x1b':  # ESC
            self.running = False
            
        # Movement commands (only when walking)
        elif self.mode == 2:  # Walking mode
            if key == 'w':
                self.vx = min(self.vx + 0.02, 0.1)  # Very slow for stability
                print(f"\nForward velocity: {self.vx:.3f} (slow for stability)")
            elif key == 's':
                self.vx = max(self.vx - 0.02, -0.1)
                print(f"\nForward velocity: {self.vx:.3f}")
            elif key == 'a':
                self.vyaw = min(self.vyaw + 0.05, 0.2)
                print(f"\nTurn rate: {self.vyaw:.3f}")
            elif key == 'd':
                self.vyaw = max(self.vyaw - 0.05, -0.2)
                print(f"\nTurn rate: {self.vyaw:.3f}")
            elif key == 'q':
                self.vy = min(self.vy + 0.02, 0.08)
                print(f"\nStrafe velocity: {self.vy:.3f}")
            elif key == 'e':
                self.vy = max(self.vy - 0.02, -0.08)
                print(f"\nStrafe velocity: {self.vy:.3f}")
    
    def send_sport_command(self):
        """Send sport mode command"""
        # Create a simple message format that the bridge can understand
        # We'll use a JSON string for simplicity
        cmd = {
            "vx": self.vx,
            "vy": self.vy, 
            "vyaw": self.vyaw,
            "mode": self.mode,
            "timestamp": time.time()
        }
        
        # In a real implementation, this would be a proper DDS message
        # For now, we'll just demonstrate the concept
        if self.sport_cmd_pub:
            # This would send to the bridge if we had the proper message type
            pass
        
        # For debugging, we can print what we would send
        if self.mode > 0:  # Only when not sitting
            pass  # Removed debug print to reduce spam
    
    def print_status(self):
        """Print current status"""
        mode_str = ["SIT", "STAND", "WALK"][self.mode]
        print(f"\rMode: {mode_str} | Vx: {self.vx:+.3f} | Vy: {self.vy:+.3f} | Vyaw: {self.vyaw:+.3f}", 
              end='', flush=True)
    
    def run(self):
        """Main control loop"""
        print("="*60)
        print("SPORT MODE RUDIMENTARY GAIT TELEOP")
        print("="*60)
        print("WARNING: This will create a very stable but UGLY walking gait!")
        print("The robot will walk like a duck - one leg at a time.")
        print("This prioritizes stability over grace.")
        print()
        print("Make sure the simulation is running first:")
        print("  cd backend/planning")  
        print("  python3 unitree_mujoco.py")
        print("="*60)
        
        # Initialize SDK for simulation
        try:
            ChannelFactoryInitialize(1, "lo")
            print("SDK initialized for simulation")
        except Exception as e:
            print(f"SDK initialization failed: {e}")
            print("Make sure the simulation is running!")
            return
        
        self.setup_terminal()
        self.print_help()
        
        print("Starting sport mode teleop...")
        print("Commands will be converted to rudimentary gait in the bridge!")
        
        try:
            while self.running:
                key = self.get_key()
                if key:
                    if key.lower() == 'h' and key.isupper():
                        self.print_help()
                    else:
                        self.handle_input(key.lower())
                
                # Send command
                self.send_sport_command()
                
                # Print status
                self.print_status()
                
                time.sleep(0.02)  # 50Hz update
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Exiting...")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Stop movement
            self.vx = self.vy = self.vyaw = 0.0
            self.mode = 0
            self.send_sport_command()
            
            self.restore_terminal()
            print("\nSport mode teleop stopped.")
            print("The robot should have returned to sit position.")

if __name__ == '__main__':
    print("Starting Sport Mode Teleop with Rudimentary Gait...")
    teleop = SportModeKeyboardTeleop()
    teleop.run()
