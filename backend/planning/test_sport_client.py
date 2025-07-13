#!/usr/bin/env python3
"""
Test script to verify SportClient functionality
"""

import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def test_sport_client():
    """Test sport client initialization"""
    try:
        print("Initializing SDK...")
        ChannelFactoryInitialize(1, "lo")  # Simulation
        
        print("Creating SportClient...")
        sport_client = SportClient()
        sport_client.SetTimeout(10.0)
        sport_client.Init()
        
        print("SportClient initialized successfully!")
        
        # Test a simple command
        print("Testing BalanceStand command...")
        result = sport_client.BalanceStand()
        print(f"BalanceStand result: {result}")
        
        print("Testing Move command...")
        result = sport_client.Move(0.1, 0.0, 0.0)  # Move forward slowly
        print(f"Move result: {result}")
        
        print("Testing StopMove command...")
        result = sport_client.StopMove()
        print(f"StopMove result: {result}")
        
        print("All tests passed!")
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    
    return True

if __name__ == '__main__':
    print("Testing SportClient functionality...")
    success = test_sport_client()
    if success:
        print("SportClient test completed successfully!")
    else:
        print("SportClient test failed!")
        sys.exit(1)
