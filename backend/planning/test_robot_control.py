#!/usr/bin/env python3
"""
Test script for Robot Control MCP Server

This script tests the robot control functionality without requiring the full frontend.
"""

import asyncio
import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_mcp_server import (
    get_robot_state,
    move_forward,
    move_backward,
    stop_and_stay
)

async def test_robot_control():
    """Test basic robot control functions."""
    print("🤖 Testing Robot Control MCP Server...")
    
    try:
        # Test getting robot state
        print("\n📊 Getting robot state...")
        state = await get_robot_state()
        print(f"State: {state}")
        
        # Test movement
        print("\n➡️ Moving forward...")
        result = await move_forward(0.2)
        print(f"Result: {result}")
        
        # Wait a moment
        await asyncio.sleep(2)
        
        # Test stopping
        print("\n⏹️ Stopping robot...")
        result = await stop_and_stay()
        print(f"Result: {result}")
        
        # Get final state
        print("\n📊 Final robot state...")
        state = await get_robot_state()
        print(f"State: {state}")
        
        print("\n✅ Robot control test completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Error during robot control test: {e}")
        return False
    
    return True

if __name__ == "__main__":
    print("🚀 Starting Robot Control Test...")
    print("⚠️  Make sure the MuJoCo viewer opens and robot is visible")
    
    # Run the test
    success = asyncio.run(test_robot_control())
    
    if success:
        print("\n🎉 All tests passed! Robot control system is working.")
    else:
        print("\n💥 Tests failed! Check the error messages above.")
        sys.exit(1)
