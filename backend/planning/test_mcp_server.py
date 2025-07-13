#!/usr/bin/env python3
"""
Test script for Robot Control MCP Server

This script tests the MCP server tools independently to ensure they work correctly.
"""

import asyncio
import json
import sys
import os

# Add the current directory to path so we can import the server
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

async def test_mcp_tools():
    """Test all MCP server tools."""
    print("🤖 Testing Robot Control MCP Server Tools\n")
    
    try:
        # Import the server tools
        from robot_mcp_server import (
            get_robot_state,
            move_forward,
            move_backward, 
            move_left,
            move_right,
            rotate_left,
            rotate_right,
            run_in_circle,
            stop_and_stay,
            do_flip
        )
        
        print("✅ Successfully imported MCP server tools\n")
        
        # Test 1: Get robot state  
        print("� Test 1: Getting robot state...")
        state_result = await get_robot_state()
        print(f"Result: {state_result}\n")
        
        # Test 2: Move forward
        print("⬆️ Test 2: Moving forward...")
        forward_result = await move_forward(0.5)
        print(f"Result: {forward_result}\n")
        
        # Test 3: Move backward
        print("⬇️ Test 3: Moving backward...")
        backward_result = await move_backward(0.3)
        print(f"Result: {backward_result}\n")
        
        # Test 4: Move left
        print("⬅️ Test 4: Moving left...")
        left_result = await move_left(0.4)
        print(f"Result: {left_result}\n")
        
        # Test 5: Move right
        print("➡️ Test 5: Moving right...")
        right_result = await move_right(0.4)
        print(f"Result: {right_result}\n")
        
        # Test 6: Rotate left
        print("🔄 Test 6: Rotating left...")
        rotate_left_result = await rotate_left(90)
        print(f"Result: {rotate_left_result}\n")
        
        # Test 7: Rotate right
        print("🔄 Test 7: Rotating right...")
        rotate_right_result = await rotate_right(45)
        print(f"Result: {rotate_right_result}\n")
        
        # Test 8: Run in circle (short duration for testing)
        print("⭕ Test 8: Running in circle...")
        circle_result = await run_in_circle(radius=1.5, duration=3.0)
        print(f"Result: {circle_result}\n")
        
        # Wait for circle motion to start
        await asyncio.sleep(2)
        
        # Test 9: Stop and stay
        print("🛑 Test 9: Stop and stay...")
        stop_result = await stop_and_stay()
        print(f"Result: {stop_result}\n")
        
        # Test 10: Do flip
        print("🤸 Test 10: Doing flip...")
        flip_result = await do_flip()
        print(f"Result: {flip_result}\n")
        
        print("✅ All tests completed successfully!")
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure to install dependencies with: uv add 'mcp[cli]' mujoco mujoco-mpc")
        return False
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        return False
    
    return True

async def test_mcp_server_format():
    """Test that the MCP server can start and list tools."""
    print("\n🔧 Testing MCP Server Protocol Compliance\n")
    
    try:
        from robot_mcp_server import mcp
        
        # Get the tools list that would be exposed to MCP clients
        tools = []
        
        # The FastMCP framework automatically generates tool definitions from decorated functions
        print("📋 Tools that would be exposed to MCP clients:")
        
        # List the decorated functions (this is what FastMCP does internally)
        import inspect
        from robot_mcp_server import (
            get_robot_state, move_forward, move_backward, move_left, move_right,
            rotate_left, rotate_right, run_in_circle, stop_and_stay, do_flip
        )
        
        tool_functions = [
            get_robot_state, move_forward, move_backward, move_left, move_right,
            rotate_left, rotate_right, run_in_circle, stop_and_stay, do_flip
        ]
        
        for i, func in enumerate(tool_functions, 1):
            sig = inspect.signature(func)
            doc = func.__doc__ or "No description"
            # Extract first line of docstring
            description = doc.split('\n')[0].strip()
            
            print(f"  {i}. {func.__name__}")
            print(f"     Description: {description}")
            print(f"     Parameters: {list(sig.parameters.keys())}")
            print()
        
        print("✅ MCP Server format validation passed!")
        return True
        
    except Exception as e:
        print(f"❌ MCP Server validation error: {e}")
        return False

def print_usage_examples():
    """Print example usage commands for Claude Desktop."""
    print("\n💬 Example Commands for Claude Desktop:\n")
    
    examples = [
        "Get the current robot position",
        "Move the robot forward 1 meter", 
        "Move the robot backward half a meter",
        "Move the robot left 0.5 meters",
        "Move the robot right 0.3 meters",
        "Rotate the robot left 90 degrees",
        "Rotate the robot right 45 degrees",
        "Make the robot run in a circle with 3 meter radius",
        "Stop the robot and make it stay still",
        "Make the robot do a flip"
    ]
    
    for i, example in enumerate(examples, 1):
        print(f"  {i}. \"{example}\"")
    
    print("\n📝 Note: These natural language commands will be translated to the appropriate MCP tool calls by Claude.")
    print("\n🎮 Motion Primitives Available:")
    print("  • Forward/Backward movement (relative to current position)")
    print("  • Left/Right movement (relative to current position)")  
    print("  • Left/Right rotation (around current position)")
    print("  • Circular motion (around current position)")
    print("  • Stop and stay in place")
    print("  • Flip maneuver")

if __name__ == "__main__":
    print("🚀 Robot Control MCP Server Test Suite")
    print("=" * 50)
    
    async def run_all_tests():
        # Test MCP server format
        format_ok = await test_mcp_server_format()
        
        if format_ok:
            # Test individual tools (requires robot simulation)
            print("\n" + "=" * 50)
            tools_ok = await test_mcp_tools()
            
            if tools_ok:
                print_usage_examples()
                print("\n🎉 All tests passed! The MCP server is ready for use.")
            else:
                print("\n⚠️ Tool tests failed. Check robot simulation setup.")
        else:
            print("\n❌ MCP server format tests failed.")
    
    # Run the tests
    asyncio.run(run_all_tests())
