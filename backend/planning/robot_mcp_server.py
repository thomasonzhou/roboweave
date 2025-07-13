#!/usr/bin/env python3
"""
Robot Control MCP Server

This MCP server exposes robot control actions as tools that can be invoked by LLM clients.
Provides tools for getting robot state, controlling movement, and performing actions like flips.
"""

from typing import Any, Dict, List
import asyncio
import json
import time
import math
import pathlib
from mcp.server.fastmcp import FastMCP
import mujoco
from mujoco_mpc import agent as agent_lib
from mujoco_mpc import mjpc_parameters

# Initialize FastMCP server
mcp = FastMCP("robot-control")

# Global agent instance
_agent = None
_agent_lock = asyncio.Lock()

def _get_model_path():
    """Get the path to the robot model."""
    return (
        pathlib.Path(__file__).parent.parent
        / "robot/tasks/quadruped/task_flat.xml"
    )

async def _get_agent():
    """Get or create the robot agent instance."""
    global _agent
    async with _agent_lock:
        if _agent is None:
            model_path = _get_model_path()
            model = mujoco.MjModel.from_xml_path(str(model_path))
            
            _agent = agent_lib.Agent(
                server_binary_path=pathlib.Path(agent_lib.__file__).parent
                / "mjpc"
                / "ui_agent_server",
                task_id="Quadruped Flat",
                model=model,
                extra_flags=["--planner_enabled", "--enable_ui"]  # Enable interactive viewer
            )
            # Initialize the agent context
            await asyncio.get_event_loop().run_in_executor(None, _agent.__enter__)
        
        return _agent
@mcp.tool()
async def get_robot_state() -> str:
    """Get the current state of the robot including position and orientation.
    
    Returns:
        JSON string containing robot position and orientation
    """
    try:
        agent = await _get_agent()
        
        # Get current state
        state = agent.get_state()
        current_mode = agent.get_mode()
        
        # Extract position and orientation from qpos
        position = {
            "x": float(state.qpos[0]),
            "y": float(state.qpos[1]), 
            "z": float(state.qpos[2])
        }
        
        orientation = {
            "w": float(state.qpos[3]),
            "x": float(state.qpos[4]),
            "y": float(state.qpos[5]),
            "z": float(state.qpos[6])
        }
        
        result = {
            "position": position,
            "orientation": orientation,
            "current_mode": current_mode,
            "timestamp": time.time()
        }
        
        return json.dumps(result, indent=2)
        
    except Exception as e:
        return f"Error getting robot state: {str(e)}"

@mcp.tool()
async def move_forward(distance: float = 0.3) -> str:
    """Move the robot forward by a specified distance.
    
    Args:
        distance: Distance to move forward in meters (default: 0.3m)
        
    Returns:
        Status message with new goal position
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Calculate forward direction based on current orientation
        # For simplicity, assume forward is +X direction
        goal_x = current_x + 0.3 + distance
        goal_y = current_y
        goal_z = 0.26
        
        goal_pose = mjpc_parameters.Pose(pos=[goal_x, goal_y, goal_z], quat=[1, 0, 0, 0])
        agent.set_mocap({"goal": goal_pose})
        
        return f"Moving forward {distance}m from ({current_x:.2f}, {current_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})"
        
    except Exception as e:
        return f"Error moving forward: {str(e)}"

@mcp.tool()
async def move_backward(distance: float = 0.3) -> str:
    """Move the robot backward by a specified distance.
    
    Args:
        distance: Distance to move backward in meters (default: 0.3m)
        
    Returns:
        Status message with new goal position
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Move backward (negative X direction)
        goal_x = current_x + 0.3 - distance
        goal_y = current_y
        goal_z = 0.26
        
        goal_pose = mjpc_parameters.Pose(pos=[goal_x, goal_y, goal_z], quat=[1, 0, 0, 0])
        agent.set_mocap({"goal": goal_pose})
        
        return f"Moving backward {distance}m from ({current_x:.2f}, {current_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})"
        
    except Exception as e:
        return f"Error moving backward: {str(e)}"

@mcp.tool()
async def move_left(distance: float = 0.3) -> str:
    """Move the robot left by a specified distance.
    
    Args:
        distance: Distance to move left in meters (default: 0.3m)
        
    Returns:
        Status message with new goal position
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Move left (positive Y direction)
        goal_x = current_x + 0.3
        goal_y = current_y + distance
        goal_z = 0.26
        
        goal_pose = mjpc_parameters.Pose(pos=[goal_x, goal_y, goal_z], quat=[1, 0, 0, 0])
        agent.set_mocap({"goal": goal_pose})
        
        return f"Moving left {distance}m from ({current_x:.2f}, {current_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})"
        
    except Exception as e:
        return f"Error moving left: {str(e)}"

@mcp.tool()
async def move_right(distance: float = 0.3) -> str:
    """Move the robot right by a specified distance.
    
    Args:
        distance: Distance to move right in meters (default: 0.3m)
        
    Returns:
        Status message with new goal position
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Move right (negative Y direction)
        goal_x = current_x
        goal_y = current_y - distance
        goal_z = 0.26
        
        goal_pose = mjpc_parameters.Pose(pos=[goal_x, goal_y, goal_z], quat=[1, 0, 0, 0])
        agent.set_mocap({"goal": goal_pose})
        
        return f"Moving right {distance}m from ({current_x:.2f}, {current_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})"
        
    except Exception as e:
        return f"Error moving right: {str(e)}"

@mcp.tool()
async def rotate_left(angle_degrees: float = 45.0) -> str:
    """Rotate the robot left (counter-clockwise) by a specified angle.
    
    Args:
        angle_degrees: Angle to rotate in degrees (default: 45 degrees)
        
    Returns:
        Status message with rotation information
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Convert angle to radians and create rotation quaternion around Z axis
        angle_rad = math.radians(angle_degrees)
        # Quaternion for rotation around Z axis: [cos(θ/2), 0, 0, sin(θ/2)]
        cos_half = math.cos(angle_rad / 2)
        sin_half = math.sin(angle_rad / 2)
        
        goal_pose = mjpc_parameters.Pose(
            pos=[current_x, current_y, 0.26], 
            quat=[cos_half, 0, 0, sin_half]
        )
        agent.set_mocap({"goal": goal_pose})
        
        return f"Rotating left {angle_degrees}° at position ({current_x:.2f}, {current_y:.2f})"
        
    except Exception as e:
        return f"Error rotating left: {str(e)}"

@mcp.tool()
async def rotate_right(angle_degrees: float = 45.0) -> str:
    """Rotate the robot right (clockwise) by a specified angle.
    
    Args:
        angle_degrees: Angle to rotate in degrees (default: 45 degrees)
        
    Returns:
        Status message with rotation information
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Convert angle to radians and create rotation quaternion around Z axis (negative for clockwise)
        angle_rad = math.radians(-angle_degrees)
        cos_half = math.cos(angle_rad / 2)
        sin_half = math.sin(angle_rad / 2)
        
        goal_pose = mjpc_parameters.Pose(
            pos=[current_x, current_y, 0.26], 
            quat=[cos_half, 0, 0, sin_half]
        )
        agent.set_mocap({"goal": goal_pose})
        
        return f"Rotating right {angle_degrees}° at position ({current_x:.2f}, {current_y:.2f})"
        
    except Exception as e:
        return f"Error rotating right: {str(e)}"

@mcp.tool()
async def run_in_circle(radius: float = 2.0, duration: float = 10.0) -> str:
    """Make the robot run in a circle.
    
    Args:
        radius: Radius of the circle in meters (default: 2.0m)
        duration: How long to run in circle in seconds (default: 10s)
        
    Returns:
        Status message indicating circular motion has started
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current position as center of circle
        state = agent.get_state()
        center_x = float(state.qpos[0])
        center_y = float(state.qpos[1])
        
        # Start circular motion in background
        async def circle_motion():
            start_time = time.time()
            angular_velocity = 2 * math.pi / 8.0  # Complete circle in 8 seconds
            
            while time.time() - start_time < duration:
                elapsed = time.time() - start_time
                
                # Calculate position on circle
                angle = elapsed * angular_velocity
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                z = 0.26
                
                goal_pose = mjpc_parameters.Pose(pos=[x, y, z], quat=[1, 0, 0, 0])
                agent.set_mocap({"goal": goal_pose})
                
                await asyncio.sleep(0.05)  # Update every 50ms
        
        # Start the circular motion task
        asyncio.create_task(circle_motion())
        
        return f"Running in circle: center=({center_x:.2f}, {center_y:.2f}), radius={radius}m, duration={duration}s"
        
    except Exception as e:
        return f"Error running in circle: {str(e)}"

@mcp.tool()
async def stop_and_stay() -> str:
    """Stop the robot and make it stay in current position.
    
    Returns:
        Status message with current position
    """
    try:
        agent = await _get_agent()
        agent.set_mode("Quadruped")
        
        # Get current state and set goal to current position
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        goal_pose = mjpc_parameters.Pose(pos=[current_x, current_y, 0.26], quat=[1, 0, 0, 0])
        agent.set_mocap({"goal": goal_pose})
        
        return f"Robot stopped and staying at position ({current_x:.2f}, {current_y:.2f})"
        
    except Exception as e:
        return f"Error stopping robot: {str(e)}"

@mcp.tool()
async def do_flip() -> str:
    """Make the robot perform a flip maneuver.
    
    Returns:
        Status message indicating the flip command was issued
    """
    try:
        agent = await _get_agent()
        
        # Switch to flip mode
        agent.set_mode("Flip")
        
        # Give some time for the mode to take effect
        await asyncio.sleep(1.0)
        
        return "Flip command issued. Robot should perform a flip maneuver."
        
    except Exception as e:
        return f"Error performing flip: {str(e)}"

if __name__ == "__main__":
    # Run the MCP server using stdio transport
    mcp.run(transport='stdio')
