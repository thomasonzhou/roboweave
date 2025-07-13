#!/usr/bin/env python3
"""
Standalone Robot Control HTTP Server

This script creates an HTTP server that directly controls the robot using the shared agent,
ensuring the MuJoCo viewer runs in the main thread for visibility.
"""

import asyncio
import json
import sys
import os
import signal
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
import threading

# Add the current directory to path
# sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import shared robot agent
from robot_agent import get_robot_agent, cleanup_robot_agent

# FastAPI app
app = FastAPI(
    title="Robot Control HTTP Server",
    description="HTTP server for direct robot control with MuJoCo viewer",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify actual frontend origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request models
class MoveRequest(BaseModel):
    distance: float = 0.3

class RotateRequest(BaseModel):
    angle_degrees: float = 45.0

class CircleRequest(BaseModel):
    radius: float = 2.0
    duration: float = 10.0

# Import the required dependencies
try:
    from mujoco_mpc import mjpc_parameters
    import math
except ImportError as e:
    print(f"Missing dependencies: {e}")
    sys.exit(1)

# Robot control functions (copied from MCP server)
async def get_robot_state_impl():
    """Get the current state of the robot."""
    try:
        agent = get_robot_agent().get_agent()
        
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
        
        return {
            "position": position,
            "orientation": orientation,
            "mode": current_mode
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting robot state: {str(e)}")

async def move_robot_impl(direction: str, distance: float = 0.3):
    """Move robot in specified direction."""
    try:
        agent = get_robot_agent().get_agent()
        
        # Get current position
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        current_z = float(state.qpos[2])
        
        # Calculate new position based on direction
        if direction == "forward":
            goal_x = current_x + distance
            goal_y = current_y
        elif direction == "backward":
            goal_x = current_x - distance
            goal_y = current_y
        elif direction == "left":
            goal_x = current_x
            goal_y = current_y + distance
        elif direction == "right":
            goal_x = current_x
            goal_y = current_y - distance
        else:
            raise HTTPException(status_code=400, detail=f"Invalid direction: {direction}")
        
        goal_z = current_z
        
        # Set goal position
        goal_pose = mjpc_parameters.Pose(pos=[goal_x, goal_y, goal_z], quat=[1, 0, 0, 0])
        agent.set_task_parameters("Goal", goal_pose)
        
        return f"Moving {direction} by {distance} meters to position ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f})"
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error moving robot: {str(e)}")

# API endpoints
@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "ok", "message": "Robot Control HTTP Server is running"}

@app.get("/robot/state")
async def get_robot_state():
    """Get current robot state"""
    return await get_robot_state_impl()

@app.post("/robot/move/forward")
async def move_forward(request: MoveRequest = MoveRequest()):
    """Move robot forward"""
    result = await move_robot_impl("forward", request.distance)
    return {"status": "success", "message": result}

@app.post("/robot/move/backward")
async def move_backward(request: MoveRequest = MoveRequest()):
    """Move robot backward"""
    result = await move_robot_impl("backward", request.distance)
    return {"status": "success", "message": result}

@app.post("/robot/move/left")
async def move_left(request: MoveRequest = MoveRequest()):
    """Move robot left"""
    result = await move_robot_impl("left", request.distance)
    return {"status": "success", "message": result}

@app.post("/robot/move/right")
async def move_right(request: MoveRequest = MoveRequest()):
    """Move robot right"""
    result = await move_robot_impl("right", request.distance)
    return {"status": "success", "message": result}

@app.post("/robot/rotate/left")
async def rotate_left(request: RotateRequest = RotateRequest()):
    """Rotate robot left"""
    try:
        agent = get_robot_agent().get_agent()
        
        # Get current position and orientation
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        current_z = float(state.qpos[2])
        
        # Current quaternion
        current_quat = [
            float(state.qpos[3]),  # w
            float(state.qpos[4]),  # x
            float(state.qpos[5]),  # y
            float(state.qpos[6])   # z
        ]
        
        # Convert angle to radians and create rotation around Z-axis
        angle_rad = math.radians(request.angle_degrees)
        
        # Simple rotation around Z-axis (yaw)
        # For now, just use a simple rotation quaternion
        cos_half = math.cos(angle_rad / 2)
        sin_half = math.sin(angle_rad / 2)
        
        # Rotation quaternion for Z-axis
        rot_quat = [cos_half, 0, 0, sin_half]  # [w, x, y, z]
        
        goal_pose = mjpc_parameters.Pose(
            pos=[current_x, current_y, current_z], 
            quat=rot_quat
        )
        agent.set_task_parameters("Goal", goal_pose)
        
        return {"status": "success", "message": f"Rotating left by {request.angle_degrees} degrees"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error rotating robot: {str(e)}")

@app.post("/robot/rotate/right")
async def rotate_right(request: RotateRequest = RotateRequest()):
    """Rotate robot right"""
    try:
        # Rotate right is just rotating left by negative angle
        request.angle_degrees = -request.angle_degrees
        return await rotate_left(request)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error rotating robot: {str(e)}")

@app.post("/robot/action/circle")
async def run_in_circle(request: CircleRequest = CircleRequest()):
    """Make robot run in a circle"""
    try:
        agent = get_robot_agent().get_agent()
        
        # Get current position as circle center
        state = agent.get_state()
        center_x = float(state.qpos[0])
        center_y = float(state.qpos[1])
        center_z = float(state.qpos[2])
        
        # Create circular motion by setting multiple waypoints
        radius = request.radius
        duration = request.duration
        num_points = 8  # Number of waypoints around the circle
        
        for i in range(num_points):
            angle = (2 * math.pi * i) / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = center_z
            
            goal_pose = mjpc_parameters.Pose(pos=[x, y, z], quat=[1, 0, 0, 0])
            agent.set_task_parameters("Goal", goal_pose)
            
            # Wait a bit between waypoints
            await asyncio.sleep(duration / num_points)
        
        return {"status": "success", "message": f"Completed circular motion with radius {radius}m"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error running in circle: {str(e)}")

@app.post("/robot/action/stop")
async def stop_and_stay():
    """Stop robot and maintain current position"""
    try:
        agent = get_robot_agent().get_agent()
        
        # Get current position
        state = agent.get_state()
        current_x = float(state.qpos[0])
        current_y = float(state.qpos[1])
        
        # Set goal to current position to stop movement
        goal_pose = mjpc_parameters.Pose(pos=[current_x + 0.3, current_y, 0.26], quat=[1, 0, 0, 0])
        agent.set_task_parameters("Goal", goal_pose)
        
        return {"status": "success", "message": "Robot stopped and maintaining position"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error stopping robot: {str(e)}")

@app.post("/robot/action/flip")
async def do_flip():
    """Make robot perform a flip"""
    try:
        agent = get_robot_agent().get_agent()
        
        # Switch to flip mode
        agent.set_mode("Flip")
        
        # Give some time for the mode to take effect
        await asyncio.sleep(1.0)
        
        return {"status": "success", "message": "Flip command issued. Robot should perform a flip maneuver."}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing flip: {str(e)}")

def signal_handler(signum, frame):
    """Handle shutdown signals."""
    print("Received shutdown signal, cleaning up...")
    cleanup_robot_agent()
    sys.exit(0)

def run_server_with_agent():
    """Run the HTTP server with agent initialized in main thread."""
    # Set up signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Initialize the agent first (this must happen in main thread for viewer)
        print("Initializing robot agent...")
        get_robot_agent().initialize()
        print("Agent initialized successfully!")
        
        # Start the HTTP server in a separate thread
        def run_uvicorn():
            uvicorn.run(app, host="0.0.0.0", port=8080, log_level="info")
        
        server_thread = threading.Thread(target=run_uvicorn, daemon=True)
        server_thread.start()
        
        print("HTTP server started on port 8080")
        print("MuJoCo viewer should now be visible!")
        
        # Keep main thread alive for the viewer
        while True:
            import time
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Error running server: {e}")
    finally:
        cleanup_robot_agent()

if __name__ == "__main__":
    run_server_with_agent()
