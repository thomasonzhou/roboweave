#!/usr/bin/env python3
"""
HTTP Bridge for Robot Control MCP Server

This script creates an HTTP server that bridges REST API calls to the MCP server,
allowing the frontend to communicate with the robot control server.
"""

import asyncio
import json
import sys
import os
from pathlib import Path
from typing import Any, Dict

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# Add the current directory to path so we can import the server
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import MCP server tools
try:
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
except ImportError as e:
    print(f"Failed to import MCP server tools: {e}")
    sys.exit(1)

# FastAPI app
app = FastAPI(
    title="Robot Control HTTP Bridge",
    description="HTTP bridge for the Robot Control MCP Server",
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

# Tool mapping
TOOL_FUNCTIONS = {
    "get_robot_state": get_robot_state,
    "move_forward": move_forward,
    "move_backward": move_backward,
    "move_left": move_left,
    "move_right": move_right,
    "rotate_left": rotate_left,
    "rotate_right": rotate_right,
    "run_in_circle": run_in_circle,
    "stop_and_stay": stop_and_stay,
    "do_flip": do_flip,
}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "ok", "message": "Robot Control HTTP Bridge is running"}

@app.get("/mcp/tools")
async def list_tools():
    """List available MCP tools"""
    tools = []
    for name, func in TOOL_FUNCTIONS.items():
        doc = func.__doc__ or "No description available"
        # Extract first line of docstring
        description = doc.split('\n')[0].strip()
        
        tools.append({
            "name": name,
            "description": description,
        })
    
    return {"tools": tools}

@app.post("/mcp/tools/{tool_name}")
async def call_tool(tool_name: str, params: Dict[str, Any] = None):
    """Call an MCP tool with parameters"""
    if tool_name not in TOOL_FUNCTIONS:
        raise HTTPException(status_code=404, detail=f"Tool '{tool_name}' not found")
    
    func = TOOL_FUNCTIONS[tool_name]
    
    try:
        if params:
            # Filter parameters to match function signature
            import inspect
            sig = inspect.signature(func)
            filtered_params = {k: v for k, v in params.items() if k in sig.parameters}
            result = await func(**filtered_params)
        else:
            result = await func()
        
        return {"success": True, "result": result}
    
    except Exception as e:
        return {"success": False, "error": str(e)}

# Specific endpoints for better API design
@app.get("/robot/state")
async def get_state():
    """Get current robot state"""
    try:
        result = await get_robot_state()
        return {"success": True, "data": json.loads(result)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/move/forward")
async def move_robot_forward(request: MoveRequest):
    """Move robot forward"""
    try:
        result = await move_forward(request.distance)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/move/backward")
async def move_robot_backward(request: MoveRequest):
    """Move robot backward"""
    try:
        result = await move_backward(request.distance)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/move/left")
async def move_robot_left(request: MoveRequest):
    """Move robot left"""
    try:
        result = await move_left(request.distance)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/move/right")
async def move_robot_right(request: MoveRequest):
    """Move robot right"""
    try:
        result = await move_right(request.distance)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/rotate/left")
async def rotate_robot_left(request: RotateRequest):
    """Rotate robot left"""
    try:
        result = await rotate_left(request.angle_degrees)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/rotate/right")
async def rotate_robot_right(request: RotateRequest):
    """Rotate robot right"""
    try:
        result = await rotate_right(request.angle_degrees)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/circle")
async def run_robot_in_circle(request: CircleRequest):
    """Make robot run in circle"""
    try:
        result = await run_in_circle(request.radius, request.duration)
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/stop")
async def stop_robot():
    """Stop robot and make it stay"""
    try:
        result = await stop_and_stay()
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/robot/flip")
async def flip_robot():
    """Make robot do a flip"""
    try:
        result = await do_flip()
        return {"success": True, "message": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    print("🚀 Starting Robot Control HTTP Bridge...")
    print("🔗 Frontend can connect to: http://localhost:8080")
    print("📚 API docs available at: http://localhost:8080/docs")
    print("🤖 MCP Bridge ready for robot control!")
    
    uvicorn.run(
        app, 
        host="0.0.0.0", 
        port=8080,
        log_level="info"
    )
