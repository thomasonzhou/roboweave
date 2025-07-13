#!/usr/bin/env python3
"""
Shared Robot Agent Module

This module provides a singleton robot agent that can be shared between
the MCP server and HTTP bridge, ensuring the MuJoCo viewer runs in the main thread.
"""

import asyncio
import pathlib
import signal
import sys
import time
from typing import Optional

import mujoco
from mujoco_mpc import agent as agent_lib

class RobotAgent:
    """Singleton robot agent for MuJoCo control."""
    
    _instance: Optional['RobotAgent'] = None
    _agent = None
    _initialized = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        if not hasattr(self, '_setup_done'):
            self._setup_done = True
    
    def _get_model_path(self):
        """Get the path to the robot model."""
        return (
            pathlib.Path(__file__).parent.parent
            / "robot/tasks/quadruped/task_flat.xml"
        )
    
    def initialize(self):
        """Initialize the robot agent in the main thread."""
        if self._initialized:
            return self._agent
            
        print("Initializing robot agent...")
        model_path = self._get_model_path()
        model = mujoco.MjModel.from_xml_path(str(model_path))
        
        self._agent = agent_lib.Agent(
            server_binary_path=pathlib.Path(agent_lib.__file__).parent
            / "mjpc"
            / "ui_agent_server",
            task_id="Quadruped Flat",
            model=model,
            extra_flags=["--planner_enabled"]  # Enable interactive viewer
        )
        
        # Initialize the agent context
        self._agent.__enter__()
        self._initialized = True
        
        print("Robot agent initialized with MuJoCo viewer")
        return self._agent
    
    def get_agent(self):
        """Get the agent instance."""
        if not self._initialized:
            return self.initialize()
        return self._agent
    
    def cleanup(self):
        """Clean up the agent when shutting down."""
        if self._initialized and self._agent:
            try:
                self._agent.__exit__(None, None, None)
                print("Robot agent cleaned up")
                self._initialized = False
            except Exception as e:
                print(f"Error cleaning up agent: {e}")

# Global singleton instance
robot_agent = RobotAgent()

def get_robot_agent():
    """Get the global robot agent instance."""
    return robot_agent

def cleanup_robot_agent():
    """Clean up the global robot agent."""
    robot_agent.cleanup()
