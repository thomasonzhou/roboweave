#!/usr/bin/env python3
"""
Gemini MCP Service

This service uses FastMCP to connect Gemini with the robot control MCP server.
It processes voice commands via text input and returns structured robot commands.
"""

import os
import json
import time
import asyncio
from typing import Any, Dict, List, Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import google.generativeai as genai
import weave
import httpx
from dotenv import load_dotenv
import httpx

# Load environment variables
load_dotenv()

# Configure Google API
genai.configure(api_key=os.getenv('GOOGLE_API_KEY'))

# Configure Weave for observability
weave.init(project_name=os.getenv('WANDB_PROJECT', 'roboweave'))

app = FastAPI(title="Gemini MCP Service", version="1.0.0")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[os.getenv('FRONTEND_URL', 'http://localhost:3000'), "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# FastMCP client configuration
ROBOT_MCP_URL = os.getenv('ROBOT_MCP_URL', 'http://localhost:3001')

class VoiceCommandRequest(BaseModel):
    transcript: str
    confidence: float = 1.0
    session_id: Optional[str] = None

class RobotCommand(BaseModel):
    action: str
    parameters: Optional[Dict[str, Any]] = None
    reasoning: Optional[str] = None
    confidence: float

class GeminiResponse(BaseModel):
    commands: List[RobotCommand]
    explanation: str
    timestamp: float
    processing_time: float
    session_id: str

# System instruction for Gemini
SYSTEM_INSTRUCTION = """You are RoboWeave Voice Controller, an AI assistant that converts natural language voice commands into structured robot control actions.

ROBOT CAPABILITIES:
- Movement: move_forward, move_backward, move_left, move_right (distance in meters, 0.1-2.0)
- Rotation: rotate_left, rotate_right (angle in degrees, 15-180)
- Actions: stop, flip, circle (radius 0.5-3.0m, duration 5-30s)

RESPONSE FORMAT (JSON only):
{
  "commands": [
    {
      "action": "move_forward",
      "parameters": {"distance": 0.5},
      "reasoning": "User requested forward movement",
      "confidence": 0.95
    }
  ],
  "explanation": "Moving the robot forward by 0.5 meters as requested."
}

GUIDELINES:
- Always respond with valid JSON in the exact format above
- Use safe parameters (small distances/angles for safety)
- Provide clear reasoning for each command
- If unclear, suggest a safe default action
- Support compound commands: "move forward then turn left"
- Default values: distance=0.3m, angle=45°, radius=2m, duration=10s

AVAILABLE ACTIONS:
- move_forward: Move robot forward (distance parameter)
- move_backward: Move robot backward (distance parameter)
- move_left: Move robot left (distance parameter)
- move_right: Move robot right (distance parameter)
- rotate_left: Rotate robot left (angle parameter)
- rotate_right: Rotate robot right (angle parameter)
- stop: Stop all robot movement
- flip: Perform a flip maneuver
- circle: Move in a circle (radius and duration parameters)

EXAMPLES:
"Move forward" → move_forward, distance: 0.3
"Turn around" → rotate_left, angle: 180
"Go backward a bit" → move_backward, distance: 0.2
"Do a flip" → flip
"Move in a circle" → circle, radius: 2, duration: 10
"Stop the robot" → stop"""

# Initialize Gemini model
model = genai.GenerativeModel(
    model_name='gemini-1.5-flash',
    system_instruction=SYSTEM_INSTRUCTION,
    generation_config=genai.types.GenerationConfig(
        temperature=0.1,
        max_output_tokens=1000,
        response_mime_type="application/json",
    )
)

@weave.op()
async def process_voice_command_with_gemini(transcript: str, confidence: float) -> Dict[str, Any]:
    """Process voice command using Gemini and return structured robot commands."""
    
    prompt = f"""
User voice command: "{transcript}"
Voice recognition confidence: {confidence}

Provide a JSON response with robot commands for this voice input.
"""
    
    try:
        response = await model.generate_content_async(prompt)
        response_text = response.text.strip()
        
        # Parse JSON response
        try:
            parsed_response = json.loads(response_text)
            return {
                "success": True,
                "data": parsed_response,
                "raw_response": response_text
            }
        except json.JSONDecodeError as e:
            return {
                "success": False,
                "error": f"Failed to parse JSON response: {str(e)}",
                "raw_response": response_text
            }
            
    except Exception as e:
        return {
            "success": False,
            "error": f"Gemini API error: {str(e)}"
        }

@app.post("/process-voice-command", response_model=GeminiResponse)
async def process_voice_command(request: VoiceCommandRequest):
    """Process a voice command and return structured robot commands."""
    
    import time
    start_time = time.time()
    
    # Generate session ID if not provided
    session_id = request.session_id or f"session-{int(time.time())}-{hash(request.transcript) % 10000}"
    
    try:
        # Process with Gemini
        result = await process_voice_command_with_gemini(request.transcript, request.confidence)
        
        if not result["success"]:
            raise HTTPException(status_code=500, detail=result.get("error", "Unknown error"))

        data = result["data"]
        
        # Convert to response format
        commands = []
        for cmd in data.get("commands", []):
            commands.append(RobotCommand(
                action=cmd.get("action", "stop"),
                parameters=cmd.get("parameters", {}),
                reasoning=cmd.get("reasoning", ""),
                confidence=cmd.get("confidence", 0.8)
            ))

        # Execute commands via MCP (actual robot control)
        mcp_results = await execute_robot_commands([cmd.dict() for cmd in commands])
        
        processing_time = time.time() - start_time
        
        # Add MCP execution results to the explanation
        execution_summary = []
        for result in mcp_results:
            if result["success"]:
                execution_summary.append(f"✅ {result['command']['action']}")
            else:
                execution_summary.append(f"❌ {result['command']['action']}: {result['mcp_result'].get('error', 'Failed')}")
        
        enhanced_explanation = data.get("explanation", "Command processed")
        if execution_summary:
            enhanced_explanation += f"\n\nExecution Results:\n" + "\n".join(execution_summary)

        response = GeminiResponse(
            commands=commands,
            explanation=enhanced_explanation,
            timestamp=time.time(),
            processing_time=processing_time,
            session_id=session_id
        )
        
        return response
        
    except Exception as e:
        processing_time = time.time() - start_time
        raise HTTPException(status_code=500, detail=f"Error processing command: {str(e)}")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Gemini MCP Service",
        "timestamp": time.time(),
        "google_api_configured": bool(os.getenv('GOOGLE_API_KEY')),
        "weave_project": os.getenv('WANDB_PROJECT', 'roboweave')
    }

# MCP Robot Client Configuration
MCP_ROBOT_URL = "http://localhost:8080"

async def call_robot_mcp_tool(tool_name: str, arguments: Dict[str, Any] = None) -> Dict[str, Any]:
    """Call a robot MCP tool via HTTP bridge."""
    if arguments is None:
        arguments = {}
    
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(
                f"{MCP_ROBOT_URL}/call_tool",
                json={
                    "name": tool_name,
                    "arguments": arguments
                },
                timeout=30.0
            )
            
            if response.status_code == 200:
                result = response.json()
                return {
                    "success": True,
                    "result": result.get("content", [{}])[0].get("text", ""),
                    "tool": tool_name,
                    "arguments": arguments
                }
            else:
                return {
                    "success": False,
                    "error": f"MCP call failed: {response.status_code} {response.text}",
                    "tool": tool_name,
                    "arguments": arguments
                }
                
    except Exception as e:
        return {
            "success": False,
            "error": f"MCP call exception: {str(e)}",
            "tool": tool_name,
            "arguments": arguments
        }

async def execute_robot_commands(commands: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Execute robot commands via MCP and return results."""
    results = []
    
    for command in commands:
        action = command.get("action", "")
        parameters = command.get("parameters", {})
        
        # Map action to MCP tool name and arguments
        tool_call = None
        
        if action == "move_forward":
            tool_call = ("move_forward", {"distance": parameters.get("distance", 0.3)})
        elif action == "move_backward":
            tool_call = ("move_backward", {"distance": parameters.get("distance", 0.3)})
        elif action == "move_left":
            tool_call = ("move_left", {"distance": parameters.get("distance", 0.3)})
        elif action == "move_right":
            tool_call = ("move_right", {"distance": parameters.get("distance", 0.3)})
        elif action == "rotate_left":
            tool_call = ("rotate_left", {"angle_degrees": parameters.get("angle", 45.0)})
        elif action == "rotate_right":
            tool_call = ("rotate_right", {"angle_degrees": parameters.get("angle", 45.0)})
        elif action == "stop":
            tool_call = ("stop_and_stay", {})
        elif action == "flip":
            tool_call = ("do_flip", {})
        elif action == "circle":
            tool_call = ("run_in_circle", {
                "radius": parameters.get("radius", 2.0),
                "duration": parameters.get("duration", 10.0)
            })
        
        if tool_call:
            tool_name, tool_args = tool_call
            result = await call_robot_mcp_tool(tool_name, tool_args)
            results.append({
                "command": command,
                "mcp_result": result,
                "success": result.get("success", False)
            })
        else:
            results.append({
                "command": command,
                "mcp_result": {"success": False, "error": f"Unknown action: {action}"},
                "success": False
            })
    
    return results

if __name__ == "__main__":
    import uvicorn
    
    port = int(os.getenv('PORT', 8000))
    host = os.getenv('HOST', 'localhost')
    debug = os.getenv('DEBUG', 'true').lower() == 'true'
    
    print(f"Starting Gemini MCP Service on {host}:{port}")
    print(f"Debug mode: {debug}")
    print(f"Google API Key configured: {'✓' if os.getenv('GOOGLE_API_KEY') else '✗'}")
    print(f"Weave Project: {os.getenv('WANDB_PROJECT', 'roboweave')}")
    
    uvicorn.run(
        "gemini_mcp_service:app",
        host=host,
        port=port,
        reload=debug,
        log_level="debug" if debug else "info"
    )
