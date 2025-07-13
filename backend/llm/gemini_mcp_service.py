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
from dotenv import load_dotenv

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
        
        processing_time = time.time() - start_time
        
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
        
        response = GeminiResponse(
            commands=commands,
            explanation=data.get("explanation", "Command processed"),
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
