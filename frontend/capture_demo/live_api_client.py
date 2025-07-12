"""
Slimmed-down Live-API helper for navigation oracle.
Uses google.genai.Client().aio.live.connect() with gemini-live-2.5-flash-preview.
"""

import asyncio
import base64
import os
from typing import Optional

import google.genai as genai


# Navigation oracle system prompt
NAVIGATION_SYSTEM_PROMPT = """You are a navigation oracle for a 2-D toy world.
The image shows a red agent square and one blue target square.
Return ONE WORD only — Up, Down, Left, or Right — that moves the red square closer to the blue.
No extra text."""


class NavigationOracle:
    """Live API client for navigation guidance."""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the navigation oracle."""
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY is required")
        
        # Configure the client
        self.client = genai.Client(api_key=self.api_key)
    
    async def next_move(self, frame_bytes: bytes) -> str:
        """
        Analyze frame and return navigation direction.
        
        Args:
            frame_bytes: Raw image data (PNG/JPEG)
            
        Returns:
            One of: "Up", "Down", "Left", "Right"
        """
        try:
            # Convert image bytes to base64
            image_b64 = base64.b64encode(frame_bytes).decode('utf-8')
            
            # Connect to Live API
            async with self.client.aio.live.connect(
                model="gemini-live-2.5-flash-preview",
                config={
                    "system_instruction": {
                        "parts": [{"text": NAVIGATION_SYSTEM_PROMPT}]
                    }
                }
            ) as session:
                
                # Send image
                await session.send({
                    "client_content": {
                        "turns": [{
                            "parts": [{
                                "inline_data": {
                                    "mime_type": "image/png",
                                    "data": image_b64
                                }
                            }]
                        }],
                        "turn_complete": True
                    }
                })
                
                # Wait for first text response
                async for response in session:
                    if hasattr(response, 'server_content') and response.server_content:
                        if hasattr(response.server_content, 'model_turn'):
                            model_turn = response.server_content.model_turn
                            if model_turn and hasattr(model_turn, 'parts'):
                                for part in model_turn.parts:
                                    if hasattr(part, 'text') and part.text:
                                        # Extract and validate direction
                                        direction = part.text.strip()
                                        return self._validate_direction(direction)
                
                # No valid response received
                return "Up"
                
        except Exception as e:
            print(f"Navigation oracle error: {e}")
            return "Up"  # Default fallback
    
    def _validate_direction(self, direction: str) -> str:
        """Validate and normalize direction response."""
        direction = direction.strip().title()  # Normalize case
        
        if direction in {"Up", "Down", "Left", "Right"}:
            return direction
        
        # Default fallback for invalid responses
        return "Up"


# Global oracle instance
_oracle = None


async def next_move(frame_bytes: bytes) -> str:
    """
    Get next move direction for the red square.
    
    Args:
        frame_bytes: Raw image data
        
    Returns:
        Direction string: "Up", "Down", "Left", or "Right"
    """
    global _oracle
    
    if _oracle is None:
        _oracle = NavigationOracle()
    
    return await _oracle.next_move(frame_bytes)


# Backward compatibility function (deprecated)
async def analyze(image_bytes: bytes, prompt: str) -> str:
    """
    Legacy function for backward compatibility.
    Now redirects to navigation oracle.
    """
    direction = await next_move(image_bytes)
    return f"Navigate: {direction}" 