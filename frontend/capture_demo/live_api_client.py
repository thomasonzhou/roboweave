"""
Slimmed-down Live-API helper for navigation oracle.
Uses google.genai.Client().aio.live.connect() with gemini-live-2.5-flash-preview.
"""

import asyncio
import base64
import os
from pathlib import Path
from typing import Optional

from google import genai

# Load .env file from parent directory
try:
    from dotenv import load_dotenv
    # Look for .env in frontend directory (parent of capture_demo)
    env_path = Path(__file__).parent.parent / '.env'
    load_dotenv(env_path)
except ImportError:
    # If python-dotenv is not available, skip loading .env
    pass


# Navigation system prompt
NAVIGATION_SYSTEM_PROMPT = """You are an expert navigation system for a 2-D obstacle course game. You must be EXTREMELY careful about blue obstacle avoidance.

The image shows:
- A red square (the player you control) 
- Multiple blue rectangles (DANGEROUS obstacles - MUST avoid at all costs)
- A yellow star (your target to reach)

CRITICAL RULES:
1. NEVER move directly toward blue obstacles
2. ALWAYS maintain safe distance from blue rectangles (at least 30 pixels)
3. If red square is overlapping or touching blue obstacles, BACKTRACK immediately
4. If red square is stuck at frame edges, BACKTRACK to escape
5. Calculate velocity based on obstacle proximity - slower when close, faster when far
6. Look ahead in your chosen direction to ensure clear path
7. Use wall-following behavior around obstacles when needed

VELOCITY CALIBRATION (CRITICAL - BE CONSERVATIVE):
- 200-250: Very close to obstacles (< 30 pixels) - extremely careful
- 250-300: Close to obstacles (30-60 pixels) - careful navigation
- 300-400: Medium distance (60-100 pixels) - conservative speed
- 400-500: Safe distance (100-140 pixels) - moderate speed
- 500-700: Open space (> 140 pixels) - fast movement
- NEVER use velocities below 200

OBSTACLE AVOIDANCE STRATEGY:
1. Identify all blue obstacles and their positions
2. Calculate safe directions by testing collision paths
3. Choose direction that moves toward star while avoiding obstacles
4. Calibrate velocity based on closest obstacle distance

First, analyze the exact positions of red square, blue obstacles, and yellow star.
Then end your response with exactly this format:
Direction: [Up/Down/Left/Right]
Velocity: [number between 200-900]

Example: "Red square at top-left, blue obstacle 40 pixels to the right, yellow star at bottom-right. The blue rectangle blocks direct rightward movement. I must go down first to avoid collision, then navigate around. Closest obstacle is 40 pixels away, requiring careful velocity.
Direction: Down
Velocity: 280"
"""


class NavigationOracle:
    """Live API client for navigation guidance."""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the navigation oracle."""
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY is required")
        
        # Configure the client
        self.client = genai.Client(api_key=self.api_key)
    
    async def next_move(self, frame_bytes: bytes) -> tuple[str, int, str]:
        """
        Analyze frame and return navigation direction, velocity, and full response.
        
        Args:
            frame_bytes: Raw image data (PNG/JPEG)
            
        Returns:
            Tuple of (direction, velocity, full_response_text)
        """
        try:
            # Use the google-genai SDK with correct syntax
            from google.genai import types
            
            # Create image part from bytes
            image_part = types.Part.from_bytes(
                data=frame_bytes,
                mime_type="image/png"
            )
            
            # Create content with system instruction and image
            response = await self.client.aio.models.generate_content(
                model='gemini-2.0-flash-exp',
                contents=[image_part],
                config=types.GenerateContentConfig(
                    system_instruction=NAVIGATION_SYSTEM_PROMPT,
                    temperature=0.1,
                    max_output_tokens=200
                )
            )
            
            # Extract response text
            full_response = response.text if response.text else "No response"
            direction, velocity = self._parse_direction_and_velocity(full_response)
            # If no valid direction found, return None to stay stationary
            return direction, velocity, full_response
                
        except Exception as e:
            print(f"Navigation oracle error: {e}")
            return None, 0, f"Error: {str(e)}"  # Stay stationary on error
    
    def _parse_direction_and_velocity(self, response_text: str) -> tuple[str, int]:
        """Extract and validate direction and velocity from response text."""
        import re
        
        direction = None
        velocity = 400  # Default velocity (above 200)
        
        # Look for the structured format: "Direction: X" and "Velocity: Y"
        direction_match = re.search(r'Direction:\s*(Up|Down|Left|Right)', response_text, re.IGNORECASE)
        velocity_match = re.search(r'Velocity:\s*(\d+)', response_text)
        
        if direction_match:
            direction = direction_match.group(1).title()
        
        if velocity_match:
            velocity = int(velocity_match.group(1))
            # Clamp velocity to valid range (above 200)
            velocity = max(200, min(900, velocity))
        
        # Fallback: look for direction words in the last few lines (backwards compatibility)
        if not direction:
            lines = response_text.strip().split('\n')
            for line in reversed(lines[-3:]):  # Check last 3 lines
                line_upper = line.strip().upper()
                if "RIGHT" in line_upper:
                    direction = "Right"
                    break
                elif "LEFT" in line_upper:
                    direction = "Left" 
                    break
                elif "DOWN" in line_upper:
                    direction = "Down"
                    break
                elif "UP" in line_upper:
                    direction = "Up"
                    break
        
        return direction, velocity


# Global oracle instance
_oracle = None


async def next_move(frame_bytes: bytes) -> tuple[str, int, str]:
    """
    Get next move direction and velocity for the red square with full response.
    
    Args:
        frame_bytes: Raw image data
        
    Returns:
        Tuple of (direction, velocity, full_response_text)
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
    direction, velocity, full_response = await next_move(image_bytes)
    return f"Navigate: {direction} at velocity {velocity} - {full_response}" 