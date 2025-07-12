"""
Live API client for Google Gemini Vision analysis.
Provides VisionSession and analyze function for image analysis.
"""

import asyncio
import base64
import os
from typing import Optional
import google.generativeai as genai
from google.generativeai.types import HarmCategory, HarmBlockThreshold


class VisionSession:
    """Session for Google Gemini Vision analysis."""
    
    def __init__(self, api_key: Optional[str] = None):
        """Initialize the vision session."""
        self.api_key = api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("GEMINI_API_KEY is required")
        
        # Configure the API
        genai.configure(api_key=self.api_key)
        
        # System instruction for RoboWeave Vision Coach
        self.system_instruction = """You are "RoboWeave Vision Coach".
When you receive an image, perform two tasks in plain text:
① Succinctly describe what is visible.
② Recommend one concrete next step an engineer should take to advance the project.
If a data-or metric-driven plot would help, call the tool "render_altair" and pass a vega-lite spec as a JSON string in the arg `json_graph`.
Otherwise answer normally."""
        
        # Initialize the model
        self.model = genai.GenerativeModel(
            'gemini-1.5-flash',
            system_instruction=self.system_instruction,
            safety_settings={
                HarmCategory.HARM_CATEGORY_HATE_SPEECH: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_HARASSMENT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_SEXUALLY_EXPLICIT: HarmBlockThreshold.BLOCK_NONE,
                HarmCategory.HARM_CATEGORY_DANGEROUS_CONTENT: HarmBlockThreshold.BLOCK_NONE,
            }
        )
    
    async def analyze_image(self, image_bytes: bytes, prompt: str) -> str:
        """Analyze an image with the given prompt."""
        try:
            # Convert image bytes to base64
            image_data = base64.b64encode(image_bytes).decode('utf-8')
            
            # Create image part
            image_part = {
                "mime_type": "image/png",
                "data": image_data
            }
            
            # Generate response
            response = await asyncio.to_thread(
                self.model.generate_content,
                [prompt, image_part]
            )
            
            return response.text
            
        except Exception as e:
            raise RuntimeError(f"Vision analysis failed: {str(e)}")


# Global session instance
_session = None


async def analyze(image_bytes: bytes, prompt: str) -> str:
    """
    Analyze an image using Google Gemini Vision.
    
    Args:
        image_bytes: Raw image data
        prompt: Analysis prompt
        
    Returns:
        Analysis result as string
    """
    global _session
    
    if _session is None:
        _session = VisionSession()
    
    return await _session.analyze_image(image_bytes, prompt) 