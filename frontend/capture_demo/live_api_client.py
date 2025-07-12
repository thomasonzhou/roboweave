"""
Simple Live-API helper for navigation oracle with W&B Weave observability.
Following the clean pattern from test.ipynb
"""

import os
import json
import base64
from pathlib import Path
from typing import Optional, Tuple

from google import genai
import weave
from pydantic import BaseModel, Field

# Load .env file from parent directory
try:
    from dotenv import load_dotenv
    env_path = Path(__file__).parent.parent / '.env'
    load_dotenv(env_path)
except ImportError:
    pass

# Configure Gemini API
api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("GEMINI_API_KEY is required")

# Initialize W&B Weave
try:
    weave.init('gemini-navigation-ai')
    if not os.environ.get('SUBPROCESS_MODE'):
        print("[DEBUG] W&B Weave initialized successfully")
except Exception as e:
    if not os.environ.get('SUBPROCESS_MODE'):
        print(f"[DEBUG] W&B Weave initialization failed: {e}")

# Create the client and model
client = genai.Client(api_key=api_key)
model_name = "gemini-1.5-pro-latest"

# Navigation system prompt - ultra-compact response
NAVIGATION_PROMPT = """2D game: 400x400 pixels. Red square (player), blue rects (obstacles), yellow star (goal).

Plan 6 SLOW deliberate bezier steps (3 primary, 3 speculative). Each step 20-40px max, duration 0.5-0.8s. Stay in bounds 0-370.

ULTRA-COMPACT JSON:
{"m":[{"t":"p","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"step1"},{"t":"p","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"step2"},{"t":"p","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"step3"},{"t":"s","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"alt1"},{"t":"s","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"alt2"},{"t":"s","s":[x,y],"c1":[x,y],"c2":[x,y],"e":[x,y],"d":0.6,"v":200,"r":"alt3"}],"st":"slow moves"}

CRITICAL: All coords 0-370 max. Small moves (20-40px). SLOW duration 0.5-0.8s. NO spaces. <800 chars total."""

@weave.op
def analyze_navigation_image(image_bytes: bytes) -> dict:
    """Analyze navigation image and return structured response"""
    try:
        # Create the image part using google-genai
        from google.genai import types
        
        image_part = types.Part.from_bytes(
            data=image_bytes,
            mime_type="image/png"
        )
        
        # Generate content
        response = client.models.generate_content(
            model=model_name,
            contents=[
                types.Content(parts=[
                    types.Part(text=NAVIGATION_PROMPT),
                    image_part
                ])
            ],
            config=types.GenerateContentConfig(
                temperature=0.1,
                max_output_tokens=2000,  # Reduced to prevent truncation
                response_mime_type="application/json"
            )
        )
        
        # Parse response - handle markdown code blocks and double encoding
        try:
            response_text = response.text
            
            # Remove markdown code blocks if present
            if '```json' in response_text and '```' in response_text:
                # Extract JSON from markdown code blocks
                start = response_text.find('```json') + 7
                end = response_text.find('```', start)
                if start > 6 and end > start:
                    response_text = response_text[start:end].strip()
            
            # First parse attempt
            output = json.loads(response_text)
            
            # Check if the response is properly structured
            if isinstance(output, dict):
                # If motion_primitives is a string, try to parse it as JSON
                if 'motion_primitives' in output and isinstance(output['motion_primitives'], str):
                    try:
                        output['motion_primitives'] = json.loads(output['motion_primitives'])
                    except json.JSONDecodeError:
                        pass
                
                # If overall_strategy is missing but we have motion_primitives, add it
                if 'motion_primitives' in output and 'overall_strategy' not in output:
                    output['overall_strategy'] = "AI-generated motion plan"
                    
        except Exception as e:
            if not os.environ.get('SUBPROCESS_MODE'):
                print(f"[DEBUG] JSON parsing failed: {e}")
                print(f"[DEBUG] Raw response text: {response.text}")
            output = response.text
            
        # Debug: log the final parsed output structure
        if not os.environ.get('SUBPROCESS_MODE') and isinstance(output, dict):
            print(f"[DEBUG] Final parsed output keys: {list(output.keys())}")
            if 'motion_primitives' in output:
                mp = output['motion_primitives']
                print(f"[DEBUG] Motion primitives type: {type(mp)}, length: {len(mp) if isinstance(mp, (list, dict)) else 'N/A'}")
            
        return {
            'navigation_response': output
        }
        
    except Exception as e:
        if not os.environ.get('SUBPROCESS_MODE'):
            print(f"Navigation analysis error: {e}")
        return {
            'navigation_response': {
                'direction': None,
                'velocity': 0,
                'reasoning': f"Error: {str(e)}"
            }
        }

async def next_move(frame_bytes: bytes) -> Tuple[Optional[list], str]:
    """
    Get motion primitives for bezier curve navigation.
    
    Args:
        frame_bytes: Raw image data
        
    Returns:
        Tuple of (motion_primitives_list, overall_strategy)
    """
    try:
        # Use the weave-tracked function
        result = analyze_navigation_image(frame_bytes)
        nav_response = result['navigation_response']
        
        # Handle string response (fallback)
        if isinstance(nav_response, str):
            return None, nav_response
        
        # Extract motion primitives (handle both compact and full field names)
        motion_primitives = nav_response.get('m', nav_response.get('motion_primitives', []))
        overall_strategy = nav_response.get('st', nav_response.get('overall_strategy', 'No strategy provided'))
        
        # Validate motion primitives
        validated_primitives = []
        if not os.environ.get('SUBPROCESS_MODE'):
            print(f"[DEBUG] Validating {len(motion_primitives) if isinstance(motion_primitives, list) else 'non-list'} motion primitives")
        
        for i, primitive in enumerate(motion_primitives):
            if not os.environ.get('SUBPROCESS_MODE'):
                print(f"[DEBUG] Primitive {i}: type={type(primitive)}, keys={list(primitive.keys()) if isinstance(primitive, dict) else 'N/A'}")
                
            if isinstance(primitive, dict):
                # Convert compact field names to full names
                converted_primitive = {}
                
                # Handle type field
                converted_primitive['type'] = primitive.get('t', primitive.get('type', 'primary'))
                if converted_primitive['type'] == 'p':
                    converted_primitive['type'] = 'primary'
                elif converted_primitive['type'] == 's':
                    converted_primitive['type'] = 'speculative'
                
                # Handle coordinate fields
                converted_primitive['start'] = primitive.get('s', primitive.get('start', [0, 0]))
                converted_primitive['control1'] = primitive.get('c1', primitive.get('control1', [0, 0]))
                converted_primitive['control2'] = primitive.get('c2', primitive.get('control2', [0, 0]))
                converted_primitive['end'] = primitive.get('e', primitive.get('end', [0, 0]))
                
                # Handle numeric fields
                converted_primitive['duration'] = primitive.get('d', primitive.get('duration', 0.16))
                converted_primitive['velocity'] = primitive.get('v', primitive.get('velocity', 400))
                converted_primitive['reasoning'] = primitive.get('r', primitive.get('reasoning', ''))
                
                # Check required fields
                required_fields = ['start', 'control1', 'control2', 'end', 'duration']
                missing_fields = [field for field in required_fields if field not in converted_primitive]
                
                if not missing_fields:
                    # Clamp values to valid ranges - SLOW deliberate motion
                    converted_primitive['duration'] = max(0.5, min(1.0, float(converted_primitive['duration'])))  # Minimum 0.5s
                    converted_primitive['velocity'] = max(150, min(400, int(converted_primitive['velocity'])))  # Slower velocity
                    
                    # Ensure coordinates are within bounds and movements are small
                    for coord_key in ['start', 'control1', 'control2', 'end']:
                        if coord_key in converted_primitive and len(converted_primitive[coord_key]) >= 2:
                            x = max(0, min(370, float(converted_primitive[coord_key][0])))  # 370 = 400 - 30 (square size)
                            y = max(0, min(370, float(converted_primitive[coord_key][1])))
                            converted_primitive[coord_key] = [x, y]
                    
                    # Validate movement distance (should be small steps)
                    if 'start' in converted_primitive and 'end' in converted_primitive:
                        start = converted_primitive['start']
                        end = converted_primitive['end']
                        distance = ((end[0] - start[0])**2 + (end[1] - start[1])**2)**0.5
                        if distance > 60:  # Max 60 pixel movement per primitive
                            if not os.environ.get('SUBPROCESS_MODE'):
                                print(f"[DEBUG] Primitive {i} movement too large ({distance:.1f}px), clamping to smaller movement")
                            # Clamp to smaller movement
                            dx = end[0] - start[0]
                            dy = end[1] - start[1]
                            scale = 40 / distance  # Scale to max 40 pixels
                            converted_primitive['end'] = [start[0] + dx * scale, start[1] + dy * scale]
                            converted_primitive['control2'] = [start[0] + dx * scale * 0.7, start[1] + dy * scale * 0.7]
                    
                    validated_primitives.append(converted_primitive)
                    if not os.environ.get('SUBPROCESS_MODE'):
                        print(f"[DEBUG] Primitive {i} validated successfully")
                else:
                    if not os.environ.get('SUBPROCESS_MODE'):
                        print(f"[DEBUG] Primitive {i} missing required fields: {missing_fields}")
            else:
                if not os.environ.get('SUBPROCESS_MODE'):
                    print(f"[DEBUG] Primitive {i} is not a dict: {type(primitive)}")
        
        # Limit to 6 primitives max
        validated_primitives = validated_primitives[:6]
        
        if not os.environ.get('SUBPROCESS_MODE'):
            print(f"[DEBUG] Final validated primitives count: {len(validated_primitives)}")
        
        if not validated_primitives:
            return None, "No valid motion primitives generated"
        
        return validated_primitives, overall_strategy
        
    except Exception as e:
        if not os.environ.get('SUBPROCESS_MODE'):
            print(f"Navigation error: {e}")
        return None, f"Error: {str(e)}"

# Backward compatibility
async def analyze(image_bytes: bytes, prompt: str) -> str:
    """Legacy function for backward compatibility"""
    motion_primitives, strategy = await next_move(image_bytes)
    if motion_primitives and len(motion_primitives) > 0:
        first_primitive = motion_primitives[0]
        end_pos = first_primitive.get('end', [0, 0])
        velocity = first_primitive.get('velocity', 400)
        return f"Navigate: to {end_pos} at velocity {velocity} - {strategy}"
    return f"Strategy: {strategy}" 