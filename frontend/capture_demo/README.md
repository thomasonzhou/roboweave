# Navigation Oracle - Live API Integration

A slimmed-down Live API client that provides navigation guidance for a 2D toy world using Google's gemini-live-2.5-flash-preview model.

## Navigation Oracle

The navigation oracle analyzes game screenshots and provides one-word directional guidance to move a red agent square closer to a blue target square.

### Features

- **Live API Integration**: Uses `google.genai.Client().aio.live.connect()`
- **Navigation Oracle**: AI that returns single-word directions (Up, Down, Left, Right)
- **Real-time Guidance**: Analyzes current game state and suggests optimal moves
- **Minimal Dependencies**: Only requires `google-genai`

## Setup & Usage

```bash
export GEMINI_API_KEY="your-api-key-here"
pip install -r requirements.txt
python verify_live_api.py
```

## Files

- `live_api_client.py` - Navigation oracle Live API client
- `verify_live_api.py` - Testing script for navigation oracle
- `sample.png` - Test image with red and blue squares
- `requirements.txt` - Minimal dependencies (google-genai only)

## API Reference

### `async def next_move(frame_bytes: bytes) -> str`

Analyzes a game screenshot and returns navigation guidance.

**Parameters:**
- `frame_bytes`: Raw PNG/JPEG image data

**Returns:**
- One of: "Up", "Down", "Left", "Right"
- Defaults to "Up" if response is invalid

**Example:**
```python
from live_api_client import next_move

# Load game screenshot
with open('game_screenshot.png', 'rb') as f:
    frame_bytes = f.read()

# Get navigation guidance
direction = await next_move(frame_bytes)
print(f"Move: {direction}")  # e.g., "Right"
```

## System Instruction

The navigation oracle uses this specialized system prompt:

```
You are a navigation oracle for a 2-D toy world.
The image shows a red agent square and one blue target square.
Return ONE WORD only — Up, Down, Left, or Right — that moves the red square closer to the blue.
No extra text.
```

## Integration with PySide6

The navigation oracle integrates with `wasd_stream.py`:

```bash
pip install PySide6 google-genai
export GEMINI_API_KEY="YOUR-KEY"
python ../wasd_stream.py
```

**Controls:**
- **WASD/arrows**: Manual movement
- **SPACE**: Get AI navigation guidance
- **ENTER**: Auto-move based on AI suggestion

## Testing

The verification script tests the navigation oracle:

```bash
python verify_live_api.py --image sample.png
```

**Expected Output:**
```
### Navigation Oracle replied in 1234.5 ms ###

Recommended direction: Right

✅ Navigation oracle test successful! Direction: Right
```

## Error Handling

- Invalid API responses default to "Up"
- Network errors return "Up" with error logging
- Missing API key raises ValueError
- Invalid image files are handled gracefully

## Technical Details

- **Model**: `gemini-live-2.5-flash-preview`
- **API Method**: `google.genai.Client().aio.live.connect()`
- **Image Format**: Base64-encoded PNG sent via Live API
- **Response Processing**: Extracts first text chunk, validates direction
- **Fallback**: Returns "Up" for any invalid or missing response

This navigation oracle demonstrates focused AI guidance for game automation and interactive assistance. 