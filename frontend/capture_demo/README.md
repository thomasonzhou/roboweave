# Live API Smoke Test

Verifies that the Live API client can round-trip an image through Google Live API.

## Setup & Usage

```bash
export GEMINI_API_KEY="•your-key•"
pip install -r frontend/capture_demo/requirements.txt
python frontend/capture_demo/verify_live_api.py
```

## Options

```bash
python frontend/capture_demo/verify_live_api.py --image sample.png \
  --prompt "Describe the image and suggest a next step."
```

## Files

- `live_api_client.py` - Live API client with VisionSession and analyze function
- `verify_live_api.py` - Smoke test script
- `sample.png` - 320×240 test image
- `requirements.txt` - Dependencies (google-generativeai, pillow)

## Testing

The script will:
1. Load the specified image
2. Send it to Google Gemini Vision API
3. Display the response with timing information
4. Exit with status 0 if successful, 1 if failed

Example output:
```
### Gemini replied in 1234.5 ms ###

This image shows geometric shapes including a blue rectangle, red circle, and green triangle on a white background with "RoboWeave Test" text.

Next step: Consider adding more complex visual elements or integrating this test image into a larger computer vision pipeline for object detection or pattern recognition tasks.

✅ Live API verification successful!
``` 