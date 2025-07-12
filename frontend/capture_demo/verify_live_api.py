#!/usr/bin/env python3
"""
Smoke test script for navigation oracle Live API integration.
Tests the next_move function with the gemini-live-2.5-flash-preview model.
"""

from pathlib import Path
import asyncio
import time
from live_api_client import next_move


async def main(img):
    """Main function to test navigation oracle."""
    t0 = time.perf_counter()
    direction = await next_move(Path(img).read_bytes())
    print(f"\n### Navigation Oracle replied in {1000*(time.perf_counter()-t0):.1f} ms ###\n")
    print(f"Recommended direction: {direction}")
    
    # Verify we got a valid direction
    if direction not in {"Up", "Down", "Left", "Right"}:
        raise RuntimeError(f"Invalid direction: {direction}")
    
    return direction


if __name__ == "__main__":
    import argparse
    import os
    import sys
    
    ap = argparse.ArgumentParser(description="Test navigation oracle")
    ap.add_argument("--image", default="sample.png", help="Game screenshot to analyze")
    args = ap.parse_args()
    
    # Check for API key
    if "GEMINI_API_KEY" not in os.environ:
        sys.exit("Set GEMINI_API_KEY first")
    
    # Check if image file exists
    if not Path(args.image).exists():
        sys.exit(f"Image file not found: {args.image}")
    
    try:
        direction = asyncio.run(main(args.image))
        print(f"\n✅ Navigation oracle test successful! Direction: {direction}")
    except Exception as e:
        print(f"\n❌ Navigation oracle test failed: {e}")
        sys.exit(1) 