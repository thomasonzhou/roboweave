#!/usr/bin/env python3
"""
Smoke test script for Google Live API integration.
Verifies that the Live API client can round-trip an image through Google Live API.
"""

from pathlib import Path, PurePath
import asyncio
import time
from live_api_client import VisionSession, analyze


async def main(img, prm):
    """Main function to test Live API integration."""
    t0 = time.perf_counter()
    answer = await analyze(Path(img).read_bytes(), prm)
    print(f"\n### Gemini replied in {1000*(time.perf_counter()-t0):.1f} ms ###\n")
    print(answer)
    
    # Verify we got a non-empty answer
    if not answer or not answer.strip():
        raise RuntimeError("Empty response from Gemini API")
    
    return answer


if __name__ == "__main__":
    import argparse
    import os
    import sys
    
    ap = argparse.ArgumentParser(description="Verify Live API integration")
    ap.add_argument("--image", default="sample.png", help="Image file to analyze")
    ap.add_argument("--prompt", default="What is depicted and next step?", help="Analysis prompt")
    args = ap.parse_args()
    
    # Check for API key
    if "GEMINI_API_KEY" not in os.environ:
        sys.exit("Set GEMINI_API_KEY first")
    
    # Check if image file exists
    if not Path(args.image).exists():
        sys.exit(f"Image file not found: {args.image}")
    
    try:
        asyncio.run(main(args.image, args.prompt))
        print("\n✅ Live API verification successful!")
    except Exception as e:
        print(f"\n❌ Live API verification failed: {e}")
        sys.exit(1) 