#!/usr/bin/env python3
"""
RoboWeave Live Demo Startup Script
Launches WebSocket bridge and website development server simultaneously
"""

import subprocess
import sys
import os
import time
import signal
import threading
from pathlib import Path

def check_dependencies():
    """Check if all required dependencies are installed"""
    try:
        import websockets
        import weave
        print("✓ Python dependencies checked")
    except ImportError as e:
        print(f"❌ Missing Python dependency: {e}")
        print("Run: pip install -r requirements.txt")
        return False
    
    # Check if Node.js/npm is available
    try:
        subprocess.run(['npm', '--version'], check=True, capture_output=True)
        print("✓ Node.js/npm checked")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("❌ Node.js/npm not found")
        print("Please install Node.js: https://nodejs.org/")
        return False
    
    return True

def start_websocket_bridge():
    """Start the WebSocket bridge server"""
    print("🚀 Starting WebSocket bridge...")
    bridge_path = Path(__file__).parent / 'websocket_bridge.py'
    return subprocess.Popen([sys.executable, str(bridge_path)])

def start_website_dev_server():
    """Start the website development server"""
    print("🌐 Starting website development server...")
    website_path = Path(__file__).parent / 'website'
    
    # Check if node_modules exists, if not install dependencies
    if not (website_path / 'node_modules').exists():
        print("📦 Installing website dependencies...")
        subprocess.run(['npm', 'install'], cwd=website_path, check=True)
    
    return subprocess.Popen(['npm', 'run', 'dev'], cwd=website_path)

def main():
    """Main startup function"""
    print("🤖 RoboWeave Live Demo Startup")
    print("=" * 50)
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Check for .env file
    env_path = Path(__file__).parent / '.env'
    if not env_path.exists():
        print("⚠️  Warning: .env file not found!")
        print("Create .env with: GEMINI_API_KEY=your_api_key_here")
        print("The system will work with mock data without it.")
        print()
    
    processes = []
    
    try:
        # Start WebSocket bridge
        bridge_process = start_websocket_bridge()
        processes.append(bridge_process)
        
        # Wait a moment for bridge to start
        time.sleep(2)
        
        # Start website dev server
        website_process = start_website_dev_server()
        processes.append(website_process)
        
        print()
        print("🎉 RoboWeave is now running!")
        print("=" * 50)
        print("📡 WebSocket Bridge: ws://localhost:8765")
        print("🌐 Website: http://localhost:5173/demo")
        print()
        print("💡 How to use:")
        print("  1. Open http://localhost:5173/demo in your browser")
        print("  2. Enter a text prompt or upload an image")
        print("  3. Click 'Execute Pipeline' to see live processing")
        print("  4. Use 'Spawn New Agent' to create multiple workflows")
        print()
        print("Press Ctrl+C to stop all services")
        
        # Wait for processes
        while True:
            time.sleep(1)
            
            # Check if any process died
            for i, process in enumerate(processes):
                if process.poll() is not None:
                    print(f"❌ Process {i} died with code {process.returncode}")
                    return
    
    except KeyboardInterrupt:
        print("\n🛑 Shutting down RoboWeave...")
    
    finally:
        # Clean shutdown
        for process in processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
        
        print("✅ All services stopped")

if __name__ == "__main__":
    main() 