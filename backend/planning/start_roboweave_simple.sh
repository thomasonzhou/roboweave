#!/bin/bash

# RoboWeave Startup Script
# This script starts the robot control HTTP server with MuJoCo viewer in main thread

echo "🚀 Starting RoboWeave Robot Control System..."

# Set display for MuJoCo viewer (for Linux/WSL)
export DISPLAY=${DISPLAY:-:0}

# Ensure we're in the correct directory
cd "$(dirname "$0")"

echo "📂 Working directory: $(pwd)"

# Check if required Python packages are installed
echo "🔍 Checking Python dependencies..."
python3 -c "import mujoco, mujoco_mcp, fastapi" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing required packages. Installing..."
    pip install -r requirements_mcp.txt
fi

echo "🤖 Starting Robot Control HTTP Server..."
echo "   - MuJoCo viewer will open in the main window"
echo "   - Agent will be initialized in main thread for viewer visibility"
echo "   - HTTP API will be available on port 8080"

echo ""
echo "🎉 Starting RoboWeave Robot Control System..."
echo ""
echo "🔗 Services will be available at:"
echo "   - HTTP API: http://localhost:8080"
echo "   - API Docs: http://localhost:8080/docs"
echo "   - Health Check: http://localhost:8080/health"
echo ""
echo "🎮 MuJoCo viewer should be visible for robot control"
echo "🌐 Frontend can now connect to the robot control system"
echo ""
echo "Press Ctrl+C to stop the server..."
echo ""

# Run the standalone HTTP server (agent in main thread, HTTP in background thread)
python3 robot_http_server.py
