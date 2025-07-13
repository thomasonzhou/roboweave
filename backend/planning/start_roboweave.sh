#!/bin/bash

# RoboWeave Startup Script
# This script starts both the MCP server (with MuJoCo viewer) and the HTTP bridge

echo "🚀 Starting RoboWeave Robot Control System..."

# Set display for MuJoCo viewer (for Linux/WSL)
export DISPLAY=${DISPLAY:-:0}

# Ensure we're in the correct directory
cd "$(dirname "$0")"

echo "📂 Working directory: $(pwd)"

# Check if required Python packages are installed
echo "🔍 Checking Python dependencies..."
python3 -c "import mujoco, mujoco_mpc, fastapi" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing required packages. Installing..."
    pip install -r requirements_mcp.txt
fi

# Function to cleanup background processes
cleanup() {
    echo "🛑 Shutting down services..."
    if [ ! -z "$HTTP_BRIDGE_PID" ]; then
        kill $HTTP_BRIDGE_PID 2>/dev/null
    fi
    if [ ! -z "$MCP_SERVER_PID" ]; then
        kill $MCP_SERVER_PID 2>/dev/null
    fi
    exit 0
}

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

echo "🤖 Starting Robot Control MCP Server..."
echo "   - MuJoCo viewer will open in a separate window"
echo "   - Agent will be initialized in background thread"

# Start the HTTP Bridge
echo "🌐 Starting HTTP Bridge Server on port 8080..."
python3 mcp_http_bridge.py &
HTTP_BRIDGE_PID=$!

# Wait for services to start
sleep 3

# Check if services are running
if ps -p $HTTP_BRIDGE_PID > /dev/null; then
    echo "✅ HTTP Bridge started successfully (PID: $HTTP_BRIDGE_PID)"
else
    echo "❌ Failed to start HTTP Bridge"
    exit 1
fi

echo ""
echo "🎉 RoboWeave Robot Control System is ready!"
echo ""
echo "🔗 Services available at:"
echo "   - HTTP API: http://localhost:8080"
echo "   - API Docs: http://localhost:8080/docs"
echo "   - Health Check: http://localhost:8080/health"
echo ""
echo "🎮 MuJoCo viewer should be visible for robot control"
echo "🌐 Frontend can now connect to the robot control system"
echo ""
echo "Press Ctrl+C to stop all services..."

# Wait for user to stop
wait $HTTP_BRIDGE_PID
