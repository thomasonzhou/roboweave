#!/bin/bash

# Startup script for RoboWeave MCP Integration
# This script starts both the MCP server and the HTTP bridge

echo "🚀 Starting RoboWeave MCP Integration..."

# Check if we're in the right directory
if [ ! -f "robot_mcp_server.py" ]; then
    echo "❌ Error: robot_mcp_server.py not found. Please run this script from the backend/planning directory."
    exit 1
fi

# Activate virtual environment if it exists
if [ -d ".venv" ]; then
    echo "📦 Activating virtual environment..."
    source .venv/bin/activate
fi

# Check dependencies
echo "🔍 Checking dependencies..."
python -c "import mcp, mujoco, fastapi, uvicorn" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📋 Installing dependencies..."
    if command -v uv &> /dev/null; then
        uv add "mcp[cli]" mujoco mujoco-mpc fastapi "uvicorn[standard]" pydantic
    else
        pip install -r requirements_mcp.txt
    fi
fi

# Start the HTTP bridge server
echo "🌐 Starting HTTP Bridge Server..."
echo "📍 Server will be available at: http://localhost:8080"
echo "📚 API docs will be available at: http://localhost:8080/docs"
echo ""
echo "🎮 Frontend can now connect to the robot control system!"
echo "💡 Use Ctrl+C to stop the server"
echo ""

python mcp_http_bridge.py
