#!/bin/bash

# Setup script for the Robot Control MCP Server

echo "Setting up Robot Control MCP Server..."


# Install dependencies
echo "Installing dependencies..."
uv add "mcp[cli]" mujoco mujoco-mpc

echo "Setup complete!"
echo ""
echo "To test the server, run:"
echo "  python robot_mcp_server.py"
echo ""
echo "To configure with Claude Desktop, add this to your claude_desktop_config.json:"
echo '{'
echo '  "mcpServers": {'
echo '    "robot-control": {'
echo '      "command": "uv",'
echo '      "args": ['
echo '        "--directory",'
echo '        "'$(pwd)'",'
echo '        "run",'
echo '        "robot_mcp_server.py"'
echo '      ]'
echo '    }'
echo '  }'
echo '}'
