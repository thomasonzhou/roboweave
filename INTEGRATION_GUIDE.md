# RoboWeave Integration Guide

This guide explains how to use the integrated RoboWeave system with the MCP server and frontend.

## System Overview

The RoboWeave system consists of:

1. **Robot Control MCP Server** - Exposes robot control tools via Model Context Protocol
2. **HTTP Bridge** - Translates HTTP requests to MCP tool calls
3. **React Frontend** - Interactive flow builder with robot control panel
4. **MuJoCo Viewer** - Visual simulation environment (runs in separate thread)

## Prerequisites

### Backend Requirements
```bash
# Install Python dependencies
cd backend/planning
pip install -r requirements_mcp.txt
```

### Frontend Requirements
```bash
# Install Node.js dependencies
cd frontend/website
npm install
```

## Starting the System

### Option 1: Full Stack (Recommended)

1. **Start the backend services:**
   ```bash
   cd backend/planning
   ./start_roboweave.sh
   ```
   
   This will:
   - Start the HTTP Bridge on port 8080
   - Initialize the MCP server with MuJoCo viewer
   - Display the robot simulation window

2. **Start the frontend:**
   ```bash
   cd frontend/website
   npm run dev
   ```
   
   Access the frontend at: http://localhost:5173

### Option 2: Backend Only (Testing)

```bash
cd backend/planning
python3 test_robot_control.py
```

This runs a simple test to verify the robot control system works.

## Using the System

### 1. Frontend Flow Builder

- Navigate to http://localhost:5173/demo
- Use the flow builder to create robot control pipelines
- Drag nodes from the sidebar to build your flow
- Connect nodes to define execution order

### 2. Robot Control Panel

In the sidebar, click the robot icon (🤖) to access the Robot Control Panel:

**Movement Controls:**
- ⬆️ Forward/Backward movement
- ⬅️➡️ Left/Right strafing  
- 🔄 Rotation controls
- ⏹️ Stop and stay

**Advanced Actions:**
- 🔄 Run in circle (configurable radius and duration)
- 🤸 Perform flip maneuver

**Real-time Status:**
- Robot position (x, y, z coordinates)
- Current operating mode
- Connection status
- Last command result

### 3. MuJoCo Viewer

The MuJoCo viewer window shows:
- 3D robot simulation
- Real-time robot movement
- Physics simulation
- Interactive camera controls

## API Endpoints

The HTTP bridge exposes these endpoints:

### Health Check
```
GET http://localhost:8080/health
```

### Robot State
```
GET http://localhost:8080/robot/state
```

### Movement Commands
```
POST http://localhost:8080/robot/move/forward
POST http://localhost:8080/robot/move/backward
POST http://localhost:8080/robot/move/left
POST http://localhost:8080/robot/move/right
```

### Rotation Commands
```
POST http://localhost:8080/robot/rotate/left
POST http://localhost:8080/robot/rotate/right
```

### Special Actions
```
POST http://localhost:8080/robot/circle
POST http://localhost:8080/robot/stop
POST http://localhost:8080/robot/flip
```

## Architecture

```
Frontend (React)
    ↓ HTTP Requests
HTTP Bridge (FastAPI)
    ↓ Function Calls
MCP Server (FastMCP)
    ↓ Agent Commands
MuJoCo Agent (Background Thread)
    ↓ Simulation
MuJoCo Viewer (Visual Window)
```

## Key Features

### Thread Safety
- MuJoCo agent runs in a dedicated thread
- HTTP bridge handles concurrent requests
- Frontend state management with React hooks

### Real-time Updates
- Robot state refreshes every 2 seconds
- Immediate feedback for all commands
- Error handling and status reporting

### Extensibility
- Add new MCP tools in `robot_mcp_server.py`
- Extend HTTP endpoints in `mcp_http_bridge.py`
- Create new flow nodes in the frontend

## Troubleshooting

### MuJoCo Viewer Not Appearing
```bash
export DISPLAY=:0  # Linux/WSL
```

### Connection Errors
- Ensure port 8080 is available
- Check firewall settings
- Verify Python dependencies are installed

### Frontend Build Issues
```bash
cd frontend/website
npm run type-check
npm run lint:fix
```

### Robot Not Responding
- Check MuJoCo viewer is active
- Verify robot state via API: `curl http://localhost:8080/robot/state`
- Restart the backend services

## Development

### Adding New Robot Commands

1. **Add MCP Tool:**
   ```python
   # In robot_mcp_server.py
   @mcp.tool()
   async def new_command() -> str:
       # Implementation
       pass
   ```

2. **Add HTTP Endpoint:**
   ```python
   # In mcp_http_bridge.py
   @app.post("/robot/new_command")
   async def new_command():
       return await call_tool("new_command")
   ```

3. **Add Frontend Method:**
   ```typescript
   // In mcp-client.ts
   async newCommand(): Promise<MCPToolResponse> {
       return this.callRobotAPI('/robot/new_command', {});
   }
   ```

4. **Update UI:**
   ```tsx
   // In robot-control-panel.tsx
   <button onClick={() => robotActions.newCommand()}>
       New Command
   </button>
   ```

## Support

For issues and questions:
- Check the console output for error messages
- Verify all dependencies are installed
- Ensure proper file permissions for scripts
- Test with the simple backend test script first

## Architecture Decisions

### Why Threading for MuJoCo?
- MuJoCo viewer requires main thread access
- Async/await doesn't work well with MuJoCo's blocking operations
- Threading allows concurrent HTTP requests while maintaining viewer

### Why HTTP Bridge?
- Separates concerns between MCP protocol and web frontend
- Enables future integration with other clients
- Provides standard REST API for testing and development

### Why React Flow?
- Visual flow building interface
- Node-based architecture matches robot control pipelines
- Extensible component system for custom nodes
