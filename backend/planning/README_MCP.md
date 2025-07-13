# Robot Control MCP Server

This is a Model Context Protocol (MCP) server that exposes robot control actions as tools for LLM clients like Claude Desktop. It allows AI assistants to control and monitor a simulated quadruped robot.

## Features

The server provides the following motion primitive tools:

### 🤖 Robot State
- **`get_robot_state`** - Get current robot position and orientation

### 🎯 Basic Movement Primitives
- **`move_forward`** - Move robot forward by specified distance (relative to current position)
- **`move_backward`** - Move robot backward by specified distance (relative to current position)
- **`move_left`** - Move robot left by specified distance (relative to current position)
- **`move_right`** - Move robot right by specified distance (relative to current position)

### 🔄 Rotation Primitives
- **`rotate_left`** - Rotate robot counter-clockwise by specified angle
- **`rotate_right`** - Rotate robot clockwise by specified angle

### 🏃 Complex Motions
- **`run_in_circle`** - Make robot run in a circle with configurable radius and duration
- **`stop_and_stay`** - Stop robot and maintain current position

### 🤸 Special Actions
- **`do_flip`** - Make the robot perform a flip maneuver

## Installation

### Prerequisites
- Python 3.10 or higher
- MuJoCo physics simulator
- MuJoCo MPC library

### Quick Setup
1. Run the setup script:
```bash
chmod +x setup_mcp.sh
./setup_mcp.sh
```

### Manual Setup
1. Install uv package manager:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

2. Create and activate virtual environment:
```bash
uv venv
source .venv/bin/activate
```

3. Install dependencies:
```bash
uv add "mcp[cli]" mujoco mujoco-mpc
```

## Usage

### Testing the Server
Test the server directly:
```bash
source .venv/bin/activate
python robot_mcp_server.py
```

### Claude Desktop Integration
1. Add the server to your Claude Desktop configuration file at:
   - **macOS/Linux**: `~/Library/Application Support/Claude/claude_desktop_config.json`
   - **Windows**: `%APPDATA%/Claude/claude_desktop_config.json`

2. Add this configuration:
```json
{
  "mcpServers": {
    "robot-control": {
      "command": "uv",
      "args": [
        "--directory",
        "/ABSOLUTE/PATH/TO/roboweave/backend/planning",
        "run",
        "robot_mcp_server.py"
      ]
    }
  }
}
```

3. Restart Claude Desktop

### Example Commands in Claude Desktop
Once configured, you can use natural language commands like:

- "Get the current robot position"
- "Move the robot forward 1 meter"
- "Move the robot backward half a meter" 
- "Move the robot left 0.5 meters"
- "Move the robot right 0.3 meters"
- "Rotate the robot left 90 degrees"
- "Rotate the robot right 45 degrees"
- "Make the robot run in a circle with 3 meter radius"
- "Stop the robot and make it stay still"
- "Make the robot do a flip"

## Tool Reference

### get_robot_state()
Returns current robot state including:
- Position (x, y, z coordinates)
- Orientation (quaternion)
- Current mode

### move_forward(distance=0.3)
Moves robot forward relative to current position.
- `distance`: Distance in meters (default: 0.3m)

### move_backward(distance=0.3)
Moves robot backward relative to current position.
- `distance`: Distance in meters (default: 0.3m)

### move_left(distance=0.3)
Moves robot left relative to current position.
- `distance`: Distance in meters (default: 0.3m)

### move_right(distance=0.3)
Moves robot right relative to current position.
- `distance`: Distance in meters (default: 0.3m)

### rotate_left(angle_degrees=45.0)
Rotates robot counter-clockwise around current position.
- `angle_degrees`: Rotation angle in degrees (default: 45°)

### rotate_right(angle_degrees=45.0)
Rotates robot clockwise around current position.
- `angle_degrees`: Rotation angle in degrees (default: 45°)

### run_in_circle(radius=2.0, duration=10.0)
Makes robot run in a circle around current position.
- `radius`: Circle radius in meters (default: 2.0m)
- `duration`: Motion duration in seconds (default: 10s)

### stop_and_stay()
Stops robot movement and maintains current position.

### do_flip()
Switches robot to flip mode and executes flip maneuver.

## Architecture

The server uses:
- **FastMCP**: Simplified MCP server framework
- **MuJoCo**: Physics simulation engine
- **MuJoCo MPC**: Model Predictive Control for robot planning
- **Async/await**: Non-blocking operations for smooth control

## Troubleshooting

### Server Not Detected
- Ensure Claude Desktop is restarted after configuration
- Check that the absolute path in the config is correct
- Verify all dependencies are installed

### Robot Control Issues
- Make sure the robot model file exists at `../robot/tasks/quadruped/task_flat.xml`
- Check that MuJoCo and MuJoCo MPC are properly installed
- Verify the robot simulation is running

### Import Errors
Run the setup script or manually install dependencies:
```bash
uv add "mcp[cli]" mujoco mujoco-mpc
```

## Development

To extend the server with new tools:

1. Add a new function decorated with `@mcp.tool()`
2. Include proper type hints and docstring
3. Handle errors gracefully
4. Return string responses

Example:
```python
@mcp.tool()
async def custom_robot_action(param: float) -> str:
    """Description of the custom action.
    
    Args:
        param: Parameter description
        
    Returns:
        Status message
    """
    try:
        # Your robot control logic here
        return "Action completed successfully"
    except Exception as e:
        return f"Error: {str(e)}"
```

## License

This project follows the same license as the parent roboweave project.
