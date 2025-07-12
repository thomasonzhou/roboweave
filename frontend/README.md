# GameWidget - PySide6 Implementation

A self-contained game widget that renders a fixed blue square and a movable red square with keyboard controls. Features a main window with side-by-side game view and live preview.

## Features

- **Fixed blue square** at the center of a 400×400 pixel window
- **Movable red square** that responds to keyboard input
- **Boundary clamping** - the red square stays within the window boundaries
- **Smooth movement** - 10 pixels per key press
- **Side-by-side layout** - Game widget on the left, live preview on the right
- **Real-time preview** - Live capture of the game widget updated every 100ms

## Controls

- **W** or **Up Arrow**: Move red square up
- **S** or **Down Arrow**: Move red square down
- **A** or **Left Arrow**: Move red square left
- **D** or **Right Arrow**: Move red square right

## Installation

1. Install PySide6:
   ```bash
   pip3 install PySide6
   ```

2. Or use the requirements file:
   ```bash
   pip3 install -r requirements.txt
   ```

## Usage

Run the game widget:
```bash
python3 game_widget.py
```

## Implementation Details

- **GameWidget class**: Inherits from `QWidget`
  - **paintEvent**: Renders white background, blue square (center), and red square (current position)
  - **keyPressEvent**: Handles WASD and arrow key input with boundary clamping
  - **Window size**: Fixed at 400×400 pixels
  - **Square size**: 30×30 pixels
  - **Movement step**: 10 pixels per key press

- **Main class**: Inherits from `QWidget`
  - **QHBoxLayout**: Arranges GameWidget and QLabel side by side
  - **QTimer**: Updates live preview every 100ms using `grab()`
  - **Window size**: Fixed at 800×400 pixels (two 400×400 panes)
  - **keyPressEvent**: Forwards keyboard events to the GameWidget

## Requirements

- Python 3.9+
- PySide6 6.5.0+

## Architecture

The application uses Qt's event system:
- **GameWidget**:
  - `paintEvent()` for rendering
  - `keyPressEvent()` for input handling
  - `setFocusPolicy(Qt.StrongFocus)` to receive keyboard events
  - `update()` to trigger repaints when position changes
- **Main widget**:
  - `QHBoxLayout` for side-by-side arrangement
  - `QTimer` for periodic screen capture
  - `grab()` method to capture GameWidget contents
  - `QLabel` with `setScaledContents(True)` for preview display 