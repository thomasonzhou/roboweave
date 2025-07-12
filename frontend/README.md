# Frontend Components

This directory contains both PySide6 applications and the Live API Web Console for multimodal interactions.

## PySide6 Game Widget with Gemini Analysis

A self-contained game widget that renders a fixed blue square and a movable red square with keyboard controls. Features real-time Gemini Vision analysis.

### Features

- **Fixed blue square** at the center of a 400×400 pixel window
- **Movable red square** that responds to keyboard input
- **Boundary clamping** - the red square stays within the window boundaries
- **Smooth movement** - 10 pixels per key press
- **Side-by-side layout** - Game widget on the left, live preview on the right
- **Real-time preview** - Live capture of the game widget updated every 100ms
- **Gemini Vision analysis** - Press SPACE to analyze current view

### Controls

- **WASD/Arrow keys**: Move red square
- **SPACE**: Analyze current view with Gemini

### Usage

```bash
pip install PySide6 google-generativeai
export GEMINI_API_KEY="YOUR-KEY"
python wasd_stream.py
```

## Live API Web Console

A React-based web console for using the [Live API](https://ai.google.dev/api/multimodal-live) over a websocket. It provides modules for streaming audio playback, recording user media such as from a microphone, webcam or screen capture as well as a unified log view.

[![Live API Demo](readme/thumbnail.png)](https://www.youtube.com/watch?v=J_q7JY1XxFE)

### Getting Started

To get started, [create a free Gemini API key](https://aistudio.google.com/apikey) and add it to the `.env` file. Then:

```bash
npm install && npm start
```

### System Instruction

For RoboWeave integration, use this system instruction:

```
You are "RoboWeave Vision Coach".
When you receive an image, perform two tasks in plain text:
① Succinctly describe what is visible.
② Recommend one concrete next step an engineer should take to advance the project.
If a data-or metric-driven plot would help, call the tool "render_altair" and pass a vega-lite spec as a JSON string in the arg `json_graph`.
Otherwise answer normally.
```

Select model: **gemini-live-2.5-flash-preview**

### Example Applications

Several example applications are available on other branches:

- [demos/GenExplainer](https://github.com/google-gemini/multimodal-live-api-web-console/tree/demos/genexplainer)
- [demos/GenWeather](https://github.com/google-gemini/multimodal-live-api-web-console/tree/demos/genweather)
- [demos/GenList](https://github.com/google-gemini/multimodal-live-api-web-console/tree/demos/genlist)

### Available Scripts

In the project directory, you can run:

#### `npm start`

Runs the app in the development mode.
Open [http://localhost:3000](http://localhost:3000) to view it in the browser.

#### `npm run build`

Builds the app for production to the `build` folder.

## Architecture Integration

### PySide6 Components
- `wasd_stream.py` - Game widget with Gemini analysis
- `game_widget.py` - Basic game widget
- `capture_demo/` - Live API client and testing tools

### Live API Web Console
- `src/` - React application source
- `src/lib/genai-live-client.ts` - Core Live API client
- `src/hooks/use-live-api.ts` - React hooks for Live API
- `src/components/` - UI components

### Integration Points

The PySide6 application (`wasd_stream.py`) can integrate with the Live API client through:

1. **Direct API calls** via `capture_demo/live_api_client.py`
2. **Screen sharing** with the web console for real-time analysis
3. **Shared system instructions** for consistent behavior

## Requirements

- Python 3.9+ (for PySide6 components)
- Node.js 16+ (for web console)
- Gemini API key
- PySide6, google-generativeai, pillow (Python deps)
- React, TypeScript (web console deps)

_This includes experiments showcasing the Live API, not official Google products. We'll do our best to support and maintain these experiments but your mileage may vary._
