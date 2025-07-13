# RoboWeave: Prompt-Based Multimodal Control of Embodied Agents

**Daniel Siegel, Thomason Zhou**  
_Weave Hackathon 2025_

## Abstract

RoboWeave is a multimodal robot control pipeline that maps high-level prompts—expressed via natural language, images, or video—into structured robotic commands. It integrates the Google Gemini large multimodal model that calls on model context protocol (MCP) servers to enable intuitive control of physical systems. The project emphasizes the use of prompt engineering, multimodal reasoning, and real-time integration with the Weave platform for frontend interaction and visualization.

## System Overview

The core objective of RoboWeave is to enable robotic actuation from semantically rich prompts. Input is collected via a Weave-integrated frontend that reads sensor input, routed to Gemini for interpretation, and ultimately converted into commands to send to a backend motion execution API. The architecture supports a mixture of text, image, audio, and video inputs and is designed to generalize across multiple robotic tasks.

## Architecture

**Pipeline Components:**

1. **Weave Frontend Integration**
   - Handles prompt input via typed text, image upload, or recorded video.
   - Streams live actuator states and sensor readings from a robot
   - Offers live feedback, status visualization, and demonstration playback.
   - Provides an interface for operators to review prompt-to-behavior mappings.

2. **Prompt Interpretation**
   - Prompts are passed to Gemini, which performs multimodal reasoning and outputs structured descriptions of the intended robot behavior.
   - Gemini's output includes explicit action representations (e.g., "walk forward 2 meters avoiding obstacles") as well as implicit spatial and temporal cues inferred from media.

3. **LLM Parsing and Planning Layer**
   - Extracts semantic intent and translates it into discrete motion primitives.
   - Maintains state awareness and accounts for feasibility based on robot capabilities.

4. **Backend Execution**
   - Commands are translated into MCP format (Model Context Protocol).
   - Sent via fault-tolerant channels to the robot controller.
   - Execution is monitored for safety, validity, and alignment with prompt expectations.

## Key Features

- Multimodal Prompting: Supports text, images, and videos as input modalities.
- LLM-Driven Reasoning: Leverages Gemini’s integrated text-vision model to interpret human intent in both explicit and latent forms.
- Realtime Feedback Loop: Weave integration enables visualization of both prompts and resulting robotic behaviors in a continuous interaction loop.
- MCP Control Interface: Encapsulates low-level robot actuation via MCP-compatible command streaming.

## Use Case Scenarios

Three representative success cases were implemented and validated in the hackathon setting:

1. **Forward Navigation Without Collision**
   - User prompt: "Walk forward without hitting anything"
   - System generates obstacle-aware straight-line motion using LLM and planner

2. **Orientation Change**
   - User prompt: "Turn left 45 degrees"
   - Pose change verified via simulation and robot telemetry

3. **Simple Task Execution**
   - User prompt: "Stand up and initiate handshake"
   - Combines pose planning with action sequence construction

## Project Structure

```
roboweave/
├── frontend/            # Weave-based interface and prompt collection
├── backend/
│   ├── llm/             # Gemini interaction and prompt-to-motion translation
│   └── mcp/             # MCP integration layer
├── scripts/             # QA, telemetry, test cases
├── docs/                # Architecture notes and presentation assets
└── README.md
```

## Dependencies

- Google Gemini API (private access)
- MuJoCo (info TBD)
- Weave developer tools and presentation layer
- MCP-compatible robotic controller (real or simulated)
- Python 3.10+

## Deliverables (Weave Hackathon 2025)

- End-to-end demonstrator for prompt-driven robot control
- Fully integrated frontend-backend pipeline with Weave tracing
- Evaluation of multimodal reasoning performance for real-time robotic control
- Narrated presentation video and source-level documentation

## Team

Daniel Siegel [siegel.bio](https://www.siegel.bio)
Thomason Zhou [thzhou.com](https://thzhou.com/)

Affiliated with the 2025 Weave Hackathon  
For inquiries: danieledisonsiegel@gmail.com
