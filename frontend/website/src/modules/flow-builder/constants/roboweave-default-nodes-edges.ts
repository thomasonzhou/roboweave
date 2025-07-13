import { type Edge, type Node } from "@xyflow/react";

export const defaultRoboweaveNodes: Node[] = [
    {
        id: "prompt-input",
        type: "promptInput",
        position: { x: 50, y: 50 },
        data: {
            label: "Prompt Input",
            description: "Text and image inputs",
            status: "idle",
            icon: "📝"
        },
    },
    {
        id: "llm-processing",
        type: "llmProcessing", 
        position: { x: 250, y: 50 },
        data: {
            label: "LLM Processing",
            description: "Multimodal reasoning",
            status: "idle",
            icon: "🧠"
        },
    },
    {
        id: "motion-planner",
        type: "motionPlanner",
        position: { x: 450, y: 50 },
        data: {
            label: "Motion Planner",
            description: "Path planning & control",
            status: "idle", 
            icon: "🎯"
        },
    },
    {
        id: "mcp-protocol",
        type: "mcpProtocol",
        position: { x: 150, y: 200 },
        data: {
            label: "MCP Protocol",
            description: "Model Control Protocol",
            status: "idle",
            icon: "🔗"
        },
    },
    {
        id: "robot-execution",
        type: "robotExecution",
        position: { x: 350, y: 200 },
        data: {
            label: "Robot Execution", 
            description: "Hardware control",
            status: "idle",
            icon: "🤖"
        },
    },
    {
        id: "weave-monitoring",
        type: "weaveMonitoring",
        position: { x: 550, y: 200 },
        data: {
            label: "Weave Monitoring",
            description: "Trace & metrics",
            status: "idle",
            icon: "📊"
        },
    },
];

export const defaultRoboweaveEdges: Edge[] = [
    {
        id: "prompt-to-llm",
        source: "prompt-input",
        target: "llm-processing",
        type: "deletable",
        animated: true,
    },
    {
        id: "llm-to-motion",
        source: "llm-processing", 
        target: "motion-planner",
        type: "deletable",
        animated: true,
    },
    {
        id: "llm-to-mcp",
        source: "llm-processing",
        target: "mcp-protocol", 
        type: "deletable",
        animated: true,
    },
    {
        id: "motion-to-robot",
        source: "motion-planner",
        target: "robot-execution",
        type: "deletable",
        animated: true,
    },
    {
        id: "mcp-to-robot", 
        source: "mcp-protocol",
        target: "robot-execution",
        type: "deletable",
        animated: true,
    },
    {
        id: "robot-to-weave",
        source: "robot-execution",
        target: "weave-monitoring",
        type: "deletable",
        animated: true,
    },
]; 