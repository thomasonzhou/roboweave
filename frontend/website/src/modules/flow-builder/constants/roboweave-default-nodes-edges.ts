import { nanoid } from "nanoid";

import { BuilderNode } from "~/modules/nodes/types";
import { createNodeWithDefaultData } from "~/modules/nodes/utils";

// Create RoboWeave pipeline nodes
const textInputNode = createNodeWithDefaultData(BuilderNode.PROMPT_INPUT, { 
    position: { x: 50, y: 200 },
    data: { inputType: "text", example: "Walk forward without hitting anything" }
});

const imageInputNode = createNodeWithDefaultData(BuilderNode.PROMPT_INPUT, { 
    position: { x: 50, y: 100 },
    data: { inputType: "image", example: "Visual scene analysis" }
});

const videoInputNode = createNodeWithDefaultData(BuilderNode.PROMPT_INPUT, { 
    position: { x: 50, y: 300 },
    data: { inputType: "video", example: "Temporal motion analysis" }
});

const llmProcessingNode = createNodeWithDefaultData(BuilderNode.LLM_PROCESSING, { 
    position: { x: 400, y: 200 }
});

const motionPlannerNode = createNodeWithDefaultData(BuilderNode.MOTION_PLANNER, { 
    position: { x: 750, y: 150 }
});

const mcpProtocolNode = createNodeWithDefaultData(BuilderNode.MCP_PROTOCOL, { 
    position: { x: 750, y: 250 }
});

const robotExecutionNode = createNodeWithDefaultData(BuilderNode.ROBOT_EXECUTION, { 
    position: { x: 1100, y: 200 }
});

const weaveMonitoringNode = createNodeWithDefaultData(BuilderNode.WEAVE_MONITORING, { 
    position: { x: 1100, y: 100 }
});

const roboweaveNodes = [
    textInputNode,
    imageInputNode,
    videoInputNode,
    llmProcessingNode,
    motionPlannerNode,
    mcpProtocolNode,
    robotExecutionNode,
    weaveMonitoringNode
];

const roboweaveEdges = [
    { id: nanoid(), source: textInputNode.id, target: llmProcessingNode.id, type: "deletable" },
    { id: nanoid(), source: imageInputNode.id, target: llmProcessingNode.id, type: "deletable" },
    { id: nanoid(), source: videoInputNode.id, target: llmProcessingNode.id, type: "deletable" },
    { id: nanoid(), source: llmProcessingNode.id, target: motionPlannerNode.id, type: "deletable" },
    { id: nanoid(), source: llmProcessingNode.id, target: mcpProtocolNode.id, type: "deletable" },
    { id: nanoid(), source: motionPlannerNode.id, target: robotExecutionNode.id, type: "deletable" },
    { id: nanoid(), source: mcpProtocolNode.id, target: robotExecutionNode.id, type: "deletable" },
    { id: nanoid(), source: robotExecutionNode.id, target: weaveMonitoringNode.id, type: "deletable" },
];

export {
    roboweaveNodes as defaultRoboweaveNodes,
    roboweaveEdges as defaultRoboweaveEdges,
}; 