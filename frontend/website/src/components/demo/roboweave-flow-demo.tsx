import { useCallback, useState } from 'react';
import {
    ReactFlow,
    useNodesState,
    useEdgesState,
    addEdge,
    Background,
    Controls,
    MiniMap,
} from '@xyflow/react';
import type { Connection, Edge, Node } from '@xyflow/react';

interface RoboWeaveNodeData extends Record<string, unknown> {
    label: string;
    description: string;
    category: string;
}

type RoboWeaveNode = Node<RoboWeaveNodeData>;

const initialNodes: RoboWeaveNode[] = [
    {
        id: '1',
        type: 'default',
        position: { x: 50, y: 100 },
        data: { 
            label: '📝 Text Prompt',
            description: 'Natural language commands like "Walk forward without hitting anything"',
            category: 'Input'
        },
        style: {
            background: 'linear-gradient(135deg, #3b82f6, #06b6d4)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '2',
        type: 'default',
        position: { x: 50, y: 200 },
        data: { 
            label: '🖼️ Image Input',
            description: 'Visual scene analysis and object detection for spatial understanding',
            category: 'Input'
        },
        style: {
            background: 'linear-gradient(135deg, #3b82f6, #06b6d4)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '3',
        type: 'default',
        position: { x: 50, y: 300 },
        data: { 
            label: '🎥 Video Sequence',
            description: 'Temporal motion analysis and demonstration learning from video',
            category: 'Input'
        },
        style: {
            background: 'linear-gradient(135deg, #3b82f6, #06b6d4)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '4',
        type: 'default',
        position: { x: 350, y: 200 },
        data: { 
            label: '🧠 Gemini 1.5 Pro',
            description: 'Multimodal reasoning with 1M+ token context window for complex scene understanding',
            category: 'Processing'
        },
        style: {
            background: 'linear-gradient(135deg, #8b5cf6, #ec4899)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '5',
        type: 'default',
        position: { x: 650, y: 150 },
        data: { 
            label: '⚡ Motion Planner',
            description: 'Converts semantic intent into discrete motion primitives with safety constraints',
            category: 'Planning'
        },
        style: {
            background: 'linear-gradient(135deg, #10b981, #059669)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '6',
        type: 'default',
        position: { x: 650, y: 250 },
        data: { 
            label: '🔄 MCP Protocol',
            description: 'Model Context Protocol for fault-tolerant robot communication',
            category: 'Communication'
        },
        style: {
            background: 'linear-gradient(135deg, #f59e0b, #d97706)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '7',
        type: 'default',
        position: { x: 950, y: 200 },
        data: { 
            label: '🤖 Robot Execution',
            description: 'Physical robot actuation with real-time safety monitoring and validation',
            category: 'Output'
        },
        style: {
            background: 'linear-gradient(135deg, #dc2626, #b91c1c)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
    {
        id: '8',
        type: 'default',
        position: { x: 950, y: 100 },
        data: { 
            label: '📊 Weave Monitoring',
            description: 'Real-time observability and performance tracking with W&B Weave',
            category: 'Monitoring'
        },
        style: {
            background: 'linear-gradient(135deg, #6366f1, #8b5cf6)',
            color: 'white',
            border: 'none',
            borderRadius: '12px',
            padding: '10px',
            fontSize: '14px',
            fontWeight: 600,
        },
    },
];

const initialEdges: Edge[] = [
    { id: 'e1-4', source: '1', target: '4', animated: true, style: { stroke: '#3b82f6', strokeWidth: 2 } },
    { id: 'e2-4', source: '2', target: '4', animated: true, style: { stroke: '#3b82f6', strokeWidth: 2 } },
    { id: 'e3-4', source: '3', target: '4', animated: true, style: { stroke: '#3b82f6', strokeWidth: 2 } },
    { id: 'e4-5', source: '4', target: '5', animated: true, style: { stroke: '#8b5cf6', strokeWidth: 2 } },
    { id: 'e4-6', source: '4', target: '6', animated: true, style: { stroke: '#8b5cf6', strokeWidth: 2 } },
    { id: 'e5-7', source: '5', target: '7', animated: true, style: { stroke: '#10b981', strokeWidth: 2 } },
    { id: 'e6-7', source: '6', target: '7', animated: true, style: { stroke: '#f59e0b', strokeWidth: 2 } },
    { id: 'e7-8', source: '7', target: '8', animated: true, style: { stroke: '#dc2626', strokeWidth: 2 } },
];

export function RoboWeaveFlowDemo() {
    const [nodes, setNodes, onNodesChange] = useNodesState(initialNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState(initialEdges);
    const [selectedNode, setSelectedNode] = useState<RoboWeaveNode | null>(null);

    const onConnect = useCallback(
        (params: Connection) => setEdges((eds) => addEdge(params, eds)),
        [setEdges]
    );

    const onNodeClick = useCallback((_event: React.MouseEvent, node: RoboWeaveNode) => {
        setSelectedNode(node);
    }, []);

    const onPaneClick = useCallback(() => {
        setSelectedNode(null);
    }, []);

    return (
        <div className="w-full h-full relative">
            <ReactFlow
                nodes={nodes}
                edges={edges}
                onNodesChange={onNodesChange}
                onEdgesChange={onEdgesChange}
                onConnect={onConnect}
                onNodeClick={onNodeClick}
                onPaneClick={onPaneClick}
                fitView
                attributionPosition="bottom-left"
            >
                <Controls />
                <MiniMap 
                    style={{
                        background: '#1f2937',
                        border: '1px solid #374151',
                    }}
                    nodeColor={(node) => {
                        const category = node.data?.category;
                        switch (category) {
                            case 'Input': return '#3b82f6';
                            case 'Processing': return '#8b5cf6';
                            case 'Planning': return '#10b981';
                            case 'Communication': return '#f59e0b';
                            case 'Output': return '#dc2626';
                            case 'Monitoring': return '#6366f1';
                            default: return '#6b7280';
                        }
                    }}
                />
                <Background color="#374151" />
            </ReactFlow>

            {/* Node Details Panel */}
            {selectedNode && (
                <div className="absolute top-4 right-4 w-80 bg-dark-800/95 backdrop-blur-sm border border-dark-600 rounded-xl p-6 shadow-2xl">
                    <div className="flex justify-between items-start mb-4">
                        <h3 className="text-lg font-semibold text-light-50">
                            {selectedNode.data.label}
                        </h3>
                        <button
                            onClick={() => setSelectedNode(null)}
                            className="text-light-400 hover:text-light-50 transition-colors"
                        >
                            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                            </svg>
                        </button>
                    </div>
                    
                    <div className="mb-4">
                        <span className={`inline-block px-3 py-1 rounded-full text-xs font-medium ${
                            selectedNode.data.category === 'Input' ? 'bg-blue-500/20 text-blue-300' :
                            selectedNode.data.category === 'Processing' ? 'bg-purple-500/20 text-purple-300' :
                            selectedNode.data.category === 'Planning' ? 'bg-green-500/20 text-green-300' :
                            selectedNode.data.category === 'Communication' ? 'bg-orange-500/20 text-orange-300' :
                            selectedNode.data.category === 'Output' ? 'bg-red-500/20 text-red-300' :
                            selectedNode.data.category === 'Monitoring' ? 'bg-indigo-500/20 text-indigo-300' :
                            'bg-gray-500/20 text-gray-300'
                        }`}>
                            {selectedNode.data.category}
                        </span>
                    </div>
                    
                    <p className="text-light-300 text-sm leading-relaxed">
                        {selectedNode.data.description}
                    </p>
                </div>
            )}

            {/* Legend */}
            <div className="absolute bottom-4 left-4 bg-dark-800/95 backdrop-blur-sm border border-dark-600 rounded-xl p-4 shadow-2xl">
                <h4 className="text-sm font-semibold text-light-50 mb-3">Pipeline Categories</h4>
                <div className="space-y-2 text-xs">
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-blue-500"></div>
                        <span className="text-light-300">Input</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-purple-500"></div>
                        <span className="text-light-300">Processing</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-green-500"></div>
                        <span className="text-light-300">Planning</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-orange-500"></div>
                        <span className="text-light-300">Communication</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-red-500"></div>
                        <span className="text-light-300">Output</span>
                    </div>
                    <div className="flex items-center gap-2">
                        <div className="w-3 h-3 rounded-full bg-indigo-500"></div>
                        <span className="text-light-300">Monitoring</span>
                    </div>
                </div>
            </div>
        </div>
    );
} 