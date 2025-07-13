import { Background, type Connection, type Edge, type EdgeTypes, type Node, type NodeChange, ReactFlow, addEdge, useEdgesState, useNodesState, useReactFlow } from "@xyflow/react";
import { nanoid } from "nanoid";
import { useCallback, useEffect } from "react";

import CustomControls from "~/modules/flow-builder/components/controls/custom-controls";
import CustomDeletableEdge from "~/modules/flow-builder/components/edges/custom-deletable-edge";
import { defaultRoboweaveNodes, defaultRoboweaveEdges } from "~/modules/flow-builder/constants/roboweave-default-nodes-edges";
import { useDeleteKeyCode } from "~/modules/flow-builder/hooks/use-delete-key-code";
import { useDragDropFlowBuilder } from "~/modules/flow-builder/hooks/use-drag-drop-flow-builder";
import { useIsValidConnection } from "~/modules/flow-builder/hooks/use-is-valid-connection";
import { useNodeAutoAdjust } from "~/modules/flow-builder/hooks/use-node-auto-adjust";
import { useOnNodesDelete } from "~/modules/flow-builder/hooks/use-on-nodes-delete";
import { NODE_TYPES } from "~/modules/nodes";
import { useApplicationState } from "~/stores/application-state";

const edgeTypes: EdgeTypes = {
    deletable: CustomDeletableEdge,
};

interface RoboWeaveFlowBuilderProps {
    agents?: any[];
    steps?: any[];
    weaveData?: any;
    isProcessing?: boolean;
}

export function RoboWeaveFlowBuilder({ agents = [], steps = [], weaveData, isProcessing = false }: RoboWeaveFlowBuilderProps) {
    const [isMobileView] = useApplicationState(s => [s.view.mobile]);

    const [nodes, setNodes, onNodesChange] = useNodesState<Node>(defaultRoboweaveNodes);
    const [edges, setEdges, onEdgesChange] = useEdgesState<Edge>(defaultRoboweaveEdges);

    const { getNodes } = useReactFlow();

    const deleteKeyCode = useDeleteKeyCode();
    const onNodesDelete = useOnNodesDelete(nodes);

    const autoAdjustNode = useNodeAutoAdjust();

    const [onDragOver, onDrop] = useDragDropFlowBuilder();
    const isValidConnection = useIsValidConnection(nodes, edges);

    // Update nodes with real-time data from weave and processing steps
    useEffect(() => {
        if (steps.length > 0 || weaveData || isProcessing) {
            setNodes(currentNodes => 
                currentNodes.map(node => {
                    const stepData = steps.find(step => step.id === node.id);
                    let status = 'idle';
                    
                    if (stepData) {
                        status = stepData.status;
                    } else if (isProcessing) {
                        // Auto-advance status based on processing state
                        if (node.id === 'prompt-input') status = 'processing';
                        else if (node.id === 'llm-processing' && steps.some(s => s.id === 'prompt-input' && s.status === 'completed')) status = 'processing';
                        else if (node.id === 'motion-planner' && steps.some(s => s.id === 'llm-processing' && s.status === 'completed')) status = 'processing';
                    }

                    return {
                        ...node,
                        data: {
                            ...node.data,
                            status,
                            weaveData: weaveData && weaveData[node.id] ? weaveData[node.id] : null,
                            agents: agents.filter(agent => agent.nodeId === node.id),
                            timestamp: stepData?.timestamp || new Date().toISOString(),
                        }
                    };
                })
            );
        }
    }, [steps, weaveData, isProcessing, agents, setNodes]);

    // Update edge animations based on processing state
    useEffect(() => {
        if (isProcessing) {
            setEdges(currentEdges => 
                currentEdges.map(edge => ({
                    ...edge,
                    animated: true,
                    style: { 
                        stroke: isProcessing ? '#f97316' : '#64748b',
                        strokeWidth: isProcessing ? 2 : 1
                    }
                }))
            );
        }
    }, [isProcessing, setEdges]);

    const onConnect = useCallback(
        (connection: Connection) => {
            const edge = { ...connection, id: nanoid(), type: "deletable" } as Edge;
            setEdges(edges => addEdge(edge, edges));
        },
        [setEdges],
    );

    const handleAutoAdjustNodeAfterNodeMeasured = useCallback(
        (id: string) => {
            setTimeout(() => {
                const node = getNodes().find(n => n.id === id);
                if (!node) { return; }

                if (node.measured === undefined) {
                    handleAutoAdjustNodeAfterNodeMeasured(id);
                    return;
                }

                autoAdjustNode(node);
            });
        },
        [autoAdjustNode, getNodes],
    );

    const handleNodesChange = useCallback(
        (changes: NodeChange[]) => {
            onNodesChange(changes);

            changes.forEach((change) => {
                if (change.type === "dimensions") {
                    const node = getNodes().find(n => n.id === change.id);
                    if (node) {
                        autoAdjustNode(node);
                    }
                }

                if (change.type === "add") {
                    handleAutoAdjustNodeAfterNodeMeasured(change.item.id);
                }
            });
        },
        [autoAdjustNode, getNodes, handleAutoAdjustNodeAfterNodeMeasured, onNodesChange],
    );

    return (
        <div className="w-full h-full overflow-hidden">
            <ReactFlow
                proOptions={{ hideAttribution: true }}
                nodeTypes={NODE_TYPES}
                onInit={({ fitView }) => fitView({ padding: 0.1, minZoom: 0.8, maxZoom: 1.2 })}
                nodes={nodes}
                onNodesChange={handleNodesChange}
                edgeTypes={edgeTypes}
                edges={edges}
                onEdgesChange={onEdgesChange}
                onConnect={onConnect}
                onDrop={onDrop}
                onDragOver={onDragOver}
                onNodeDragStop={(_, node) => { autoAdjustNode(node); }}
                onNodesDelete={onNodesDelete}
                isValidConnection={isValidConnection}
                multiSelectionKeyCode={null}
                deleteKeyCode={deleteKeyCode}
                snapGrid={[16, 16]}
                snapToGrid
                fitView
                fitViewOptions={{ padding: 0.1, minZoom: 0.8, maxZoom: 1.2 }}
                minZoom={0.8}
                maxZoom={1.2}
                zoomOnDoubleClick={false}
                panOnDrag={[1, 2]}
                selectionOnDrag={true}
                className="bg-transparent"
            >
                <Background 
                    color="rgba(148, 163, 184, 0.1)" 
                    gap={24} 
                    size={1}
                />
                <CustomControls />
            </ReactFlow>
            
            {/* Weave Data Overlay */}
            {weaveData && (
                <div className="absolute top-4 left-4 bg-white/95 backdrop-blur-sm rounded-lg p-4 shadow-lg border border-stone-200 max-w-lg max-h-[85vh] overflow-y-auto">
                    <div className="space-y-3">
                        <div className="flex items-center justify-between">
                            <h4 className="font-semibold text-sm text-stone-800">🧶 AI Intelligence Analysis</h4>
                            <span className="text-xs text-green-600 bg-green-100 px-2 py-1 rounded">Live</span>
                        </div>
                        
                        {/* Trace Header */}
                        <div className="text-xs text-stone-600 bg-stone-50 p-2 rounded border">
                            <div className="font-mono break-all">{weaveData.trace_id}</div>
                            <div className="mt-1">{weaveData.robot_type} • {weaveData.simulator}</div>
                            <div className="mt-1 font-medium text-stone-700">"{weaveData.user_command}"</div>
                        </div>
                        
                        {/* Cognitive Analysis */}
                        {weaveData.cognitive_analysis && (
                            <div className="space-y-2">
                                <div className="text-xs font-medium text-purple-800 bg-purple-50 px-2 py-1 rounded">🧠 Cognitive Analysis</div>
                                
                                {weaveData.cognitive_analysis.command_interpretation && (
                                    <div className="text-xs text-stone-600 bg-purple-50 p-2 rounded">
                                        <div className="font-medium text-purple-700 mb-1">Command Understanding</div>
                                        <div><span className="font-medium">Context:</span> {weaveData.cognitive_analysis.command_interpretation.emotional_context}</div>
                                        <div><span className="font-medium">Intent:</span> {weaveData.cognitive_analysis.command_interpretation.inferred_intent}</div>
                                        <div><span className="font-medium">Urgency:</span> {weaveData.cognitive_analysis.command_interpretation.urgency_level}</div>
                                    </div>
                                )}
                                
                                {weaveData.cognitive_analysis.decision_tree && (
                                    <div className="text-xs text-stone-600 bg-blue-50 p-2 rounded">
                                        <div className="font-medium text-blue-700 mb-1">Decision Process</div>
                                        <div><span className="font-medium">Strategy:</span> {weaveData.cognitive_analysis.decision_tree.primary_strategy}</div>
                                        <div className="mt-1">
                                            <span className="font-medium">Alternatives:</span>
                                            <div className="ml-2">
                                                {weaveData.cognitive_analysis.decision_tree.alternatives_considered?.slice(0, 2).map((alt: string, idx: number) => (
                                                    <div key={idx} className="text-xs">• {alt}</div>
                                                ))}
                                            </div>
                                        </div>
                                    </div>
                                )}
                                
                                {weaveData.cognitive_analysis.reasoning_depth && (
                                    <div className="text-xs text-stone-600 bg-indigo-50 p-2 rounded">
                                        <div className="font-medium text-indigo-700 mb-1">Reasoning Depth</div>
                                        <div><span className="font-medium">Surface:</span> {weaveData.cognitive_analysis.reasoning_depth.surface_analysis}</div>
                                        <div className="mt-1"><span className="font-medium">Deep Analysis:</span> {weaveData.cognitive_analysis.reasoning_depth.deep_analysis}</div>
                                    </div>
                                )}
                            </div>
                        )}
                        
                        {/* Environmental Intelligence */}
                        {weaveData.environmental_intelligence && (
                            <div className="space-y-2">
                                <div className="text-xs font-medium text-green-800 bg-green-50 px-2 py-1 rounded">🌍 Environmental Intelligence</div>
                                
                                {weaveData.environmental_intelligence.scene_understanding && (
                                    <div className="text-xs text-stone-600 bg-green-50 p-2 rounded">
                                        <div className="font-medium text-green-700 mb-1">Scene Analysis</div>
                                        <div>{weaveData.environmental_intelligence.scene_understanding.spatial_layout}</div>
                                        {weaveData.environmental_intelligence.scene_understanding.obstacle_analysis && (
                                            <div className="mt-1">
                                                <span className="font-medium">Obstacles:</span>
                                                <div className="ml-2">
                                                    {weaveData.environmental_intelligence.scene_understanding.obstacle_analysis.slice(0, 2).map((obs: string, idx: number) => (
                                                        <div key={idx} className="text-xs">• {obs}</div>
                                                    ))}
                                                </div>
                                            </div>
                                        )}
                                    </div>
                                )}
                                
                                {weaveData.environmental_intelligence.situational_awareness && (
                                    <div className="text-xs text-stone-600 bg-yellow-50 p-2 rounded">
                                        <div className="font-medium text-yellow-700 mb-1">Situational Awareness</div>
                                        <div><span className="font-medium">Context:</span> {weaveData.environmental_intelligence.situational_awareness.mission_context}</div>
                                        <div><span className="font-medium">Risk Level:</span> {weaveData.environmental_intelligence.situational_awareness.risk_environment}</div>
                                    </div>
                                )}
                            </div>
                        )}
                        
                        {/* Predictive Intelligence */}
                        {weaveData.predictive_intelligence && (
                            <div className="space-y-2">
                                <div className="text-xs font-medium text-orange-800 bg-orange-50 px-2 py-1 rounded">🔮 Predictive Intelligence</div>
                                
                                {weaveData.predictive_intelligence.outcome_forecasting && (
                                    <div className="text-xs text-stone-600 bg-orange-50 p-2 rounded">
                                        <div className="font-medium text-orange-700 mb-1">Outcome Forecasting</div>
                                        {weaveData.predictive_intelligence.outcome_forecasting.short_term_predictions && (
                                            <div>
                                                <span className="font-medium">Short-term:</span>
                                                <div className="ml-2">
                                                    {weaveData.predictive_intelligence.outcome_forecasting.short_term_predictions.slice(0, 2).map((pred: string, idx: number) => (
                                                        <div key={idx} className="text-xs">• {pred}</div>
                                                    ))}
                                                </div>
                                            </div>
                                        )}
                                        {weaveData.predictive_intelligence.outcome_forecasting.confidence_intervals && (
                                            <div className="mt-1">
                                                <span className="font-medium">Confidence:</span>
                                                <span className="ml-1">Success: {(weaveData.predictive_intelligence.outcome_forecasting.confidence_intervals.execution_success * 100).toFixed(0)}%</span>
                                            </div>
                                        )}
                                    </div>
                                )}
                                
                                {weaveData.predictive_intelligence.next_action_recommendations && (
                                    <div className="text-xs text-stone-600 bg-cyan-50 p-2 rounded">
                                        <div className="font-medium text-cyan-700 mb-1">Next Actions</div>
                                        {weaveData.predictive_intelligence.next_action_recommendations.immediate_actions && (
                                            <div>
                                                <span className="font-medium">Immediate:</span>
                                                <div className="ml-2">
                                                    {weaveData.predictive_intelligence.next_action_recommendations.immediate_actions.slice(0, 3).map((action: string, idx: number) => (
                                                        <div key={idx} className="text-xs">• {action}</div>
                                                    ))}
                                                </div>
                                            </div>
                                        )}
                                    </div>
                                )}
                                
                                {weaveData.predictive_intelligence.decision_support && (
                                    <div className="text-xs text-stone-600 bg-red-50 p-2 rounded">
                                        <div className="font-medium text-red-700 mb-1">Decision Support</div>
                                        <div><span className="font-medium">Confidence:</span> {(weaveData.predictive_intelligence.decision_support.decision_confidence * 100).toFixed(0)}%</div>
                                        {weaveData.predictive_intelligence.decision_support.critical_factors && (
                                            <div className="mt-1">
                                                <span className="font-medium">Critical Factors:</span>
                                                <div className="ml-2">
                                                    {weaveData.predictive_intelligence.decision_support.critical_factors.slice(0, 2).map((factor: string, idx: number) => (
                                                        <div key={idx} className="text-xs">• {factor}</div>
                                                    ))}
                                                </div>
                                            </div>
                                        )}
                                    </div>
                                )}
                            </div>
                        )}
                        
                        {/* Enhanced Reasoning Chain */}
                        {weaveData.reasoning_chain && weaveData.reasoning_chain.length > 0 && (
                            <div className="space-y-2">
                                <div className="text-xs font-medium text-stone-800 bg-stone-100 px-2 py-1 rounded">⚡ Reasoning Process</div>
                                <div className="text-xs text-stone-600 space-y-1 max-h-32 overflow-y-auto">
                                    {weaveData.reasoning_chain.slice(0, 4).map((step: any, idx: number) => (
                                        <div key={idx} className="bg-stone-50 p-2 rounded border">
                                            <div className="flex items-center space-x-2">
                                                <span className="font-mono text-orange-600 font-bold">{step.step}</span>
                                                <span className="font-medium text-stone-700 text-xs">{step.process}</span>
                                                {step.confidence && (
                                                    <span className="text-xs text-green-600">{(step.confidence * 100).toFixed(0)}%</span>
                                                )}
                                            </div>
                                            <div className="text-stone-600 mt-1 text-xs">{step.reasoning}</div>
                                        </div>
                                    ))}
                                </div>
                            </div>
                        )}
                        
                        {/* Intelligence Metrics */}
                        {weaveData.weave_metadata?.intelligence_metrics && (
                            <div className="space-y-2">
                                <div className="text-xs font-medium text-indigo-800 bg-indigo-50 px-2 py-1 rounded">📊 Intelligence Metrics</div>
                                <div className="grid grid-cols-2 gap-1 text-xs">
                                    <div className="bg-indigo-50 p-1 rounded">
                                        <span className="text-indigo-600">Reasoning:</span>
                                        <span className="font-mono ml-1">{(weaveData.weave_metadata.intelligence_metrics.reasoning_quality * 100).toFixed(0)}%</span>
                                    </div>
                                    <div className="bg-indigo-50 p-1 rounded">
                                        <span className="text-indigo-600">Adaptation:</span>
                                        <span className="font-mono ml-1">{(weaveData.weave_metadata.intelligence_metrics.adaptation_speed * 100).toFixed(0)}%</span>
                                    </div>
                                    <div className="bg-indigo-50 p-1 rounded">
                                        <span className="text-indigo-600">Learning:</span>
                                        <span className="font-mono ml-1">{(weaveData.weave_metadata.intelligence_metrics.learning_efficiency * 100).toFixed(0)}%</span>
                                    </div>
                                    <div className="bg-indigo-50 p-1 rounded">
                                        <span className="text-indigo-600">Prediction:</span>
                                        <span className="font-mono ml-1">{(weaveData.weave_metadata.intelligence_metrics.predictive_accuracy * 100).toFixed(0)}%</span>
                                    </div>
                                </div>
                            </div>
                        )}
                    </div>
                </div>
            )}
        </div>
    );
} 