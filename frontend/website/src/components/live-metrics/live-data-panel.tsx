import React, { useState } from 'react';

interface LiveDataPanelProps {
    agents: any[];
    steps: any[];
    weaveData: any;
    isProcessing: boolean;
    connectionStatus: string;
}

type TabType = 'pipeline' | 'agents' | 'weave' | 'metrics' | 'motion';

export function LiveDataPanel({ agents, steps, weaveData, isProcessing, connectionStatus }: LiveDataPanelProps) {
    const [activeTab, setActiveTab] = useState<TabType>('pipeline');

    const tabs = [
        { id: 'pipeline' as TabType, label: 'Pipeline', icon: '🔄', count: steps.filter(s => s.status === 'processing' || s.status === 'completed').length },
        { id: 'agents' as TabType, label: 'Agents', icon: '🤖', count: agents.length },
        { id: 'weave' as TabType, label: 'Weave', icon: '📊', count: weaveData ? 1 : 0 },
        { id: 'metrics' as TabType, label: 'Metrics', icon: '⚡', count: weaveData?.observability_data ? Object.keys(weaveData.observability_data).length : 0 },
        { id: 'motion' as TabType, label: 'Motion', icon: '🦾', count: weaveData?.motion_analysis ? Object.keys(weaveData.motion_analysis).length : 0 }
    ];

    return (
        <div className="w-96 bg-white/70 backdrop-blur-sm border-l border-stone-200 shadow-lg flex flex-col h-full">
            {/* Connection Status Header */}
            <div className="p-4 border-b border-stone-200">
                <div className="flex items-center justify-between">
                    <h3 className="text-lg font-semibold text-stone-800">Live Processing Data</h3>
                    <div className="flex items-center space-x-2">
                        <div className={`w-2 h-2 rounded-full ${
                            connectionStatus === 'connected' ? 'bg-green-500' : 
                            connectionStatus === 'connecting' ? 'bg-yellow-500 animate-pulse' : 'bg-red-500'
                        }`}></div>
                        <span className="text-xs text-stone-600 capitalize">{connectionStatus}</span>
                    </div>
                </div>
            </div>

            {/* Tab Navigation */}
            <div className="flex border-b border-stone-200 bg-stone-50/50">
                {tabs.map((tab) => (
                    <button
                        key={tab.id}
                        onClick={() => setActiveTab(tab.id)}
                        className={`flex-1 px-2 py-3 text-xs font-medium transition-colors relative ${
                            activeTab === tab.id
                                ? 'text-orange-600 bg-white border-b-2 border-orange-500'
                                : 'text-stone-600 hover:text-orange-600 hover:bg-stone-100'
                        }`}
                    >
                        <div className="flex flex-col items-center space-y-1">
                            <span className="text-sm">{tab.icon}</span>
                            <span>{tab.label}</span>
                            {tab.count > 0 && (
                                <span className={`text-xs px-1.5 py-0.5 rounded-full ${
                                    activeTab === tab.id ? 'bg-orange-100 text-orange-600' : 'bg-stone-200 text-stone-600'
                                }`}>
                                    {tab.count}
                                </span>
                            )}
                        </div>
                    </button>
                ))}
            </div>

            {/* Tab Content */}
            <div className="flex-1 overflow-y-auto p-4">
                {activeTab === 'pipeline' && (
                    <div className="space-y-3">
                        <h4 className="text-sm font-semibold text-stone-700 mb-3">Pipeline Status</h4>
                        {steps.map((step, index) => (
                            <div key={step.id} className={`p-3 rounded-lg border transition-all duration-500 ${
                                step.status === 'completed' ? 'bg-green-50 border-green-200' :
                                step.status === 'processing' ? 'bg-orange-50 border-orange-200 animate-pulse' :
                                step.status === 'error' ? 'bg-red-50 border-red-200' :
                                'bg-stone-50 border-stone-200'
                            }`}>
                                <div className="flex items-center justify-between">
                                    <div className="flex items-center space-x-2">
                                        <span className="text-lg">{step.icon}</span>
                                        <div>
                                            <div className="text-sm font-medium text-stone-800">{step.name}</div>
                                            <div className="text-xs text-stone-600">{step.description}</div>
                                        </div>
                                    </div>
                                    <div className={`w-2 h-2 rounded-full ${
                                        step.status === 'completed' ? 'bg-green-500' :
                                        step.status === 'processing' ? 'bg-orange-500 animate-pulse' :
                                        step.status === 'error' ? 'bg-red-500' :
                                        'bg-stone-400'
                                    }`}></div>
                                </div>
                                
                                {step.reasoning && (
                                    <div className="mt-2 text-xs text-stone-600 bg-stone-100 p-2 rounded">
                                        <span className="font-medium">Reasoning:</span> {step.reasoning}
                                    </div>
                                )}
                                
                                {step.timing && (
                                    <div className="mt-1 text-xs text-stone-500">
                                        Completed in {step.timing}ms
                                    </div>
                                )}
                            </div>
                        ))}
                    </div>
                )}

                {activeTab === 'agents' && (
                    <div className="space-y-3">
                        <h4 className="text-sm font-semibold text-stone-700 mb-3">Active Agents ({agents.length})</h4>
                        {agents.length === 0 ? (
                            <div className="text-center py-8 text-stone-500">
                                <div className="text-4xl mb-2">🤖</div>
                                <div className="text-sm">No active agents</div>
                                <div className="text-xs">Spawn an agent to see live tracking</div>
                            </div>
                        ) : (
                            agents.map((agent) => (
                                <div key={agent.id} className="p-3 bg-stone-50 rounded-lg border border-stone-200">
                                    <div className="flex items-center justify-between mb-2">
                                        <div className="flex items-center space-x-2">
                                            <div 
                                                className="w-3 h-3 rounded-full"
                                                style={{ backgroundColor: agent.color }}
                                            ></div>
                                            <span className="text-sm font-medium text-stone-700">
                                                Agent {agent.id.split('-')[1]}
                                            </span>
                                        </div>
                                        <span className={`text-xs px-2 py-1 rounded-full ${
                                            agent.status === 'completed' ? 'bg-green-100 text-green-700' :
                                            agent.status === 'active' ? 'bg-blue-100 text-blue-700' :
                                            'bg-red-100 text-red-700'
                                        }`}>
                                            {agent.status}
                                        </span>
                                    </div>
                                    <div className="space-y-1 text-xs text-stone-600">
                                        <div>Step: {agent.currentStep + 1}/4</div>
                                        <div>Runtime: {((Date.now() - agent.startTime) / 1000).toFixed(1)}s</div>
                                    </div>
                                    
                                    {/* Agent Progress Bar */}
                                    <div className="mt-2">
                                        <div className="w-full bg-stone-200 rounded-full h-1">
                                            <div 
                                                className="bg-gradient-to-r from-blue-500 to-green-500 h-1 rounded-full transition-all duration-500"
                                                style={{ 
                                                    width: `${((agent.currentStep + 1) / 4) * 100}%`,
                                                    backgroundColor: agent.color 
                                                }}
                                            ></div>
                                        </div>
                                    </div>
                                </div>
                            ))
                        )}
                    </div>
                )}

                {activeTab === 'weave' && (
                    <div className="space-y-4">
                        <h4 className="text-sm font-semibold text-stone-700 mb-3">🧶 Weave Analysis</h4>
                        {weaveData ? (
                            <div className="space-y-4">
                                {/* Trace Header */}
                                <div className="p-3 bg-green-50 rounded-lg border border-green-200">
                                    <div className="flex items-center justify-between mb-2">
                                        <span className="text-sm font-medium text-green-800">Trace Active</span>
                                        <span className="text-xs text-green-600">✅ Live</span>
                                    </div>
                                    <div className="text-xs text-green-700 font-mono break-all">
                                        {weaveData.trace_id}
                                    </div>
                                    <div className="text-xs text-green-600 mt-1">
                                        {weaveData.robot_type} • {weaveData.simulator}
                                    </div>
                                </div>

                                {/* Motion Analysis */}
                                {weaveData.motion_analysis && (
                                    <div className="bg-blue-50 p-3 rounded-lg border border-blue-200">
                                        <h5 className="text-sm font-medium text-blue-800 mb-2">Motion Analysis</h5>
                                        <div className="space-y-2 text-xs">
                                            <div className="grid grid-cols-2 gap-2">
                                                <div>
                                                    <span className="text-blue-600">Primitives:</span>
                                                    <span className="font-mono ml-2">{weaveData.motion_analysis.motion_primitive_count || 0}</span>
                                                </div>
                                                <div>
                                                    <span className="text-blue-600">Stability:</span>
                                                    <span className="font-mono ml-2">{(weaveData.motion_analysis.stability_margin * 100).toFixed(1)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-blue-600">Efficiency:</span>
                                                    <span className="font-mono ml-2">{(weaveData.motion_analysis.energy_efficiency * 100).toFixed(1)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-blue-600">Smoothness:</span>
                                                    <span className="font-mono ml-2">{(weaveData.motion_analysis.trajectory_smoothness * 100).toFixed(1)}%</span>
                                                </div>
                                            </div>
                                            
                                            {weaveData.motion_analysis.quadruped_gait_patterns?.length > 0 && (
                                                <div className="mt-2">
                                                    <span className="text-blue-600">Gait Patterns:</span>
                                                    <div className="flex flex-wrap gap-1 mt-1">
                                                        {weaveData.motion_analysis.quadruped_gait_patterns.map((gait: string, idx: number) => (
                                                            <span key={idx} className="bg-blue-100 text-blue-700 px-2 py-1 rounded text-xs">{gait}</span>
                                                        ))}
                                                    </div>
                                                </div>
                                            )}
                                            
                                            {weaveData.motion_analysis.velocity_profiles?.length > 0 && (
                                                <div className="mt-2">
                                                    <span className="text-blue-600">Velocity Profiles:</span>
                                                    <div className="font-mono text-xs text-blue-700">
                                                        [min: {Math.min(...weaveData.motion_analysis.velocity_profiles)}, 
                                                         max: {Math.max(...weaveData.motion_analysis.velocity_profiles)}, 
                                                         avg: {(weaveData.motion_analysis.velocity_profiles.reduce((a: number, b: number) => a + b, 0) / weaveData.motion_analysis.velocity_profiles.length).toFixed(1)}]
                                                    </div>
                                                </div>
                                            )}
                                            
                                            {weaveData.motion_analysis.balance_metrics && (
                                                <div className="mt-2">
                                                    <span className="text-blue-600">Balance Metrics:</span>
                                                    <div className="text-xs text-blue-700 ml-2">
                                                        <div>Avg: {(weaveData.motion_analysis.balance_metrics.avg_stability * 100).toFixed(1)}%</div>
                                                        <div>Min: {(weaveData.motion_analysis.balance_metrics.min_stability * 100).toFixed(1)}%</div>
                                                        <div>Max: {(weaveData.motion_analysis.balance_metrics.max_stability * 100).toFixed(1)}%</div>
                                                    </div>
                                                </div>
                                            )}
                                        </div>
                                    </div>
                                )}

                                {/* MuJoCo Integration */}
                                {weaveData.mujoco_integration && (
                                    <div className="bg-purple-50 p-3 rounded-lg border border-purple-200">
                                        <h5 className="text-sm font-medium text-purple-800 mb-2">MuJoCo Integration</h5>
                                        <div className="space-y-2 text-xs">
                                            <div className="grid grid-cols-2 gap-2">
                                                <div>
                                                    <span className="text-purple-600">Sim Freq:</span>
                                                    <span className="font-mono ml-2">{weaveData.mujoco_integration.simulation_frequency}Hz</span>
                                                </div>
                                                <div>
                                                    <span className="text-purple-600">Control Freq:</span>
                                                    <span className="font-mono ml-2">{weaveData.mujoco_integration.control_frequency}Hz</span>
                                                </div>
                                                <div>
                                                    <span className="text-purple-600">Dynamics:</span>
                                                    <span className="font-mono ml-2">{weaveData.mujoco_integration.dynamics_modeling}</span>
                                                </div>
                                                <div>
                                                    <span className="text-purple-600">Actuators:</span>
                                                    <span className="font-mono ml-2">{weaveData.mujoco_integration.actuator_models}</span>
                                                </div>
                                            </div>
                                            
                                            {weaveData.mujoco_integration.sensor_integration?.length > 0 && (
                                                <div className="mt-2">
                                                    <span className="text-purple-600">Sensors:</span>
                                                    <div className="flex flex-wrap gap-1 mt-1">
                                                        {weaveData.mujoco_integration.sensor_integration.map((sensor: string, idx: number) => (
                                                            <span key={idx} className="bg-purple-100 text-purple-700 px-2 py-1 rounded text-xs">{sensor}</span>
                                                        ))}
                                                    </div>
                                                </div>
                                            )}
                                            
                                            <div className="mt-2 flex flex-wrap gap-2">
                                                {weaveData.mujoco_integration.physics_validation && (
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Physics</span>
                                                )}
                                                {weaveData.mujoco_integration.collision_avoidance && (
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Collision</span>
                                                )}
                                                {weaveData.mujoco_integration.constraint_satisfaction && (
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Constraints</span>
                                                )}
                                            </div>
                                        </div>
                                    </div>
                                )}

                                {/* Performance Metrics */}
                                {weaveData.observability_data && (
                                    <div className="bg-orange-50 p-3 rounded-lg border border-orange-200">
                                        <h5 className="text-sm font-medium text-orange-800 mb-2">Performance Metrics</h5>
                                        <div className="space-y-2 text-xs">
                                            <div className="grid grid-cols-2 gap-2">
                                                <div>
                                                    <span className="text-orange-600">Execution:</span>
                                                    <span className="font-mono ml-2">{(weaveData.observability_data.execution_time * 1000).toFixed(0)}ms</span>
                                                </div>
                                                <div>
                                                    <span className="text-orange-600">API Latency:</span>
                                                    <span className="font-mono ml-2">{weaveData.observability_data.api_latency_ms}ms</span>
                                                </div>
                                                <div>
                                                    <span className="text-orange-600">LLM Calls:</span>
                                                    <span className="font-mono ml-2">{weaveData.observability_data.llm_calls}</span>
                                                </div>
                                                <div>
                                                    <span className="text-orange-600">Tokens:</span>
                                                    <span className="font-mono ml-2">{weaveData.observability_data.tokens_processed}</span>
                                                </div>
                                                <div>
                                                    <span className="text-orange-600">Success Rate:</span>
                                                    <span className="font-mono ml-2">{(weaveData.observability_data.success_rate * 100).toFixed(1)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-orange-600">Performance:</span>
                                                    <span className="font-mono ml-2">{(weaveData.observability_data.performance_score * 100).toFixed(1)}%</span>
                                                </div>
                                            </div>
                                            
                                            <div className="mt-2">
                                                <span className="text-orange-600">Accuracy:</span>
                                                <span className="font-mono ml-2 bg-orange-100 text-orange-700 px-2 py-1 rounded">{weaveData.observability_data.planning_accuracy}</span>
                                            </div>
                                        </div>
                                    </div>
                                )}

                                {/* System Diagnostics */}
                                {weaveData.system_diagnostics && (
                                    <div className="bg-red-50 p-3 rounded-lg border border-red-200">
                                        <h5 className="text-sm font-medium text-red-800 mb-2">System Diagnostics</h5>
                                        <div className="space-y-2 text-xs">
                                            <div className="grid grid-cols-2 gap-2">
                                                <div>
                                                    <span className="text-red-600">Battery:</span>
                                                    <span className="font-mono ml-2">{(weaveData.system_diagnostics.battery_level * 100).toFixed(0)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-red-600">CPU:</span>
                                                    <span className="font-mono ml-2">{(weaveData.system_diagnostics.cpu_usage * 100).toFixed(0)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-red-600">Memory:</span>
                                                    <span className="font-mono ml-2">{(weaveData.system_diagnostics.memory_usage * 100).toFixed(0)}%</span>
                                                </div>
                                                <div>
                                                    <span className="text-red-600">Temperature:</span>
                                                    <span className="font-mono ml-2">{weaveData.system_diagnostics.temperature}°C</span>
                                                </div>
                                                <div>
                                                    <span className="text-red-600">Network:</span>
                                                    <span className="font-mono ml-2">{weaveData.system_diagnostics.network_latency?.toFixed(1)}ms</span>
                                                </div>
                                                <div>
                                                    <span className="text-red-600">Status:</span>
                                                    <span className="font-mono ml-2">{weaveData.system_diagnostics.connection_status}</span>
                                                </div>
                                            </div>
                                            
                                            <div className="mt-2">
                                                <span className="text-red-600">Health:</span>
                                                <div className="flex flex-wrap gap-1 mt-1">
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Hardware</span>
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Software</span>
                                                    <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Calibrated</span>
                                                </div>
                                            </div>
                                        </div>
                                    </div>
                                )}

                                {/* Reasoning Chain */}
                                {weaveData.reasoning_chain && weaveData.reasoning_chain.length > 0 && (
                                    <div className="bg-stone-50 p-3 rounded-lg border border-stone-200">
                                        <h5 className="text-sm font-medium text-stone-800 mb-2">Reasoning Chain</h5>
                                        <div className="space-y-2 text-xs max-h-48 overflow-y-auto">
                                            {weaveData.reasoning_chain.map((step: any, idx: number) => (
                                                <div key={idx} className="flex items-start space-x-2 p-2 bg-white rounded border">
                                                    <span className="font-mono text-orange-600 font-bold flex-shrink-0">{step.step}</span>
                                                    <div className="flex-1">
                                                        <div className="font-medium text-stone-700">{step.process}</div>
                                                        <div className="text-stone-600 mt-1">{step.reasoning}</div>
                                                    </div>
                                                </div>
                                            ))}
                                        </div>
                                    </div>
                                )}

                                {/* Weave Metadata */}
                                {weaveData.weave_metadata && (
                                    <div className="bg-indigo-50 p-3 rounded-lg border border-indigo-200">
                                        <h5 className="text-sm font-medium text-indigo-800 mb-2">Weave Metadata</h5>
                                        <div className="space-y-2 text-xs">
                                            <div>
                                                <span className="text-indigo-600">Project:</span>
                                                <span className="font-mono ml-2">{weaveData.weave_metadata.project_name}</span>
                                            </div>
                                            <div>
                                                <span className="text-indigo-600">Run ID:</span>
                                                <span className="font-mono ml-2">{weaveData.weave_metadata.run_id}</span>
                                            </div>
                                            <div>
                                                <span className="text-indigo-600">Experiment:</span>
                                                <span className="font-mono ml-2">{weaveData.weave_metadata.experiment_id}</span>
                                            </div>
                                            <div>
                                                <span className="text-indigo-600">Model Version:</span>
                                                <span className="font-mono ml-2">{weaveData.weave_metadata.model_version}</span>
                                            </div>
                                            
                                            {weaveData.weave_metadata.custom_tags && (
                                                <div className="mt-2">
                                                    <span className="text-indigo-600">Tags:</span>
                                                    <div className="flex flex-wrap gap-1 mt-1">
                                                        {weaveData.weave_metadata.custom_tags.map((tag: string, idx: number) => (
                                                            <span key={idx} className="bg-indigo-100 text-indigo-700 px-2 py-1 rounded text-xs">{tag}</span>
                                                        ))}
                                                    </div>
                                                </div>
                                            )}
                                            
                                            <div className="mt-2 flex flex-wrap gap-2">
                                                <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Tracking</span>
                                                <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Artifacts</span>
                                                <span className="bg-green-100 text-green-700 px-2 py-1 rounded text-xs">✓ Performance</span>
                                            </div>
                                        </div>
                                    </div>
                                )}

                                {/* JSON Export */}
                                <div className="bg-gray-50 p-3 rounded-lg border border-gray-200">
                                    <h5 className="text-sm font-medium text-gray-800 mb-2">Raw JSON Export</h5>
                                    <div className="bg-black text-green-400 p-2 rounded text-xs font-mono max-h-32 overflow-y-auto">
                                        <pre>{JSON.stringify(weaveData, null, 2)}</pre>
                                    </div>
                                </div>
                            </div>
                        ) : (
                            <div className="text-center py-8 text-stone-500">
                                <div className="text-4xl mb-2">🧶</div>
                                <div className="text-sm">No weave data available</div>
                                <div className="text-xs mt-1">Start processing to see traces</div>
                            </div>
                        )}
                    </div>
                )}

                {activeTab === 'metrics' && (
                    <div className="space-y-3">
                        <h4 className="text-sm font-semibold text-stone-700 mb-3">Performance Metrics</h4>
                        {weaveData?.observability_data ? (
                            <div className="space-y-3">
                                {/* Real-time Metrics */}
                                <div className="grid grid-cols-1 gap-2">
                                    {Object.entries(weaveData.observability_data).map(([key, value]) => (
                                        <div key={key} className="p-3 bg-stone-50 rounded-lg border">
                                            <div className="flex justify-between items-center">
                                                <span className="text-xs text-stone-600 capitalize">
                                                    {key.replace(/_/g, ' ')}
                                                </span>
                                                <span className="text-sm font-medium text-stone-800">
                                                    {typeof value === 'number' ? value.toFixed(2) : String(value)}
                                                    {key.includes('time') && 's'}
                                                    {key.includes('margin') && '%'}
                                                </span>
                                            </div>
                                        </div>
                                    ))}
                                </div>

                                {/* Model Status */}
                                {weaveData.observability_data.weave_model_used !== undefined && (
                                    <div className={`p-3 rounded-lg border ${
                                        weaveData.observability_data.weave_model_used 
                                            ? 'bg-green-50 border-green-200' 
                                            : 'bg-yellow-50 border-yellow-200'
                                    }`}>
                                        <div className="flex items-center justify-between">
                                            <span className="text-sm font-medium">Weave Model</span>
                                            <span className={`text-sm ${
                                                weaveData.observability_data.weave_model_used 
                                                    ? 'text-green-600' 
                                                    : 'text-yellow-600'
                                            }`}>
                                                {weaveData.observability_data.weave_model_used ? '✅ Active' : '⚠️ Fallback'}
                                            </span>
                                        </div>
                                    </div>
                                )}
                            </div>
                        ) : (
                            <div className="text-center py-8 text-stone-500">
                                <div className="text-4xl mb-2">⚡</div>
                                <div className="text-sm">No metrics available</div>
                                <div className="text-xs">Start processing to see live metrics</div>
                            </div>
                        )}
                    </div>
                )}

                {activeTab === 'motion' && (
                    <div className="space-y-3">
                        <h4 className="text-sm font-semibold text-stone-700 mb-3">Motion Analysis</h4>
                        {weaveData?.motion_analysis ? (
                            <div className="space-y-3">
                                {/* Gait Patterns */}
                                {weaveData.motion_analysis.quadruped_gait_patterns && (
                                    <div className="p-3 bg-blue-50 rounded-lg border border-blue-200">
                                        <div className="text-sm font-medium text-blue-800 mb-2">Gait Patterns</div>
                                        <div className="flex flex-wrap gap-1">
                                            {weaveData.motion_analysis.quadruped_gait_patterns.map((gait: string, idx: number) => (
                                                <span key={idx} className="text-xs bg-blue-100 text-blue-700 px-2 py-1 rounded-full">
                                                    {gait}
                                                </span>
                                            ))}
                                        </div>
                                    </div>
                                )}

                                {/* Stability */}
                                {weaveData.motion_analysis.stability_margin && (
                                    <div className="p-3 bg-green-50 rounded-lg border border-green-200">
                                        <div className="text-sm font-medium text-green-800 mb-2">Stability Analysis</div>
                                        <div className="space-y-2">
                                            <div className="flex justify-between text-xs">
                                                <span>Stability Margin:</span>
                                                <span className="font-medium text-green-700">
                                                    {(weaveData.motion_analysis.stability_margin * 100).toFixed(1)}%
                                                </span>
                                            </div>
                                            <div className="w-full bg-green-200 rounded-full h-2">
                                                <div 
                                                    className="bg-green-500 h-2 rounded-full transition-all duration-1000"
                                                    style={{ width: `${weaveData.motion_analysis.stability_margin * 100}%` }}
                                                ></div>
                                            </div>
                                        </div>
                                    </div>
                                )}

                                {/* Joint Configuration */}
                                {weaveData.motion_analysis.joint_configurations && (
                                    <div className="p-3 bg-purple-50 rounded-lg border border-purple-200">
                                        <div className="text-sm font-medium text-purple-800 mb-2">Joint Configuration</div>
                                        <div className="text-xs text-purple-700">
                                            {weaveData.motion_analysis.joint_configurations} active joints configured
                                        </div>
                                    </div>
                                )}

                                {/* Motion Primitive Count */}
                                {weaveData.motion_analysis.motion_primitive_count && (
                                    <div className="p-3 bg-orange-50 rounded-lg border border-orange-200">
                                        <div className="text-sm font-medium text-orange-800 mb-2">Motion Primitives</div>
                                        <div className="flex items-center justify-between">
                                            <span className="text-xs text-orange-700">Generated:</span>
                                            <div className="flex space-x-1">
                                                {Array.from({length: weaveData.motion_analysis.motion_primitive_count}, (_, i) => (
                                                    <div key={i} className="w-2 h-2 bg-orange-500 rounded animate-bounce" 
                                                         style={{animationDelay: `${i * 100}ms`}}></div>
                                                ))}
                                            </div>
                                        </div>
                                    </div>
                                )}
                            </div>
                        ) : (
                            <div className="text-center py-8 text-stone-500">
                                <div className="text-4xl mb-2">🦾</div>
                                <div className="text-sm">No motion data</div>
                                <div className="text-xs">Execute a command to see motion analysis</div>
                            </div>
                        )}
                    </div>
                )}
            </div>
        </div>
    );
} 