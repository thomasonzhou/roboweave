import React from 'react';

interface LiveMetricsOverlayProps {
    agents: any[];
    steps: any[];
    weaveData: any;
    isProcessing: boolean;
}

export function LiveMetricsOverlay({ agents, steps, weaveData, isProcessing }: LiveMetricsOverlayProps) {
    if (!isProcessing && agents.length === 0) return null;

    return (
        <div className="absolute top-4 left-4 z-50 pointer-events-none">
            {/* Processing Status Indicator */}
            {isProcessing && (
                <div className="bg-black/80 backdrop-blur-sm rounded-lg p-3 mb-3 border border-orange-500/50 animate-pulse">
                    <div className="flex items-center space-x-2">
                        <div className="w-3 h-3 bg-orange-500 rounded-full animate-ping"></div>
                        <span className="text-orange-400 text-sm font-medium">Processing Pipeline</span>
                    </div>
                </div>
            )}

            {/* Active Agents Counter */}
            {agents.length > 0 && (
                <div className="bg-black/80 backdrop-blur-sm rounded-lg p-3 mb-3 border border-blue-500/50">
                    <div className="flex items-center space-x-2">
                        <span className="text-blue-400 text-sm font-medium">
                            🤖 {agents.length} Active Agent{agents.length !== 1 ? 's' : ''}
                        </span>
                    </div>
                    <div className="flex space-x-1 mt-2">
                        {agents.map((agent, idx) => (
                            <div
                                key={agent.id}
                                className="w-2 h-2 rounded-full animate-bounce"
                                style={{ 
                                    backgroundColor: agent.color,
                                    animationDelay: `${idx * 200}ms`
                                }}
                            ></div>
                        ))}
                    </div>
                </div>
            )}

            {/* Current Step Indicator */}
            {steps.some(s => s.status === 'processing') && (
                <div className="bg-black/80 backdrop-blur-sm rounded-lg p-3 mb-3 border border-green-500/50">
                    {steps.map((step, idx) => {
                        if (step.status !== 'processing') return null;
                        return (
                            <div key={step.id} className="flex items-center space-x-2">
                                <span className="text-lg">{step.icon}</span>
                                <span className="text-green-400 text-sm font-medium">{step.name}</span>
                                <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse"></div>
                            </div>
                        );
                    })}
                </div>
            )}

            {/* Live Weave Metrics */}
            {weaveData && weaveData.observability_data && (
                <div className="bg-black/80 backdrop-blur-sm rounded-lg p-3 border border-purple-500/50">
                    <div className="text-purple-400 text-xs font-medium mb-2">⚡ Live Metrics</div>
                    <div className="space-y-1 text-xs">
                        {weaveData.observability_data.execution_time && (
                            <div className="flex justify-between">
                                <span className="text-gray-400">Execution:</span>
                                <span className="text-purple-300">{weaveData.observability_data.execution_time.toFixed(2)}s</span>
                            </div>
                        )}
                        {weaveData.parameters_extracted !== undefined && (
                            <div className="flex justify-between">
                                <span className="text-gray-400">Parameters:</span>
                                <span className="text-blue-300">{weaveData.parameters_extracted}</span>
                            </div>
                        )}
                        {weaveData.motion_analysis?.stability_margin && (
                            <div className="flex justify-between">
                                <span className="text-gray-400">Stability:</span>
                                <span className="text-green-300">{(weaveData.motion_analysis.stability_margin * 100).toFixed(1)}%</span>
                            </div>
                        )}
                    </div>
                </div>
            )}
        </div>
    );
} 