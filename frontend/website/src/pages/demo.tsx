import { createFileRoute } from "@tanstack/react-router";
import { ReactFlowProvider } from "@xyflow/react";
import { useState, useEffect, createContext, useContext } from "react";

import { NavigationComponent } from "~/components/navigation/navigation-component";
import { RoboWeaveFlowBuilder } from "~/modules/flow-builder/roboweave-flow-builder";
import { SidebarModule } from "~/modules/sidebar/sidebar-module";
import { LiveDataPanel } from "~/components/live-metrics/live-data-panel";
import { LiveMetricsOverlay } from "~/components/live-metrics/live-metrics-overlay";

// Context for sharing live processing state across components
interface LiveProcessingContextType {
    agents: any[];
    steps: any[];
    weaveData: any;
    isProcessing: boolean;
    connectionStatus: string;
    setAgents: (agents: any[]) => void;
    setSteps: (steps: any[]) => void;
    setWeaveData: (data: any) => void;
    setIsProcessing: (processing: boolean) => void;
    setConnectionStatus: (status: string) => void;
}

const LiveProcessingContext = createContext<LiveProcessingContextType | null>(null);

export const useLiveProcessing = () => {
    const context = useContext(LiveProcessingContext);
    if (!context) {
        throw new Error('useLiveProcessing must be used within LiveProcessingProvider');
    }
    return context;
};

export const Route = createFileRoute("/demo")({
    component: DemoPage,
});

function DemoPage() {
    // Live processing state
    const [agents, setAgents] = useState<any[]>([]);
    const [steps, setSteps] = useState<any[]>([
        {
            id: 'prompt-input',
            name: 'Prompt Input',
            status: 'pending',
            description: 'Processing text and image inputs',
            icon: '📝'
        },
        {
            id: 'llm-processing',
            name: 'LLM Processing',
            status: 'pending',
            description: 'Gemini 1.5 Pro reasoning and high-level planning',
            icon: '🧠'
        },
        {
            id: 'motion-planning',
            name: 'Motion Planning',
            status: 'pending',
            description: 'Generating MuJoCo-compatible motion sequences',
            icon: '📐'
        },
        {
            id: 'weave-monitoring',
            name: 'Weave Monitoring',
            status: 'pending',
            description: 'Real-time observability and tracing',
            icon: '📊'
        }
    ]);
    const [weaveData, setWeaveData] = useState<any>(null);
    const [isProcessing, setIsProcessing] = useState(false);
    const [connectionStatus, setConnectionStatus] = useState('disconnected');

    const liveProcessingValue = {
        agents,
        steps,
        weaveData,
        isProcessing,
        connectionStatus,
        setAgents,
        setSteps,
        setWeaveData,
        setIsProcessing,
        setConnectionStatus,
    };

    return (
        <LiveProcessingContext.Provider value={liveProcessingValue}>
            <div className="min-h-screen relative">
                {/* Dogs background matching hero */}
                <div 
                    className="absolute inset-0 bg-cover bg-center bg-no-repeat opacity-60"
                    style={{ backgroundImage: 'url(/dogs-background.png)' }}
                ></div>
                
                {/* Background overlay matching hero */}
                <div className="absolute inset-0 bg-gradient-to-br from-amber-50/95 via-orange-50/90 to-stone-100/95"></div>
                
                {/* Subtle animated background elements matching hero */}
                <div className="absolute inset-0 overflow-hidden">
                    <div className="absolute top-1/4 left-1/4 w-64 h-64 bg-orange-200/10 rounded-full blur-3xl animate-pulse"></div>
                    <div className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-amber-200/10 rounded-full blur-3xl animate-pulse delay-1000"></div>
                </div>

                <NavigationComponent />
                
                <main className="relative">
                    <div className="pt-16">
                        <div className="max-w-full mx-auto px-4 sm:px-6 lg:px-8 py-8">
                            <div className="text-center mb-8">
                                <h1 className="text-3xl md:text-4xl font-bold text-stone-800 mb-4">
                                    RoboWeave Interactive Flow Builder
                                </h1>
                                <p className="text-xl text-stone-600 max-w-4xl mx-auto">
                                    Live multimodal robot control with real-time Gemini processing and Weave observability. 
                                    Use the 🚀 Live Processing panel to test your prompts and watch the pipeline execute with animated visual feedback.
                                </p>
                            </div>
                            
                            <div className="bg-white/70 backdrop-blur-sm rounded-2xl border border-stone-200 shadow-lg h-[calc(100vh-200px)] overflow-hidden">
                                <ReactFlowProvider>
                                    <div className="flex h-full">
                                        {/* Left Sidebar - Input Controls */}
                                        <div className="w-80 border-r border-stone-200/50 bg-amber-50/30 backdrop-blur-sm">
                                            <SidebarModule />
                                        </div>
                                        
                                                                {/* Main Flow Area with Live Metrics Overlay */}
                        <div className="flex-1 relative bg-slate-50/80 backdrop-blur-sm">
                            <div className="absolute inset-0 bg-gradient-to-br from-slate-50/90 via-stone-50/90 to-gray-50/90">
                                <RoboWeaveFlowBuilder 
                                    agents={agents}
                                    steps={steps}
                                    weaveData={weaveData}
                                    isProcessing={isProcessing}
                                />
                            </div>
                            <LiveMetricsOverlay 
                                agents={agents}
                                steps={steps}
                                weaveData={weaveData}
                                isProcessing={isProcessing}
                            />
                        </div>

                                        {/* Right Panel - Live Data Tabs */}
                                        <LiveDataPanel 
                                            agents={agents}
                                            steps={steps}
                                            weaveData={weaveData}
                                            isProcessing={isProcessing}
                                            connectionStatus={connectionStatus}
                                        />
                                    </div>
                                </ReactFlowProvider>
                            </div>
                        
                        <div className="mt-8 text-center">
                            <div className="bg-white/70 backdrop-blur-sm rounded-xl p-6 border border-stone-200 shadow-lg">
                                <h3 className="text-xl font-semibold text-stone-800 mb-4">How to Use RoboWeave</h3>
                                <div className="grid grid-cols-1 md:grid-cols-4 gap-6 text-sm">
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            1
                                        </div>
                                        <p className="text-stone-600">Click the 🚀 button to open Live Processing</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            2
                                        </div>
                                        <p className="text-stone-600">Enter text prompts and upload images</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            3
                                        </div>
                                        <p className="text-stone-600">Watch real-time pipeline execution</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            4
                                        </div>
                                        <p className="text-stone-600">Spawn multiple agents and build flows</p>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </main>
        </div>
        </LiveProcessingContext.Provider>
    );
} 