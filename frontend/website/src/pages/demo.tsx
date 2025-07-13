import { createFileRoute } from "@tanstack/react-router";
import { ReactFlowProvider } from "@xyflow/react";

import { NavigationComponent } from "~/components/navigation/navigation-component";
import { RoboWeaveFlowBuilder } from "~/modules/flow-builder/roboweave-flow-builder";
import { SidebarModule } from "~/modules/sidebar/sidebar-module";
import { LiveProcessingPipeline } from "~/components/live-demo/live-processing-pipeline";

export const Route = createFileRoute("/demo")({
    component: DemoPage,
});

function DemoPage() {
    return (
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
                                Test your prompts and watch the pipeline execute with animated visual feedback.
                            </p>
                        </div>
                        
                        {/* Live Processing Pipeline */}
                        <LiveProcessingPipeline />
                        
                        <div className="bg-white/70 backdrop-blur-sm rounded-2xl border border-stone-200 shadow-lg h-[calc(100vh-200px)] overflow-hidden">
                            <ReactFlowProvider>
                                <div className="flex h-full">
                                    {/* Sidebar */}
                                    <div className="w-80 border-r border-stone-200/50 bg-amber-50/30 backdrop-blur-sm">
                                        <SidebarModule />
                                    </div>
                                    
                                    {/* Main Flow Area */}
                                    <div className="flex-1 relative">
                                        <RoboWeaveFlowBuilder />
                                    </div>
                                </div>
                            </ReactFlowProvider>
                        </div>
                        
                        <div className="mt-8 text-center">
                            <div className="bg-white/70 backdrop-blur-sm rounded-xl p-6 border border-stone-200 shadow-lg">
                                <h3 className="text-xl font-semibold text-stone-800 mb-4">How to Use the Flow Builder</h3>
                                <div className="grid grid-cols-1 md:grid-cols-4 gap-6 text-sm">
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            1
                                        </div>
                                        <p className="text-stone-600">Drag nodes from the sidebar to the canvas</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            2
                                        </div>
                                        <p className="text-stone-600">Connect nodes by dragging from handles</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            3
                                        </div>
                                        <p className="text-stone-600">Select nodes to configure properties</p>
                                    </div>
                                    <div className="text-center">
                                        <div className="w-8 h-8 bg-orange-500 rounded-full flex items-center justify-center mx-auto mb-2 text-white font-bold">
                                            4
                                        </div>
                                        <p className="text-stone-600">Build your custom robot control flow</p>
                                    </div>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>
            </main>
        </div>
    );
} 