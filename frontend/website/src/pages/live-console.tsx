import { createFileRoute } from "@tanstack/react-router";

import { NavigationComponent } from "~/components/navigation/navigation-component";
import { LiveConsole } from "~/modules/live-console/components/live-console";

export const Route = createFileRoute("/live-console")({
    component: LiveConsolePage,
});

function LiveConsolePage() {
    return (
        <div className="min-h-screen bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 text-white">
            <NavigationComponent />
            
            <main className="relative">
                <div className="container mx-auto px-6 py-8">
                    <div className="mb-8">
                        <h1 className="text-4xl font-bold mb-4 bg-gradient-to-r from-purple-400 to-pink-400 bg-clip-text text-transparent">
                            Live Robot Console
                        </h1>
                        <p className="text-xl text-gray-300">
                            Control your robot with voice commands powered by Gemini AI
                        </p>
                    </div>
                    
                    <LiveConsole />
                </div>
            </main>
        </div>
    );
}
