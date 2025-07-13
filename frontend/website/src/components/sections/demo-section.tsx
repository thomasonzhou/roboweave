import { useState } from "react";

export function DemoSection() {
    const [activeDemo, setActiveDemo] = useState(0);

    const demos = [
        {
            id: 0,
            title: "Forward Navigation Without Collision",
            description: "System generates obstacle-aware straight-line motion using LLM and planner",
            prompt: "Walk forward without hitting anything",
            process: [
                "Analyze visual scene for obstacles",
                "Generate collision-free path planning",
                "Execute controlled forward motion",
                "Monitor real-time safety constraints"
            ],
            color: "from-blue-500 to-cyan-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 7l5 5m0 0l-5 5m5-5H6" />
                </svg>
            )
        },
        {
            id: 1,
            title: "Orientation Change",
            description: "Pose change verified via simulation and robot telemetry",
            prompt: "Turn left 45 degrees",
            process: [
                "Parse directional intent from command",
                "Calculate precise rotation parameters",
                "Execute controlled angular motion",
                "Verify final orientation alignment"
            ],
            color: "from-purple-500 to-pink-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                </svg>
            )
        },
        {
            id: 2,
            title: "Simple Task Execution",
            description: "Combines pose planning with action sequence construction",
            prompt: "Stand up and initiate handshake",
            process: [
                "Decompose complex task into steps",
                "Plan sequential motion primitives",
                "Execute coordinated body movements",
                "Monitor task completion status"
            ],
            color: "from-green-500 to-teal-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 11.5V14m0-2.5v-6a1.5 1.5 0 113 0m-3 6a1.5 1.5 0 00-3 0v2a7.5 7.5 0 0015 0v-5a1.5 1.5 0 00-3 0m-6-3V11m0-5.5v-1a1.5 1.5 0 013 0v1m0 0V11m0-5.5a1.5 1.5 0 013 0v3m0 0V11" />
                </svg>
            )
        }
    ];

    const currentDemo = demos[activeDemo];

    return (
        <section id="demo" className="py-20 bg-orange-50/40">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="text-center mb-16">
                    <h2 className="text-3xl md:text-4xl font-bold text-light-50 mb-4">
                        Live Demonstrations
                    </h2>
                    <p className="text-xl text-light-300 max-w-3xl mx-auto">
                        Three representative success cases implemented and validated in the hackathon setting, 
                        showcasing multimodal control capabilities.
                    </p>
                </div>

                {/* Demo Selector */}
                <div className="flex flex-wrap justify-center gap-4 mb-12">
                    {demos.map((demo) => (
                        <button
                            key={demo.id}
                            onClick={() => setActiveDemo(demo.id)}
                            className={`px-6 py-3 rounded-lg font-medium transition-all duration-200 ${
                                activeDemo === demo.id
                                    ? `bg-gradient-to-r ${demo.color} text-white shadow-lg`
                                    : "bg-dark-800/50 text-light-300 hover:bg-dark-700 border border-dark-600"
                            }`}
                        >
                            <div className="flex items-center gap-2">
                                <span className="w-5 h-5">{demo.icon}</span>
                                {demo.title}
                            </div>
                        </button>
                    ))}
                </div>

                {/* Demo Content */}
                <div className="bg-dark-800/50 backdrop-blur-sm rounded-2xl p-8 border border-dark-600">
                    <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
                        {/* Left Side - Demo Details */}
                        <div>
                            <div className="flex items-center gap-3 mb-6">
                                <div className={`w-12 h-12 bg-gradient-to-r ${currentDemo.color} rounded-lg flex items-center justify-center text-white`}>
                                    {currentDemo.icon}
                                </div>
                                <div>
                                    <h3 className="text-2xl font-bold text-light-50">
                                        {currentDemo.title}
                                    </h3>
                                    <p className="text-light-300 text-sm">
                                        {currentDemo.description}
                                    </p>
                                </div>
                            </div>

                            {/* User Prompt */}
                            <div className="mb-6">
                                <h4 className="text-lg font-semibold text-light-50 mb-3 flex items-center gap-2">
                                    <svg className="w-5 h-5 text-blue-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 8h10M7 12h4m1 8l-4-4H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-3l-4 4z" />
                                    </svg>
                                    User Prompt
                                </h4>
                                <div className="bg-dark-900/50 rounded-lg p-4 border border-dark-600">
                                    <p className="text-light-200 font-mono">
                                        "{currentDemo.prompt}"
                                    </p>
                                </div>
                            </div>

                            {/* Process Steps */}
                            <div>
                                <h4 className="text-lg font-semibold text-light-50 mb-3 flex items-center gap-2">
                                    <svg className="w-5 h-5 text-green-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v10a2 2 0 002 2h8a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2m-6 9l2 2 4-4" />
                                    </svg>
                                    Processing Steps
                                </h4>
                                <div className="space-y-3">
                                    {currentDemo.process.map((step, index) => (
                                        <div key={index} className="flex items-start gap-3">
                                            <div className="flex-shrink-0 w-6 h-6 bg-gradient-to-r from-blue-500 to-purple-500 rounded-full flex items-center justify-center text-white text-sm font-bold">
                                                {index + 1}
                                            </div>
                                            <p className="text-light-300 pt-1">{step}</p>
                                        </div>
                                    ))}
                                </div>
                            </div>
                        </div>

                        {/* Right Side - Visualization */}
                        <div className="bg-dark-900/50 rounded-xl p-6 border border-dark-600">
                            <div className="text-center mb-6">
                                <h4 className="text-lg font-semibold text-light-50 mb-2">
                                    Execution Visualization
                                </h4>
                                <p className="text-light-400 text-sm">
                                    Real-time monitoring and feedback
                                </p>
                            </div>

                            {/* Placeholder for visualization */}
                            <div className="bg-dark-800/50 rounded-lg p-8 border border-dark-600 min-h-64 flex items-center justify-center">
                                <div className="text-center">
                                    <div className={`w-16 h-16 bg-gradient-to-r ${currentDemo.color} rounded-full flex items-center justify-center text-white mx-auto mb-4 animate-pulse`}>
                                        {currentDemo.icon}
                                    </div>
                                    <p className="text-light-300 mb-2">Demo Visualization</p>
                                    <p className="text-light-500 text-sm">
                                        Interactive robot control interface
                                    </p>
                                </div>
                            </div>

                            {/* Status Indicators */}
                            <div className="mt-6 grid grid-cols-2 gap-4">
                                <div className="bg-dark-800/50 rounded-lg p-4 text-center">
                                    <div className="text-green-400 text-lg font-bold mb-1">✓ Active</div>
                                    <p className="text-light-400 text-sm">System Status</p>
                                </div>
                                <div className="bg-dark-800/50 rounded-lg p-4 text-center">
                                    <div className="text-blue-400 text-lg font-bold mb-1">Real-time</div>
                                    <p className="text-light-400 text-sm">Feedback Loop</p>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                {/* Performance Metrics */}
                <div className="mt-16 bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600">
                    <h3 className="text-2xl font-bold text-light-50 mb-6 text-center">
                        Performance Metrics
                    </h3>
                    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                        <div className="text-center">
                            <div className="text-3xl font-bold text-green-400 mb-2">100%</div>
                            <p className="text-light-300">Success Rate</p>
                            <p className="text-light-500 text-sm">Validated use cases</p>
                        </div>
                        <div className="text-center">
                            <div className="text-3xl font-bold text-blue-400 mb-2">&lt;2s</div>
                            <p className="text-light-300">Response Time</p>
                            <p className="text-light-500 text-sm">Prompt to action</p>
                        </div>
                        <div className="text-center">
                            <div className="text-3xl font-bold text-purple-400 mb-2">Real-time</div>
                            <p className="text-light-300">Monitoring</p>
                            <p className="text-light-500 text-sm">Safety & validation</p>
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
} 