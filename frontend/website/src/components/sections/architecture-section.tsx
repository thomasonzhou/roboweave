export function ArchitectureSection() {
    const pipelineComponents = [
        {
            id: 1,
            title: "Weave Frontend Integration",
            description: "Handles prompt input via typed text, image upload, or recorded video. Streams live actuator states and sensor readings from a robot.",
            features: [
                "Multimodal prompt collection",
                "Live feedback visualization",
                "Demonstration playback",
                "Real-time monitoring"
            ],
            color: "from-blue-500 to-cyan-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                </svg>
            )
        },
        {
            id: 2,
            title: "Prompt Interpretation",
            description: "Prompts are passed to Gemini, which performs multimodal reasoning and outputs structured descriptions of intended robot behavior.",
            features: [
                "Natural language understanding",
                "Visual scene analysis",
                "Temporal sequence parsing",
                "Intent extraction"
            ],
            color: "from-purple-500 to-pink-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                </svg>
            )
        },
        {
            id: 3,
            title: "LLM Parsing & Planning",
            description: "Extracts semantic intent and translates it into discrete motion primitives. Maintains state awareness and accounts for feasibility.",
            features: [
                "Semantic intent extraction",
                "Motion primitive generation",
                "State awareness",
                "Feasibility checking"
            ],
            color: "from-green-500 to-teal-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10" />
                </svg>
            )
        },
        {
            id: 4,
            title: "Backend Execution",
            description: "Commands are translated into MCP format and sent via fault-tolerant channels to the robot controller with safety monitoring.",
            features: [
                "MCP command translation",
                "Fault-tolerant communication",
                "Safety monitoring",
                "Execution validation"
            ],
            color: "from-orange-500 to-red-500",
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
            )
        }
    ];

    return (
        <section id="architecture" className="py-20 bg-stone-50/60">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="text-center mb-16">
                    <h2 className="text-3xl md:text-4xl font-bold text-light-50 mb-4">
                        System Architecture
                    </h2>
                    <p className="text-xl text-light-300 max-w-3xl mx-auto">
                        RoboWeave follows a modular pipeline architecture that seamlessly integrates 
                        multimodal inputs with robotic control through intelligent reasoning layers.
                    </p>
                </div>

                {/* Pipeline Flow */}
                <div className="mb-16">
                    <div className="grid grid-cols-1 lg:grid-cols-4 gap-8">
                        {pipelineComponents.map((component, index) => (
                            <div key={component.id} className="relative">
                                <div className="bg-dark-800/50 backdrop-blur-sm rounded-xl p-6 border border-dark-600 hover:border-dark-500 transition-all duration-300 h-full">
                                    <div className={`w-12 h-12 bg-gradient-to-r ${component.color} rounded-lg flex items-center justify-center text-white mb-4`}>
                                        {component.icon}
                                    </div>
                                    <h3 className="text-xl font-semibold text-light-50 mb-3">
                                        {component.title}
                                    </h3>
                                    <p className="text-light-300 mb-4 leading-relaxed">
                                        {component.description}
                                    </p>
                                    <ul className="space-y-2">
                                        {component.features.map((feature, featureIndex) => (
                                            <li key={featureIndex} className="flex items-center text-light-400 text-sm">
                                                <span className="w-2 h-2 bg-gradient-to-r from-blue-400 to-purple-400 rounded-full mr-2"></span>
                                                {feature}
                                            </li>
                                        ))}
                                    </ul>
                                </div>
                                
                                {/* Arrow connector */}
                                {index < pipelineComponents.length - 1 && (
                                    <div className="hidden lg:block absolute top-1/2 -right-4 transform -translate-y-1/2 z-10">
                                        <div className="w-8 h-8 bg-dark-800 rounded-full border-2 border-dark-600 flex items-center justify-center">
                                            <svg className="w-4 h-4 text-light-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
                                            </svg>
                                        </div>
                                    </div>
                                )}
                            </div>
                        ))}
                    </div>
                </div>

                {/* Technical Stack */}
                <div className="bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600">
                    <h3 className="text-2xl font-bold text-light-50 mb-8 text-center">
                        Technical Stack
                    </h3>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
                        <div className="text-center">
                            <div className="w-16 h-16 bg-gradient-to-r from-blue-500 to-cyan-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-8 h-8 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Frontend</h4>
                            <p className="text-light-300 text-sm">React, TypeScript, Weave UI</p>
                        </div>
                        
                        <div className="text-center">
                            <div className="w-16 h-16 bg-gradient-to-r from-purple-500 to-pink-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-8 h-8 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">AI/ML</h4>
                            <p className="text-light-300 text-sm">Google Gemini 1.5 Pro</p>
                        </div>
                        
                        <div className="text-center">
                            <div className="w-16 h-16 bg-gradient-to-r from-green-500 to-teal-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-8 h-8 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 12h14M5 12a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v4a2 2 0 01-2 2M5 12a2 2 0 00-2 2v4a2 2 0 002 2h14a2 2 0 002-2v-4a2 2 0 00-2-2m-2-4h.01M17 16h.01" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Backend</h4>
                            <p className="text-light-300 text-sm">Python 3.10+, MCP Protocol</p>
                        </div>
                        
                        <div className="text-center">
                            <div className="w-16 h-16 bg-gradient-to-r from-orange-500 to-red-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-8 h-8 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547M8 4h8l-1 1v5.172a2 2 0 00.586 1.414l5 5c1.26 1.26.367 3.414-1.415 3.414H4.828c-1.782 0-2.674-2.154-1.414-3.414l5-5A2 2 0 009 10.172V5L8 4z" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Robotics</h4>
                            <p className="text-light-300 text-sm">MuJoCo, Embodied Agents</p>
                        </div>
                    </div>
                </div>

                {/* Project Structure */}
                <div className="mt-16 bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600">
                    <h3 className="text-2xl font-bold text-light-50 mb-6 text-center">
                        Project Structure
                    </h3>
                    <div className="bg-dark-900/50 rounded-lg p-6 font-mono text-sm">
                        <pre className="text-light-300 leading-relaxed">
{`roboweave/
├── frontend/            # Weave-based interface and prompt collection
├── backend/
│   ├── llm/             # Gemini interaction and parsing
│   ├── planning/        # Prompt-to-motion translation
│   └── mcp/             # MCP integration layer
├── scripts/             # QA, telemetry, test cases
├── docs/                # Architecture notes and presentation assets
└── README.md`}
                        </pre>
                    </div>
                </div>
            </div>
        </section>
    );
} 