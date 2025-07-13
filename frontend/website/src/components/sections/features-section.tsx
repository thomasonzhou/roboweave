export function FeaturesSection() {
    const features = [
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.746 0 3.332.477 4.5 1.253v13C19.832 18.477 18.246 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                </svg>
            ),
            title: "Multimodal Prompting",
            description: "Supports text, images, and videos as input modalities. Express robot intentions through natural language, visual demonstrations, or video sequences.",
            color: "from-blue-500 to-cyan-500"
        },
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                </svg>
            ),
            title: "LLM-Driven Reasoning",
            description: "Leverages Google Gemini's integrated text-vision model to interpret human intent in both explicit and latent forms with advanced multimodal understanding.",
            color: "from-purple-500 to-pink-500"
        },
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
            ),
            title: "Real-time Feedback Loop",
            description: "Weave integration enables visualization of both prompts and resulting robotic behaviors in a continuous interaction loop with live monitoring.",
            color: "from-green-500 to-teal-500"
        },
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 9l3 3-3 3m5 0h3M5 20h14a2 2 0 002-2V6a2 2 0 00-2-2H5a2 2 0 00-2 2v14a2 2 0 002 2z" />
                </svg>
            ),
            title: "MCP Control Interface",
            description: "Encapsulates low-level robot actuation via MCP-compatible command streaming with fault-tolerant communication channels.",
            color: "from-orange-500 to-red-500"
        },
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4.318 6.318a4.5 4.5 0 000 6.364L12 20.364l7.682-7.682a4.5 4.5 0 00-6.364-6.364L12 7.636l-1.318-1.318a4.5 4.5 0 00-6.364 0z" />
                </svg>
            ),
            title: "Safety & Validation",
            description: "Execution monitoring for safety, validity, and alignment with prompt expectations. Built-in feasibility checking based on robot capabilities.",
            color: "from-indigo-500 to-purple-500"
        },
        {
            icon: (
                <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 7h8m0 0v8m0-8l-8 8-4-4-6 6" />
                </svg>
            ),
            title: "Scalable Architecture",
            description: "Designed to generalize across multiple robotic tasks with modular components that can be adapted for different embodied agent platforms.",
            color: "from-yellow-500 to-orange-500"
        }
    ];

    return (
        <section id="features" className="py-20 bg-amber-50/50">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="text-center mb-16">
                    <h2 className="text-3xl md:text-4xl font-bold text-light-50 mb-4">
                        Key Features
                    </h2>
                    <p className="text-xl text-light-300 max-w-3xl mx-auto">
                        RoboWeave combines cutting-edge AI with robust robotics infrastructure to enable 
                        intuitive control of embodied agents through multimodal prompting.
                    </p>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8">
                    {features.map((feature, index) => (
                        <div
                            key={index}
                            className="bg-dark-800/50 backdrop-blur-sm rounded-xl p-6 border border-dark-600 hover:border-dark-500 transition-all duration-300 hover:transform hover:scale-105"
                        >
                            <div className={`w-12 h-12 bg-gradient-to-r ${feature.color} rounded-lg flex items-center justify-center text-white mb-4`}>
                                {feature.icon}
                            </div>
                            <h3 className="text-xl font-semibold text-light-50 mb-3">
                                {feature.title}
                            </h3>
                            <p className="text-light-300 leading-relaxed">
                                {feature.description}
                            </p>
                        </div>
                    ))}
                </div>

                {/* Technical Specifications */}
                <div className="mt-16 bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600">
                    <h3 className="text-2xl font-bold text-light-50 mb-6 text-center">
                        Technical Specifications
                    </h3>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
                        <div className="text-center">
                            <div className="text-3xl font-bold text-blue-400 mb-2">3</div>
                            <p className="text-light-300">Input Modalities</p>
                            <p className="text-light-500 text-sm">Text, Image, Video</p>
                        </div>
                        <div className="text-center">
                            <div className="text-3xl font-bold text-purple-400 mb-2">1M+</div>
                            <p className="text-light-300">Token Context</p>
                            <p className="text-light-500 text-sm">Gemini 1.5 Pro</p>
                        </div>
                        <div className="text-center">
                            <div className="text-3xl font-bold text-green-400 mb-2">Real-time</div>
                            <p className="text-light-300">Feedback Loop</p>
                            <p className="text-light-500 text-sm">Live Monitoring</p>
                        </div>
                        <div className="text-center">
                            <div className="text-3xl font-bold text-orange-400 mb-2">MCP</div>
                            <p className="text-light-300">Protocol</p>
                            <p className="text-light-500 text-sm">Model Context Protocol</p>
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
} 