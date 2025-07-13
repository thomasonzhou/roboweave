export function HeroSection() {
    return (
        <section id="overview" className="relative py-20 lg:py-32 overflow-hidden">
            {/* Dogs background */}
            <div 
                className="absolute inset-0 bg-cover bg-center bg-no-repeat opacity-60"
                style={{ backgroundImage: 'url(/dogs-background.png)' }}
            ></div>
            
            {/* Background overlay */}
            <div className="absolute inset-0 bg-gradient-to-br from-amber-50/95 via-orange-50/90 to-stone-100/95"></div>
            
            {/* Subtle animated background elements */}
            <div className="absolute inset-0 overflow-hidden">
                <div className="absolute top-1/4 left-1/4 w-64 h-64 bg-orange-200/10 rounded-full blur-3xl animate-pulse"></div>
                <div className="absolute bottom-1/4 right-1/4 w-96 h-96 bg-amber-200/10 rounded-full blur-3xl animate-pulse delay-1000"></div>
            </div>

            <div className="relative max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="text-center">
                    {/* Badge */}
                    <div className="inline-flex items-center px-4 py-2 bg-orange-100/60 border border-orange-200/60 rounded-full text-orange-800 text-sm font-medium mb-8 backdrop-blur-sm">
                        <span className="inline-block w-2 h-2 bg-orange-500 rounded-full mr-2 animate-pulse"></span>
                        Weave Hackathon 2025
                    </div>

                    {/* Main title */}
                    <h1 className="text-4xl md:text-6xl lg:text-7xl font-bold text-stone-900 mb-6" style={{ fontFamily: 'Inter, system-ui, -apple-system, sans-serif', fontWeight: 700, letterSpacing: '-0.025em' }}>
                        <span 
                            className="inline-block text-stone-900 typewriter-text"
                        >
                            RoboWeave
                        </span>
                    </h1>

                    {/* Subtitle */}
                    <p className="text-xl md:text-2xl text-stone-700 mb-4 max-w-4xl mx-auto font-medium" style={{ fontFamily: 'Inter, system-ui, -apple-system, sans-serif', letterSpacing: '-0.015em' }}>
                        Prompt-Based Multimodal Control of Embodied Agents
                    </p>

                    {/* Description */}
                    <p className="text-lg text-stone-600 mb-12 max-w-3xl mx-auto leading-relaxed" style={{ fontFamily: 'Inter, system-ui, -apple-system, sans-serif', lineHeight: '1.6' }}>
                        A multimodal robot control pipeline that maps high-level prompts—expressed via natural language, 
                        images, or video—into structured robotic commands using Google Gemini and real-time Weave integration.
                    </p>

                    {/* CTA Buttons */}
                    <div className="flex flex-col sm:flex-row gap-4 justify-center items-center">
                        <a
                            href="/demo"
                            className="bg-gradient-to-r from-orange-500 to-amber-600 text-white px-8 py-3 rounded-lg text-lg font-medium hover:from-orange-600 hover:to-amber-700 transition-all duration-200 shadow-lg hover:shadow-xl transform hover:scale-105"
                        >
                            View Demo
                        </a>
                        <a
                            href="https://github.com/thomasonzhou/roboweave"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="border border-stone-300 text-stone-700 px-8 py-3 rounded-lg text-lg font-medium hover:bg-stone-100 hover:text-stone-900 transition-all duration-200"
                        >
                            GitHub Repository
                        </a>
                    </div>

                    {/* Authors */}
                    <div className="mt-12 text-stone-500">
                        <p className="text-sm">By</p>
                        <div className="flex justify-center items-center gap-6 mt-2">
                            <a
                                href="https://siegel.bio"
                                target="_blank"
                                rel="noopener noreferrer"
                                className="hover:text-stone-700 transition-colors font-medium"
                            >
                                Daniel Siegel
                            </a>
                            <span className="text-stone-400">•</span>
                            <a
                                href="https://thzhou.com"
                                target="_blank"
                                rel="noopener noreferrer"
                                className="hover:text-stone-700 transition-colors font-medium"
                            >
                                Thomason Zhou
                            </a>
                        </div>
                    </div>
                </div>

                {/* Visual representation */}
                <div className="mt-16 relative">
                    <div className="max-w-4xl mx-auto">
                        <div className="bg-white/70 backdrop-blur-sm rounded-2xl p-8 border border-stone-200 shadow-lg">
                            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                                {/* Input */}
                                <div className="text-center">
                                    <div className="w-16 h-16 bg-orange-100 rounded-full flex items-center justify-center mx-auto mb-4 border border-orange-200">
                                        <svg className="w-8 h-8 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M7 4V2a1 1 0 011-1h8a1 1 0 011 1v2h4a1 1 0 011 1v2a1 1 0 01-1 1h-1v12a2 2 0 01-2 2H6a2 2 0 01-2-2V8H3a1 1 0 01-1-1V5a1 1 0 011-1h4z" />
                                        </svg>
                                    </div>
                                    <h3 className="text-lg font-semibold text-stone-800 mb-2">Multimodal Input</h3>
                                    <p className="text-stone-600 text-sm">Text, images, video prompts</p>
                                </div>

                                {/* Processing */}
                                <div className="text-center">
                                    <div className="w-16 h-16 bg-amber-100 rounded-full flex items-center justify-center mx-auto mb-4 border border-amber-200">
                                        <svg className="w-8 h-8 text-amber-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                                        </svg>
                                    </div>
                                    <h3 className="text-lg font-semibold text-stone-800 mb-2">LLM Processing</h3>
                                    <p className="text-stone-600 text-sm">Gemini multimodal reasoning</p>
                                </div>

                                {/* Output */}
                                <div className="text-center">
                                    <div className="w-16 h-16 bg-stone-100 rounded-full flex items-center justify-center mx-auto mb-4 border border-stone-200">
                                        <svg className="w-8 h-8 text-stone-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                                        </svg>
                                    </div>
                                    <h3 className="text-lg font-semibold text-stone-800 mb-2">Robot Control</h3>
                                    <p className="text-stone-600 text-sm">Structured commands via MCP</p>
                                </div>
                            </div>
                        </div>
                    </div>
                </div>

                {/* Built With Section */}
                <div className="mt-12 relative">
                    <div className="max-w-4xl mx-auto text-center">
                        <p className="text-stone-500 text-sm mb-6 font-medium">Powered by industry-leading technologies</p>
                        <div className="flex flex-wrap justify-center items-center gap-8 md:gap-12">
                            {/* Google Gemini */}
                            <div className="flex items-center gap-3 opacity-80 hover:opacity-100 transition-opacity duration-200">
                                <img 
                                    src="/gemini-logo.png" 
                                    alt="Google Gemini" 
                                    className="h-8 w-auto"
                                />
                            </div>

                            {/* MuJoCo */}
                            <div className="flex items-center gap-3 opacity-80 hover:opacity-100 transition-opacity duration-200">
                                <img 
                                    src="/mujoco-logo.png" 
                                    alt="MuJoCo" 
                                    className="h-8 w-auto"
                                />
                                <span className="text-stone-700 font-bold tracking-wide" style={{ fontFamily: 'Inter, system-ui, -apple-system, sans-serif', letterSpacing: '0.05em' }}>
                                    MuJoCo
                                </span>
                            </div>

                            {/* Weights & Biases Weave */}
                            <div className="flex items-center gap-3 opacity-80 hover:opacity-100 transition-opacity duration-200">
                                <img 
                                    src="/wandb-logo.png" 
                                    alt="Weights & Biases" 
                                    className="h-8 w-auto"
                                />
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </section>
    );
} 