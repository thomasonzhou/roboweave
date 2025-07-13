export function FooterSection() {
    return (
        <footer className="bg-stone-100 border-t border-stone-200">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12">
                <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
                    {/* Logo & Description */}
                    <div className="md:col-span-2">
                        <div className="flex items-center mb-4">
                            <div className="w-8 h-8 bg-gradient-to-br from-orange-500 to-amber-600 rounded-lg flex items-center justify-center">
                                <span className="text-white font-bold text-lg">R</span>
                            </div>
                            <span className="ml-2 text-xl font-bold text-stone-900">RoboWeave</span>
                        </div>
                        <p className="text-stone-600 leading-relaxed mb-4">
                            A multimodal robot control pipeline that maps high-level prompts into structured robotic commands 
                            using Google Gemini and real-time Weave integration.
                        </p>
                        <p className="text-stone-500 text-sm">
                            Developed for Weave Hackathon 2025
                        </p>
                    </div>

                    {/* Quick Links */}
                    <div>
                        <h3 className="text-lg font-semibold text-stone-900 mb-4">Quick Links</h3>
                        <ul className="space-y-2">
                            <li>
                                <a href="#overview" className="text-stone-600 hover:text-stone-900 transition-colors">
                                    Overview
                                </a>
                            </li>
                            <li>
                                <a href="#features" className="text-stone-600 hover:text-stone-900 transition-colors">
                                    Features
                                </a>
                            </li>
                            <li>
                                <a href="#architecture" className="text-stone-600 hover:text-stone-900 transition-colors">
                                    Architecture
                                </a>
                            </li>
                            <li>
                                <a href="#demo" className="text-stone-600 hover:text-stone-900 transition-colors">
                                    Demo
                                </a>
                            </li>
                            <li>
                                <a href="#team" className="text-stone-600 hover:text-stone-900 transition-colors">
                                    Team
                                </a>
                            </li>
                        </ul>
                    </div>

                    {/* Resources */}
                    <div>
                        <h3 className="text-lg font-semibold text-stone-900 mb-4">Resources</h3>
                        <ul className="space-y-2">
                            <li>
                                <a 
                                    href="https://github.com/thomasonzhou/roboweave" 
                                    target="_blank" 
                                    rel="noopener noreferrer"
                                    className="text-stone-600 hover:text-stone-900 transition-colors"
                                >
                                    GitHub Repository
                                </a>
                            </li>
                            <li>
                                <a 
                                    href="https://siegel.bio" 
                                    target="_blank" 
                                    rel="noopener noreferrer"
                                    className="text-stone-600 hover:text-stone-900 transition-colors"
                                >
                                    Daniel Siegel
                                </a>
                            </li>
                            <li>
                                <a 
                                    href="https://thzhou.com" 
                                    target="_blank" 
                                    rel="noopener noreferrer"
                                    className="text-stone-600 hover:text-stone-900 transition-colors"
                                >
                                    Thomason Zhou
                                </a>
                            </li>
                            <li>
                                <a 
                                    href="mailto:danieledisonsiegel@gmail.com" 
                                    className="text-stone-600 hover:text-stone-900 transition-colors"
                                >
                                    Contact
                                </a>
                            </li>
                        </ul>
                    </div>
                </div>

                {/* Divider */}
                <div className="border-t border-stone-300 mt-8 pt-8">
                    <div className="flex flex-col md:flex-row justify-between items-center">
                        <div className="text-stone-500 text-sm">
                            © 2025 RoboWeave. Developed for Weave Hackathon 2025.
                        </div>
                        <div className="flex items-center gap-4 mt-4 md:mt-0">
                            <a
                                href="https://github.com/thomasonzhou/roboweave"
                                target="_blank"
                                rel="noopener noreferrer"
                                className="text-stone-500 hover:text-stone-900 transition-colors"
                                aria-label="GitHub"
                            >
                                <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 24 24">
                                    <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
                                </svg>
                            </a>
                            <div className="text-stone-500 text-sm">
                                Built with React, TypeScript, and Weave
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </footer>
    );
} 