import { useState } from "react";
import { Link, useLocation } from "@tanstack/react-router";

export function NavigationComponent() {
    const [isMenuOpen, setIsMenuOpen] = useState(false);
    const location = useLocation();

    const navItems = [
        { name: "Home", href: "/" },
        { name: "Demo", href: "/demo" },
        { name: "Live Console", href: "/live-console" },
    ];

    return (
        <nav className="sticky top-0 z-50 bg-white/80 backdrop-blur-sm border-b border-stone-200">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="flex justify-between items-center h-16">
                    {/* Logo */}
                    <div className="flex items-center">
                        <a href="/" className="flex-shrink-0 flex items-center hover:opacity-80 transition-opacity duration-200">
                            <div className="w-8 h-8 bg-gradient-to-br from-orange-500 to-amber-600 rounded-lg flex items-center justify-center">
                                <span className="text-white font-bold text-lg">R</span>
                            </div>
                            <span className="ml-2 text-xl font-bold text-stone-900">RoboWeave</span>
                        </a>
                    </div>

                    {/* Desktop Navigation */}
                    <div className="hidden md:block">
                        <div className="ml-10 flex items-baseline space-x-8">
                            {navItems.map((item) => (
                                <Link
                                    key={item.name}
                                    to={item.href}
                                    className="text-stone-600 hover:text-stone-900 px-3 py-2 text-sm font-medium transition-colors duration-200 hover:bg-stone-100 rounded-md [&.active]:text-orange-600 [&.active]:bg-orange-50"
                                    activeProps={{ className: "active" }}
                                >
                                    {item.name}
                                </Link>
                            ))}
                        </div>
                    </div>

                    {/* CTA Button */}
                    <div className="hidden md:block">
                        <a
                            href="https://github.com/thomasonzhou/roboweave"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="bg-gradient-to-r from-orange-500 to-amber-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:from-orange-600 hover:to-amber-700 transition-all duration-200 shadow-lg hover:shadow-xl"
                        >
                            View on GitHub
                        </a>
                    </div>

                    {/* Mobile menu button */}
                    <div className="md:hidden">
                        <button
                            onClick={() => setIsMenuOpen(!isMenuOpen)}
                            className="text-stone-600 hover:text-stone-900 focus:outline-none focus:ring-2 focus:ring-inset focus:ring-orange-500"
                        >
                            <svg className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                {isMenuOpen ? (
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                                ) : (
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
                                )}
                            </svg>
                        </button>
                    </div>
                </div>
            </div>

            {/* Mobile Navigation */}
            {isMenuOpen && (
                <div className="md:hidden">
                    <div className="px-2 pt-2 pb-3 space-y-1 sm:px-3 bg-white/95 border-t border-stone-200">
                        {navItems.map((item) => (
                            <Link
                                key={item.name}
                                to={item.href}
                                className="text-stone-600 hover:text-stone-900 block px-3 py-2 text-base font-medium transition-colors duration-200 hover:bg-stone-100 rounded-md [&.active]:text-orange-600 [&.active]:bg-orange-50"
                                activeProps={{ className: "active" }}
                                onClick={() => setIsMenuOpen(false)}
                            >
                                {item.name}
                            </Link>
                        ))}
                        <a
                            href="https://github.com/thomasonzhou/roboweave"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="bg-gradient-to-r from-orange-500 to-amber-600 text-white block px-3 py-2 text-base font-medium rounded-md mt-4 text-center"
                        >
                            View on GitHub
                        </a>
                    </div>
                </div>
            )}
        </nav>
    );
} 