export function TeamSection() {
    const teamMembers = [
        {
            name: "Daniel Siegel",
            role: "Hardware & Systems Engineering",
            website: "https://siegel.bio",
            email: "danieledisonsiegel@gmail.com",
            bio: "Hardware and systems engineer specializing in embedded robotics platforms. Focuses on low-level control systems, real-time firmware, and hardware-software integration for autonomous agents.",
            skills: ["Hardware Engineering", "Embedded Systems", "C/C++", "Real-time Systems", "Robotics Firmware"],
            avatar: "DS",
            color: "from-blue-500 to-cyan-500"
        },
        {
            name: "Thomason Zhou",
            role: "Robotics & Systems Engineering",
            website: "https://thzhou.com",
            email: "contact@thzhou.com",
            bio: "Mechatronics engineer advancing robotic mobility and HRI. Designs ROS2 control stacks, state‑estimation for exoskeletons, and MCP‑integrated perception pipelines.",
            skills: ["Robotics", "Control Systems", "ROS2", "PyTorch", "MCP Protocol"],
            avatar: "TZ",
            color: "from-purple-500 to-pink-500"
        }
    ];

    return (
        <section id="team" className="py-20 bg-yellow-50/30">
            <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="text-center mb-16">
                    <h2 className="text-3xl md:text-4xl font-bold text-light-50 mb-4">
                        Meet the Team
                    </h2>
                    <p className="text-xl text-light-300 max-w-3xl mx-auto">
                        RoboWeave is developed by a collaborative team of AI researchers and robotics engineers, 
                        bringing together expertise in multimodal systems and embodied AI.
                    </p>
                </div>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-8 mb-16">
                    {teamMembers.map((member, index) => (
                        <div key={index} className="bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600 hover:border-dark-500 transition-all duration-300">
                            <div className="flex items-center gap-4 mb-6">
                                <div className={`w-16 h-16 bg-gradient-to-r ${member.color} rounded-full flex items-center justify-center text-white font-bold text-xl`}>
                                    {member.avatar}
                                </div>
                                <div>
                                    <h3 className="text-2xl font-bold text-light-50">{member.name}</h3>
                                    <p className="text-light-300">{member.role}</p>
                                </div>
                            </div>

                            <p className="text-light-300 mb-6 leading-relaxed">
                                {member.bio}
                            </p>

                            <div className="mb-6">
                                <h4 className="text-lg font-semibold text-light-50 mb-3">Expertise</h4>
                                <div className="flex flex-wrap gap-2">
                                    {member.skills.map((skill, skillIndex) => (
                                        <span
                                            key={skillIndex}
                                            className={`px-3 py-1 bg-gradient-to-r ${member.color} text-white rounded-full text-sm font-medium`}
                                        >
                                            {skill}
                                        </span>
                                    ))}
                                </div>
                            </div>

                            <div className="flex gap-4">
                                <a
                                    href={member.website}
                                    target="_blank"
                                    rel="noopener noreferrer"
                                    className="flex items-center gap-2 text-light-300 hover:text-light-50 transition-colors"
                                >
                                    <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 01-9 9m9-9a9 9 0 00-9-9m9 9H3m9 9v-9m0-9v9" />
                                    </svg>
                                    Website
                                </a>
                                <a
                                    href={`mailto:${member.email}`}
                                    className="flex items-center gap-2 text-light-300 hover:text-light-50 transition-colors"
                                >
                                    <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 8l7.89 4.26a2 2 0 002.22 0L21 8M5 19h14a2 2 0 002-2V7a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
                                    </svg>
                                    Email
                                </a>
                            </div>
                        </div>
                    ))}
                </div>

                {/* Hackathon Information */}
                <div className="bg-dark-800/50 backdrop-blur-sm rounded-xl p-8 border border-dark-600">
                    <div className="text-center mb-8">
                        <h3 className="text-2xl font-bold text-light-50 mb-4">
                            Weave Hackathon 2025
                        </h3>
                        <p className="text-light-300 max-w-2xl mx-auto">
                            RoboWeave was developed as part of the Weave Hackathon 2025, showcasing the integration 
                            of advanced AI systems with robotic control platforms.
                        </p>
                    </div>

                    <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                        <div className="text-center">
                            <div className="w-12 h-12 bg-gradient-to-r from-blue-500 to-purple-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Timeline</h4>
                            <p className="text-light-300">48-hour hackathon</p>
                        </div>
                        
                        <div className="text-center">
                            <div className="w-12 h-12 bg-gradient-to-r from-green-500 to-teal-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Deliverables</h4>
                            <p className="text-light-300">Full pipeline + demo</p>
                        </div>
                        
                        <div className="text-center">
                            <div className="w-12 h-12 bg-gradient-to-r from-orange-500 to-red-500 rounded-full flex items-center justify-center mx-auto mb-4">
                                <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                                </svg>
                            </div>
                            <h4 className="text-lg font-semibold text-light-50 mb-2">Innovation</h4>
                            <p className="text-light-300">Multimodal robot control</p>
                        </div>
                    </div>
                </div>

                {/* Contact Information */}
                <div className="mt-16 text-center">
                    <h3 className="text-2xl font-bold text-light-50 mb-4">
                        Get in Touch
                    </h3>
                    <p className="text-light-300 mb-6">
                        Interested in collaborating or learning more about RoboWeave? 
                        We'd love to hear from you.
                    </p>
                    <div className="flex flex-col sm:flex-row gap-4 justify-center">
                        <a
                            href="mailto:danieledisonsiegel@gmail.com"
                            className="bg-gradient-to-r from-blue-500 to-purple-600 text-white px-6 py-3 rounded-lg font-medium hover:from-blue-600 hover:to-purple-700 transition-all duration-200 shadow-lg hover:shadow-xl"
                        >
                            Contact the Team
                        </a>
                        <a
                            href="https://github.com/danielsiegel/roboweave"
                            target="_blank"
                            rel="noopener noreferrer"
                            className="bg-gradient-to-r from-orange-500 to-amber-600 text-white px-6 py-3 rounded-lg font-medium hover:from-orange-600 hover:to-amber-700 transition-all duration-200 shadow-lg hover:shadow-xl"
                        >
                            View on GitHub
                        </a>
                    </div>
                </div>
            </div>
        </section>
    );
}
