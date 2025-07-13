import { createFileRoute } from "@tanstack/react-router";

import { HeroSection } from "~/components/sections/hero-section";
import { FeaturesSection } from "~/components/sections/features-section";
import { ArchitectureSection } from "~/components/sections/architecture-section";
import { DemoSection } from "~/components/sections/demo-section";
import { TeamSection } from "~/components/sections/team-section";
import { NavigationComponent } from "~/components/navigation/navigation-component";
import { FooterSection } from "~/components/sections/footer-section";

export const Route = createFileRoute("/")({
    component: RoboWeavePage,
});

function RoboWeavePage() {
    return (
        <div className="min-h-screen bg-gradient-to-br from-amber-50 via-stone-50 to-orange-50 text-stone-900">
            <NavigationComponent />
            
            <main className="relative">
                <HeroSection />
                <FeaturesSection />
                <ArchitectureSection />
                <DemoSection />
                <TeamSection />
            </main>
            
            <FooterSection />
        </div>
    );
}
