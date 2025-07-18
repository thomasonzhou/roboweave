import React, { useState, useRef, useEffect } from 'react';
import SidebarPanelWrapper from "~/modules/sidebar/components/sidebar-panel-wrapper";
import { useLiveProcessing } from "~/pages/demo";

// CSS animations
const fadeInKeyframes = `
@keyframes fadeIn {
  from { opacity: 0; transform: translateY(10px); }
  to { opacity: 1; transform: translateY(0); }
}
.animate-fade-in {
  animation: fadeIn 0.5s ease-out;
}
@keyframes dataUpdate {
  0% { background-color: rgba(34, 197, 94, 0.3); }
  100% { background-color: transparent; }
}
.animate-data-update {
  animation: dataUpdate 1s ease-out;
}
`;

// Inject CSS
if (typeof document !== 'undefined') {
  const style = document.createElement('style');
  style.textContent = fadeInKeyframes;
  document.head.appendChild(style);
}

interface PipelineStep {
    id: string;
    name: string;
    status: 'pending' | 'processing' | 'completed' | 'error';
    description: string;
    icon: string;
    reasoning?: string;
    parameters?: Record<string, any>;
    timing?: number;
    motionPrimitives?: number;
}

interface ProcessingAgent {
    id: string;
    color: string;
    position: { x: number; y: number };
    currentStep: number;
    status: 'active' | 'completed' | 'error';
    startTime: number;
}

const PIPELINE_STEPS: PipelineStep[] = [
    {
        id: 'prompt-input',
        name: 'Prompt Input',
        status: 'pending',
        description: 'Processing text and image inputs',
        icon: '📝'
    },
    {
        id: 'llm-processing',
        name: 'LLM Processing',
        status: 'pending',
        description: 'Gemini 1.5 Pro reasoning and high-level planning',
        icon: '🧠'
    },
    {
        id: 'motion-planning',
        name: 'Motion Planning',
        status: 'pending',
        description: 'Generating MuJoCo-compatible motion sequences',
        icon: '📐'
    },
    {
        id: 'weave-monitoring',
        name: 'Weave Monitoring',
        status: 'pending',
        description: 'Real-time observability and tracing',
        icon: '📊'
    }
];

export default function LiveProcessingPanel() {
    // Get shared state from context
    const { 
        agents, 
        steps, 
        weaveData, 
        isProcessing, 
        connectionStatus,
        setAgents, 
        setSteps, 
        setWeaveData, 
        setIsProcessing, 
        setConnectionStatus 
    } = useLiveProcessing();
    
    const [textInput, setTextInput] = useState('');
    const [imageFile, setImageFile] = useState<File | null>(null);
    
    const fileInputRef = useRef<HTMLInputElement>(null);

    // Backend connection check
    useEffect(() => {
        const checkBackendConnection = async () => {
            setConnectionStatus('connecting');
            
            try {
                // Check Gemini service health
                const response = await fetch('http://localhost:8000/health', {
                    method: 'GET',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    signal: AbortSignal.timeout(5000)
                });
                
                if (response.ok) {
                    const healthData = await response.json();
                    console.log('Backend health check:', healthData);
                    setConnectionStatus('connected');
                } else {
                    console.error('Backend health check failed:', response.status);
                    setConnectionStatus('disconnected');
                }
            } catch (error) {
                console.error('Backend connection failed:', error);
                setConnectionStatus('disconnected');
                
                // Retry connection after 5 seconds
                setTimeout(checkBackendConnection, 5000);
            }
        };
        
        checkBackendConnection();
        
        // Check connection every 30 seconds
        const interval = setInterval(checkBackendConnection, 30000);
        
        return () => {
            clearInterval(interval);
        };
    }, []);

    const handleWebSocketMessage = (data: any) => {
        console.log('Received WebSocket message:', data);
        
        switch (data.type) {
            case 'step_update':
                const updatedSteps = steps.map(step => {
                    if (step.id === data.step_id) {
                        const updatedStep = { 
                            ...step, 
                            status: data.status, 
                            reasoning: data.reasoning, 
                            parameters: data.parameters || {},
                            timing: data.timing
                        };
                        
                        // Trigger animation by temporarily adding class
                        setTimeout(() => {
                            const stepElement = document.querySelector(`[data-step-id="${data.step_id}"]`);
                            if (stepElement) {
                                stepElement.classList.add('animate-data-update');
                                setTimeout(() => stepElement.classList.remove('animate-data-update'), 1000);
                            }
                        }, 100);
                        
                        return updatedStep;
                    }
                    return step;
                });
                setSteps(updatedSteps);
                break;
            case 'agent_update':
                const existingAgent = agents.find(agent => agent.id === data.agent_id);
                if (existingAgent) {
                    // Update existing agent
                    const updatedAgents = agents.map(agent => 
                        agent.id === data.agent_id 
                            ? { ...agent, currentStep: data.current_step, status: data.status }
                            : agent
                    );
                    setAgents(updatedAgents);
                } else {
                    // Agent doesn't exist, create it
                    const newAgent: ProcessingAgent = {
                        id: data.agent_id,
                        color: `hsl(${Math.random() * 360}, 70%, 60%)`,
                        position: { x: Math.random() * 200, y: Math.random() * 100 },
                        currentStep: data.current_step,
                        status: data.status,
                        startTime: Date.now()
                    };
                    setAgents([...agents, newAgent]);
                }
                break;
            case 'weave_data':
                setWeaveData((prev: any) => ({
                    ...prev,
                    ...data.data,
                    lastUpdate: Date.now()
                }));
                // Animate weave data update
                setTimeout(() => {
                    const weaveElement = document.querySelector('[data-weave-display]');
                    if (weaveElement) {
                        weaveElement.classList.add('animate-fade-in');
                        setTimeout(() => weaveElement.classList.remove('animate-fade-in'), 500);
                    }
                }, 100);
                break;
            case 'processing_complete':
                setIsProcessing(false);
                const completedSteps = steps.map(step => ({ ...step, status: 'completed' }));
                setSteps(completedSteps);
                // Update agent to completed
                const completedAgents = agents.map(agent => 
                    agent.id === data.agent_id 
                        ? { ...agent, status: 'completed' }
                        : agent
                );
                setAgents(completedAgents);
                // Trigger completion animation
                setTimeout(() => {
                    const completionElement = document.querySelector('[data-pipeline-complete]');
                    if (completionElement) {
                        completionElement.classList.add('animate-bounce');
                        setTimeout(() => completionElement.classList.remove('animate-bounce'), 1000);
                    }
                }, 100);
                break;
            case 'processing_error':
                setIsProcessing(false);
                const errorSteps = steps.map(step => 
                    step.status === 'processing' ? { ...step, status: 'error' } : step
                );
                setSteps(errorSteps);
                // Update agent to error
                const errorAgents = agents.map(agent => 
                    agent.id === data.agent_id 
                        ? { ...agent, status: 'error' }
                        : agent
                );
                setAgents(errorAgents);
                break;
            case 'connection_established':
                console.log('WebSocket connection established:', data.client_id);
                break;
            case 'processing_started':
                console.log('Processing started for agent:', data.agent_id);
                break;
            case 'pong':
                console.log('Received pong - WebSocket connection is working');
                break;
            default:
                console.log('Unknown message type:', data.type);
        }
    };

    const handleImageUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            setImageFile(file);
        }
    };

    const startProcessing = async () => {
        if (connectionStatus !== 'connected') {
            console.log('Backend not connected, status:', connectionStatus);
            return;
        }
        
        if (!textInput.trim() && !imageFile) {
            console.log('No input provided');
            return;
        }
        
        setIsProcessing(true);
        setSteps(steps.map(step => ({ ...step, status: 'pending' as const })));
        
        // Create new agent
        const newAgent: ProcessingAgent = {
            id: `agent-${Date.now()}`,
            color: `hsl(${Math.random() * 360}, 70%, 60%)`,
            position: { x: Math.random() * 200, y: Math.random() * 100 },
            currentStep: 0,
            status: 'active',
            startTime: Date.now()
        };
        
        console.log('Creating new agent:', newAgent);
        setAgents([...agents, newAgent]);
        
        try {
            // Simulate step updates
            await simulateProcessingSteps(newAgent.id);
            
            // Process the command via HTTP
            const response = await fetch('http://localhost:8000/process-voice-command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    transcript: textInput.trim(),
                    confidence: 0.9,
                    session_id: `demo-${Date.now()}`
                })
            });
            
            if (response.ok) {
                const result = await response.json();
                console.log('Processing result:', result);
                
                // Update Weave data with the response
                setWeaveData({
                    observability_data: {
                        transcript: textInput.trim(),
                        commands: result.commands,
                        explanation: result.explanation,
                        processing_time: result.processing_time,
                        session_id: result.session_id
                    },
                    motion_analysis: {
                        primitives_generated: result.commands?.length || 0,
                        execution_status: 'completed'
                    }
                });
                
                // Mark agent as completed
                setAgents(prev => prev.map(agent => 
                    agent.id === newAgent.id 
                        ? { ...agent, status: 'completed', currentStep: 4 }
                        : agent
                ));
            } else {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
        } catch (error) {
            console.error('Error processing request:', error);
            
            // Mark agent and steps as error
            setAgents(prev => prev.map(agent => 
                agent.id === newAgent.id 
                    ? { ...agent, status: 'error' }
                    : agent
            ));
            setSteps(prev => prev.map(step => ({ ...step, status: 'error' as const })));
        } finally {
            setIsProcessing(false);
        }
    };

    // Simulate processing steps with delays
    const simulateProcessingSteps = async (agentId: string) => {
        const stepIds = ['prompt-input', 'llm-processing', 'motion-planning', 'weave-monitoring'];
        
        for (let i = 0; i < stepIds.length; i++) {
            const stepId = stepIds[i];
            
            // Mark current step as processing
            setSteps(prev => prev.map(step => 
                step.id === stepId 
                    ? { ...step, status: 'processing' as const }
                    : step
            ));
            
            // Update agent progress
            setAgents(prev => prev.map(agent => 
                agent.id === agentId 
                    ? { ...agent, currentStep: i + 1 }
                    : agent
            ));
            
            // Wait for step completion
            await new Promise(resolve => setTimeout(resolve, 1000 + Math.random() * 2000));
            
            // Mark step as completed
            setSteps(prev => prev.map(step => 
                step.id === stepId 
                    ? { ...step, status: 'completed' as const, timing: Date.now() }
                    : step
            ));
        }
    };

    const fileToBase64 = (file: File): Promise<string> => {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.readAsDataURL(file);
            reader.onload = () => {
                const base64 = reader.result as string;
                resolve(base64.split(',')[1]); // Remove data:image/...;base64, prefix
            };
            reader.onerror = reject;
        });
    };

    const spawnAgent = () => {
        startProcessing();
    };

    const resetAll = () => {
        setTextInput('');
        setImageFile(null);
        setIsProcessing(false);
        setSteps(PIPELINE_STEPS);
        setAgents([]);
        setWeaveData(null);
        if (fileInputRef.current) {
            fileInputRef.current.value = '';
        }
    };

    return (
        <SidebarPanelWrapper>
            <div className="p-3">
                {/* Connection Status */}
                <div className="flex items-center space-x-2 mb-3">
                    <div className={`w-2 h-2 rounded-full ${
                        connectionStatus === 'connected' ? 'bg-green-500' : 
                        connectionStatus === 'connecting' ? 'bg-yellow-500' : 'bg-red-500'
                    }`}></div>
                    <span className="text-xs text-light-50/60">
                        {connectionStatus === 'connected' ? 'Connected' : 
                         connectionStatus === 'connecting' ? 'Connecting...' : 'Disconnected'}
                    </span>
                </div>

                {/* Text Input */}
                <div className="mb-3">
                    <label className="block text-xs font-medium text-light-50/70 mb-2">
                        Text Prompt
                    </label>
                    <textarea
                        value={textInput}
                        onChange={(e) => setTextInput(e.target.value)}
                        placeholder="Enter your robot control prompt..."
                        className="w-full h-16 px-2 py-2 bg-dark-400 border border-dark-300 rounded-lg text-xs text-light-50 placeholder-light-50/40 resize-none focus:outline-none focus:ring-2 focus:ring-orange-500"
                        disabled={isProcessing}
                    />
                </div>

                {/* Image Upload */}
                <div className="mb-3">
                    <label className="block text-xs font-medium text-light-50/70 mb-2">
                        Image Input
                    </label>
                    <input
                        ref={fileInputRef}
                        type="file"
                        accept="image/*"
                        onChange={handleImageUpload}
                        className="hidden"
                        disabled={isProcessing}
                    />
                    <button
                        onClick={() => fileInputRef.current?.click()}
                        disabled={isProcessing}
                        className="w-full px-3 py-2 bg-dark-400 border border-dark-300 rounded-lg text-xs text-light-50/60 hover:bg-dark-300 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
                    >
                        {imageFile ? imageFile.name : 'Upload Image'}
                    </button>
                </div>

                {/* Control Buttons */}
                <div className="flex space-x-2 mb-3">
                    <button
                        onClick={startProcessing}
                        disabled={isProcessing || (!textInput.trim() && !imageFile) || connectionStatus !== 'connected'}
                        className="flex-1 px-3 py-2 bg-orange-600 hover:bg-orange-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white text-xs rounded-lg transition-colors"
                    >
                        {isProcessing ? 'Processing...' : 'Start Processing'}
                    </button>
                    <button
                        onClick={spawnAgent}
                        disabled={isProcessing || (!textInput.trim() && !imageFile) || connectionStatus !== 'connected'}
                        className="px-3 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 disabled:cursor-not-allowed text-white text-xs rounded-lg transition-colors"
                    >
                        Spawn Agent
                    </button>
                </div>

                <button
                    onClick={resetAll}
                    className="w-full px-3 py-2 bg-red-600 hover:bg-red-700 text-white text-xs rounded-lg transition-colors mb-3"
                >
                    Reset All
                </button>

                {/* Status Summary */}
                <div>
                    <h3 className="text-xs font-medium text-light-50/70 mb-2">Quick Status</h3>
                    <div className="space-y-2">
                        <div className="flex justify-between items-center p-2 bg-dark-400 rounded border border-dark-300">
                            <span className="text-xs text-light-50/60">Pipeline:</span>
                            <span className="text-xs text-light-50">
                                {steps.filter(s => s.status === 'completed').length}/{steps.length} Complete
                            </span>
                        </div>
                        <div className="flex justify-between items-center p-2 bg-dark-400 rounded border border-dark-300">
                            <span className="text-xs text-light-50/60">Agents:</span>
                            <span className="text-xs text-light-50">{agents.length} Active</span>
                        </div>
                        {weaveData && (
                            <div className="flex justify-between items-center p-2 bg-dark-400 rounded border border-dark-300">
                                <span className="text-xs text-light-50/60">Weave:</span>
                                <span className="text-xs text-green-400">📊 Connected</span>
                            </div>
                        )}
                    </div>
                    
                    <div className="mt-3 p-2 bg-blue-900/20 border border-blue-500/30 rounded-lg">
                        <div className="text-xs text-blue-400 font-medium mb-1">💡 Tip</div>
                        <div className="text-xs text-blue-300">
                            Check the right panel for detailed live metrics, pipeline status, Weave traces, and motion analysis.
                        </div>
                    </div>
                </div>
            </div>
        </SidebarPanelWrapper>
    );
} 