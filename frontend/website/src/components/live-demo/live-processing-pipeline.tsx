import { useState, useCallback, useRef, useEffect } from 'react';

interface MotionPrimitive {
    t: string; // type: 'p' for primary, 's' for speculative
    s: [number, number]; // start
    c1: [number, number]; // control1
    c2: [number, number]; // control2
    e: [number, number]; // end
    d: number; // duration
    v: number; // velocity
    r: string; // reasoning
}

interface PipelineStep {
    id: string;
    name: string;
    status: 'idle' | 'processing' | 'complete' | 'error';
    data?: any;
    reasoning?: string;
    timing?: number;
    weaveTrace?: any;
}

interface Agent {
    id: string;
    name: string;
    color: string;
    position: { x: number; y: number };
    steps: PipelineStep[];
    isActive: boolean;
}

export function LiveProcessingPipeline() {
    const [agents, setAgents] = useState<Agent[]>([{
        id: 'agent-1',
        name: 'Primary Agent',
        color: 'from-blue-500 to-cyan-500',
        position: { x: 100, y: 100 },
        steps: [
            { id: 'input', name: 'Prompt Input', status: 'idle' },
            { id: 'gemini', name: 'Gemini Processing', status: 'idle' },
            { id: 'weave', name: 'Weave Analysis', status: 'idle' },
            { id: 'planning', name: 'Motion Planning', status: 'idle' },
            { id: 'execution', name: 'Execution', status: 'idle' },
            { id: 'monitoring', name: 'Monitoring', status: 'idle' }
        ],
        isActive: false
    }]);
    
    const [inputText, setInputText] = useState('Walk forward without hitting anything');
    const [uploadedImage, setUploadedImage] = useState<File | null>(null);
    const [isProcessing, setIsProcessing] = useState(false);
    const [weaveData, setWeaveData] = useState<any>({});
    
    const fileInputRef = useRef<HTMLInputElement>(null);
    const wsRef = useRef<WebSocket | null>(null);

    // Initialize WebSocket connection to local Python bridge
    useEffect(() => {
        const connectWebSocket = () => {
            try {
                wsRef.current = new WebSocket('ws://localhost:8765');
                
                wsRef.current.onopen = () => {
                    console.log('[DEBUG] WebSocket connected to Python backend');
                };
                
                wsRef.current.onmessage = (event) => {
                    const data = JSON.parse(event.data);
                    handlePythonResponse(data);
                };
                
                wsRef.current.onerror = (error) => {
                    console.error('[DEBUG] WebSocket error:', error);
                };
                
                wsRef.current.onclose = () => {
                    console.log('[DEBUG] WebSocket disconnected, attempting reconnect...');
                    setTimeout(connectWebSocket, 2000);
                };
            } catch (error) {
                console.error('[DEBUG] Failed to connect WebSocket:', error);
            }
        };
        
        connectWebSocket();
        
        return () => {
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, []);

    const handlePythonResponse = (data: any) => {
        if (data.type !== 'pipeline_update') return;
        
        const { step, agentId, status, result, weaveTrace, timing } = data;
        
        setAgents(prev => prev.map(agent => 
            agent.id === agentId ? {
                ...agent,
                steps: agent.steps.map(s => 
                    s.id === step ? {
                        ...s,
                        status,
                        data: result,
                        weaveTrace,
                        timing,
                        reasoning: result?.reasoning || result?.strategy
                    } : s
                )
            } : agent
        ));
        
        // Update Weave data
        if (weaveTrace) {
                    setWeaveData((prev: Record<string, any>) => ({
            ...prev,
            [agentId]: {
                ...prev[agentId],
                [step]: weaveTrace
            }
        }));
        }
    };

    const processInput = useCallback(async (agentId: string) => {
        if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
            console.error('[DEBUG] WebSocket not connected');
            return;
        }

        setIsProcessing(true);
        
        // Reset agent steps
        setAgents(prev => prev.map(agent => 
            agent.id === agentId ? {
                ...agent,
                isActive: true,
                steps: agent.steps.map(s => ({ ...s, status: 'idle' }))
            } : agent
        ));

        // Prepare data for Python backend
        const requestData = {
            agentId,
            text: inputText,
            image: uploadedImage ? await fileToBase64(uploadedImage) : null,
            timestamp: Date.now()
        };

        // Send to Python backend for processing
        wsRef.current.send(JSON.stringify({
            type: 'process_multimodal',
            data: requestData
        }));

        // Animate through pipeline steps
        animatePipelineSteps(agentId);
    }, [inputText, uploadedImage]);

    const animatePipelineSteps = async (agentId: string) => {
        const steps = ['input', 'gemini', 'weave', 'planning', 'execution', 'monitoring'];
        
        for (let i = 0; i < steps.length; i++) {
            const stepId = steps[i];
            
            // Start processing step
            setAgents(prev => prev.map(agent => 
                agent.id === agentId ? {
                    ...agent,
                    steps: agent.steps.map(s => 
                        s.id === stepId ? { ...s, status: 'processing' } : s
                    )
                } : agent
            ));
            
            // Wait for processing (varies by step)
            const processingTime = getStepProcessingTime(stepId);
            await new Promise(resolve => setTimeout(resolve, processingTime));
            
            // Mark as complete (Python backend will override with real data)
            setAgents(prev => prev.map(agent => 
                agent.id === agentId ? {
                    ...agent,
                    steps: agent.steps.map(s => 
                        s.id === stepId ? { ...s, status: 'complete' } : s
                    )
                } : agent
            ));
        }
        
        setIsProcessing(false);
    };

    const getStepProcessingTime = (stepId: string): number => {
        const timings = {
            input: 500,
            gemini: 2000,
            weave: 800,
            planning: 1200,
            execution: 1500,
            monitoring: 600
        };
        return timings[stepId as keyof typeof timings] || 1000;
    };

    const spawnNewAgent = useCallback(() => {
        const newAgent: Agent = {
            id: `agent-${agents.length + 1}`,
            name: `Agent ${agents.length + 1}`,
            color: getRandomAgentColor(),
            position: { 
                x: 100 + (agents.length * 300), 
                y: 100 + (agents.length * 50) 
            },
            steps: [
                { id: 'input', name: 'Prompt Input', status: 'idle' },
                { id: 'gemini', name: 'Gemini Processing', status: 'idle' },
                { id: 'weave', name: 'Weave Analysis', status: 'idle' },
                { id: 'planning', name: 'Motion Planning', status: 'idle' },
                { id: 'execution', name: 'Execution', status: 'idle' },
                { id: 'monitoring', name: 'Monitoring', status: 'idle' }
            ],
            isActive: false
        };
        
        setAgents(prev => [...prev, newAgent]);
    }, [agents]);

    const getRandomAgentColor = (): string => {
        const colors = [
            'from-blue-500 to-cyan-500',
            'from-purple-500 to-pink-500',
            'from-green-500 to-teal-500',
            'from-orange-500 to-red-500',
            'from-indigo-500 to-purple-500',
            'from-yellow-500 to-orange-500'
        ];
        return colors[Math.floor(Math.random() * colors.length)];
    };

    const fileToBase64 = (file: File): Promise<string> => {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.readAsDataURL(file);
            reader.onload = () => {
                const base64 = (reader.result as string).split(',')[1];
                resolve(base64);
            };
            reader.onerror = error => reject(error);
        });
    };

    const handleImageUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.[0];
        if (file) {
            setUploadedImage(file);
        }
    };

    const getStatusColor = (status: string): string => {
        switch (status) {
            case 'idle': return 'bg-gray-200 text-gray-600';
            case 'processing': return 'bg-orange-200 text-orange-800 animate-pulse';
            case 'complete': return 'bg-green-200 text-green-800';
            case 'error': return 'bg-red-200 text-red-800';
            default: return 'bg-gray-200 text-gray-600';
        }
    };

    return (
        <div className="w-full bg-amber-50/30 backdrop-blur-sm rounded-xl p-6 border border-stone-200 mb-8">
            <div className="flex items-center justify-between mb-6">
                <h3 className="text-2xl font-bold text-stone-800">Live Processing Pipeline</h3>
                <button
                    onClick={spawnNewAgent}
                    className="px-4 py-2 bg-gradient-to-r from-orange-500 to-amber-600 text-white rounded-lg font-medium hover:from-orange-600 hover:to-amber-700 transition-all duration-200 shadow-lg"
                >
                    Spawn New Agent
                </button>
            </div>

            {/* Input Controls */}
            <div className="bg-white/60 rounded-lg p-4 mb-6 border border-stone-200">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div>
                        <label className="block text-stone-700 font-medium mb-2">
                            Text Prompt
                        </label>
                        <input
                            type="text"
                            value={inputText}
                            onChange={(e) => setInputText(e.target.value)}
                            className="w-full px-3 py-2 border border-stone-300 rounded-lg focus:ring-2 focus:ring-orange-500 focus:border-orange-500"
                            placeholder="Enter robot command..."
                        />
                    </div>
                    
                    <div>
                        <label className="block text-stone-700 font-medium mb-2">
                            Image Upload
                        </label>
                        <div className="flex gap-2">
                            <input
                                ref={fileInputRef}
                                type="file"
                                accept="image/*"
                                onChange={handleImageUpload}
                                className="hidden"
                            />
                            <button
                                onClick={() => fileInputRef.current?.click()}
                                className="px-4 py-2 border border-stone-300 rounded-lg hover:bg-stone-50 transition-colors"
                            >
                                {uploadedImage ? uploadedImage.name : 'Choose Image'}
                            </button>
                        </div>
                    </div>
                </div>
            </div>

            {/* Pipeline Visualization */}
            <div className="relative overflow-x-auto">
                <div className="min-w-max">
                    {agents.map((agent) => (
                        <div key={agent.id} className="mb-8">
                            <div className="flex items-center gap-4 mb-4">
                                <div className={`px-3 py-1 bg-gradient-to-r ${agent.color} text-white rounded-full text-sm font-medium`}>
                                    {agent.name}
                                </div>
                                <button
                                    onClick={() => processInput(agent.id)}
                                    disabled={isProcessing}
                                    className="px-4 py-2 bg-green-500 text-white rounded-lg hover:bg-green-600 disabled:opacity-50 disabled:cursor-not-allowed transition-all"
                                >
                                    {isProcessing ? 'Processing...' : 'Execute Pipeline'}
                                </button>
                            </div>
                            
                            <div className="flex items-center gap-4 overflow-x-auto pb-4">
                                {agent.steps.map((step, index) => (
                                    <div key={step.id} className="flex items-center">
                                        {/* Step Node */}
                                        <div className="relative">
                                            <div className={`w-32 h-24 rounded-xl border-2 p-3 transition-all duration-300 ${getStatusColor(step.status)}`}>
                                                <div className="text-xs font-semibold mb-1">{step.name}</div>
                                                {step.timing && (
                                                    <div className="text-xs opacity-75">{step.timing}ms</div>
                                                )}
                                                {step.reasoning && (
                                                    <div className="text-xs mt-1 truncate" title={step.reasoning}>
                                                        {step.reasoning.slice(0, 20)}...
                                                    </div>
                                                )}
                                            </div>
                                            
                                            {/* Weave Trace Indicator */}
                                            {weaveData[agent.id]?.[step.id] && (
                                                <div className="absolute -top-2 -right-2 w-4 h-4 bg-purple-500 rounded-full text-white text-xs flex items-center justify-center">
                                                    W
                                                </div>
                                            )}
                                            
                                            {/* Motion Primitives Visualization */}
                                            {step.data?.motion_primitives && (
                                                <div className="absolute -bottom-2 left-0 right-0">
                                                    <div className="bg-blue-500 text-white text-xs px-2 py-1 rounded text-center">
                                                        {step.data.motion_primitives.length} primitives
                                                    </div>
                                                </div>
                                            )}
                                        </div>
                                        
                                        {/* Connection Arrow */}
                                        {index < agent.steps.length - 1 && (
                                            <div className="flex items-center">
                                                <div className={`w-8 h-0.5 transition-colors duration-300 ${
                                                    step.status === 'complete' ? 'bg-green-500' : 
                                                    step.status === 'processing' ? 'bg-orange-500 animate-pulse' : 
                                                    'bg-gray-300'
                                                }`}></div>
                                                <div className={`w-0 h-0 border-l-4 border-r-0 border-t-2 border-b-2 border-transparent transition-colors duration-300 ${
                                                    step.status === 'complete' ? 'border-l-green-500' : 
                                                    step.status === 'processing' ? 'border-l-orange-500' : 
                                                    'border-l-gray-300'
                                                }`}></div>
                                            </div>
                                        )}
                                    </div>
                                ))}
                            </div>
                            
                            {/* Detailed Results */}
                            {agent.isActive && agent.steps.some(s => s.data) && (
                                <div className="mt-4 bg-white/40 rounded-lg p-4 border border-stone-200">
                                    <h4 className="font-semibold text-stone-800 mb-2">Pipeline Results</h4>
                                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                                        {agent.steps.filter(s => s.data).map(step => (
                                            <div key={step.id} className="bg-white/60 rounded p-3">
                                                <div className="font-medium text-stone-700 mb-2">{step.name}</div>
                                                <pre className="text-xs text-stone-600 overflow-auto max-h-20">
                                                    {JSON.stringify(step.data, null, 2)}
                                                </pre>
                                            </div>
                                        ))}
                                    </div>
                                </div>
                            )}
                        </div>
                    ))}
                </div>
            </div>
            
            {/* Weave Integration Status */}
            <div className="mt-6 bg-purple-50 rounded-lg p-4 border border-purple-200">
                <div className="flex items-center gap-2 mb-2">
                    <div className="w-3 h-3 bg-purple-500 rounded-full animate-pulse"></div>
                    <span className="font-medium text-purple-800">Weave Observability Active</span>
                </div>
                <div className="text-sm text-purple-600">
                    Tracking {Object.keys(weaveData).length} agents with real-time parameter extraction and reasoning chains
                </div>
            </div>
        </div>
    );
} 