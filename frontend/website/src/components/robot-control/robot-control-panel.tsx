import { useState } from "react";
import { useMCPRobot } from "~/hooks/use-mcp-robot";
import { cn } from "~@/utils/cn";

export function RobotControlPanel() {
    const {
        connected,
        connecting,
        robotState,
        connect,
        disconnect,
        moveForward,
        moveBackward,
        moveLeft,
        moveRight,
        rotateLeft,
        rotateRight,
        runInCircle,
        stopAndStay,
        doFlip,
        loading
    } = useMCPRobot();

    const [distance, setDistance] = useState(0.3);
    const [angle, setAngle] = useState(45);
    const [circleRadius, setCircleRadius] = useState(2.0);
    const [circleDuration, setCircleDuration] = useState(10.0);
    const [feedback, setFeedback] = useState<string>("");

    const handleCommand = async (command: () => Promise<any>, successMessage: string) => {
        try {
            const result = await command();
            if (result.success) {
                setFeedback(result.result || successMessage);
            } else {
                setFeedback(`Error: ${result.error}`);
            }
        } catch (error) {
            setFeedback(`Error: ${error instanceof Error ? error.message : 'Unknown error'}`);
        }
    };

    const getConnectionStatusColor = () => {
        if (connecting) return "text-amber-600";
        if (connected) return "text-green-600";
        return "text-red-600";
    };

    const getConnectionStatusText = () => {
        if (connecting) return "Connecting...";
        if (connected) return "Connected";
        return "Disconnected";
    };

    return (
        <div className="p-4 space-y-4">
            {/* Connection Status */}
            <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                <h3 className="text-lg font-semibold text-stone-800 mb-3">MCP Robot Control</h3>
                
                <div className="flex items-center justify-between mb-3">
                    <span className="text-sm text-stone-600">Status:</span>
                    <span className={cn("text-sm font-medium", getConnectionStatusColor())}>
                        {getConnectionStatusText()}
                    </span>
                </div>

                {!connected && !connecting && (
                    <button
                        onClick={connect}
                        className="w-full px-3 py-2 bg-orange-500 text-white rounded-lg hover:bg-orange-600 transition-colors text-sm"
                    >
                        Connect to Robot
                    </button>
                )}

                {connected && (
                    <button
                        onClick={disconnect}
                        className="w-full px-3 py-2 bg-red-500 text-white rounded-lg hover:bg-red-600 transition-colors text-sm"
                    >
                        Disconnect
                    </button>
                )}
            </div>

            {/* Robot State */}
            {connected && robotState && (
                <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                    <h4 className="text-md font-semibold text-stone-800 mb-3">Robot State</h4>
                    <div className="space-y-2 text-sm">
                        <div>
                            <span className="text-stone-600">Position:</span>
                            <div className="text-stone-800 font-mono">
                                X: {robotState.position.x.toFixed(3)}, 
                                Y: {robotState.position.y.toFixed(3)}, 
                                Z: {robotState.position.z.toFixed(3)}
                            </div>
                        </div>
                        <div>
                            <span className="text-stone-600">Mode:</span>
                            <span className="text-stone-800 ml-2">{robotState.current_mode}</span>
                        </div>
                    </div>
                </div>
            )}

            {/* Movement Controls */}
            {connected && (
                <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                    <h4 className="text-md font-semibold text-stone-800 mb-3">Movement Controls</h4>
                    
                    {/* Distance Setting */}
                    <div className="mb-3">
                        <label className="block text-sm text-stone-600 mb-1">Distance (m):</label>
                        <input
                            type="number"
                            value={distance}
                            onChange={(e) => setDistance(parseFloat(e.target.value))}
                            step="0.1"
                            min="0.1"
                            max="5.0"
                            className="w-full px-3 py-2 border border-stone-300 rounded-lg text-sm"
                        />
                    </div>

                    {/* Movement Buttons */}
                    <div className="grid grid-cols-3 gap-2 mb-3">
                        <div></div>
                        <button
                            onClick={() => handleCommand(() => moveForward(distance), "Moved forward")}
                            disabled={loading.move}
                            className="px-3 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ↑
                        </button>
                        <div></div>
                        
                        <button
                            onClick={() => handleCommand(() => moveLeft(distance), "Moved left")}
                            disabled={loading.move}
                            className="px-3 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ←
                        </button>
                        
                        <button
                            onClick={() => handleCommand(() => stopAndStay(), "Robot stopped")}
                            disabled={loading.complex}
                            className="px-3 py-2 bg-red-500 text-white rounded-lg hover:bg-red-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ⏹
                        </button>
                        
                        <button
                            onClick={() => handleCommand(() => moveRight(distance), "Moved right")}
                            disabled={loading.move}
                            className="px-3 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            →
                        </button>
                        
                        <div></div>
                        <button
                            onClick={() => handleCommand(() => moveBackward(distance), "Moved backward")}
                            disabled={loading.move}
                            className="px-3 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ↓
                        </button>
                        <div></div>
                    </div>
                </div>
            )}

            {/* Rotation Controls */}
            {connected && (
                <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                    <h4 className="text-md font-semibold text-stone-800 mb-3">Rotation Controls</h4>
                    
                    {/* Angle Setting */}
                    <div className="mb-3">
                        <label className="block text-sm text-stone-600 mb-1">Angle (degrees):</label>
                        <input
                            type="number"
                            value={angle}
                            onChange={(e) => setAngle(parseFloat(e.target.value))}
                            step="15"
                            min="15"
                            max="180"
                            className="w-full px-3 py-2 border border-stone-300 rounded-lg text-sm"
                        />
                    </div>

                    {/* Rotation Buttons */}
                    <div className="flex gap-2">
                        <button
                            onClick={() => handleCommand(() => rotateLeft(angle), "Rotated left")}
                            disabled={loading.rotate}
                            className="flex-1 px-3 py-2 bg-purple-500 text-white rounded-lg hover:bg-purple-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ↺ Left
                        </button>
                        
                        <button
                            onClick={() => handleCommand(() => rotateRight(angle), "Rotated right")}
                            disabled={loading.rotate}
                            className="flex-1 px-3 py-2 bg-purple-500 text-white rounded-lg hover:bg-purple-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            ↻ Right
                        </button>
                    </div>
                </div>
            )}

            {/* Advanced Controls */}
            {connected && (
                <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                    <h4 className="text-md font-semibold text-stone-800 mb-3">Advanced Controls</h4>
                    
                    {/* Circle Parameters */}
                    <div className="space-y-2 mb-3">
                        <div>
                            <label className="block text-sm text-stone-600 mb-1">Circle Radius (m):</label>
                            <input
                                type="number"
                                value={circleRadius}
                                onChange={(e) => setCircleRadius(parseFloat(e.target.value))}
                                step="0.5"
                                min="0.5"
                                max="5.0"
                                className="w-full px-3 py-2 border border-stone-300 rounded-lg text-sm"
                            />
                        </div>
                        <div>
                            <label className="block text-sm text-stone-600 mb-1">Duration (s):</label>
                            <input
                                type="number"
                                value={circleDuration}
                                onChange={(e) => setCircleDuration(parseFloat(e.target.value))}
                                step="1"
                                min="1"
                                max="60"
                                className="w-full px-3 py-2 border border-stone-300 rounded-lg text-sm"
                            />
                        </div>
                    </div>

                    {/* Advanced Action Buttons */}
                    <div className="space-y-2">
                        <button
                            onClick={() => handleCommand(() => runInCircle(circleRadius, circleDuration), "Running in circle")}
                            disabled={loading.complex}
                            className="w-full px-3 py-2 bg-green-500 text-white rounded-lg hover:bg-green-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            🔄 Run in Circle
                        </button>
                        
                        <button
                            onClick={() => handleCommand(() => doFlip(), "Performing flip")}
                            disabled={loading.complex}
                            className="w-full px-3 py-2 bg-orange-500 text-white rounded-lg hover:bg-orange-600 disabled:opacity-50 transition-colors text-sm"
                        >
                            🤸 Do Flip
                        </button>
                    </div>
                </div>
            )}

            {/* Feedback */}
            {feedback && (
                <div className="bg-white/70 backdrop-blur-sm rounded-xl p-4 border border-stone-200">
                    <h4 className="text-md font-semibold text-stone-800 mb-2">Feedback</h4>
                    <div className="text-sm text-stone-700 bg-stone-50 p-3 rounded-lg">
                        {feedback}
                    </div>
                    <button
                        onClick={() => setFeedback("")}
                        className="mt-2 px-3 py-1 text-xs bg-stone-500 text-white rounded hover:bg-stone-600 transition-colors"
                    >
                        Clear
                    </button>
                </div>
            )}
        </div>
    );
}
