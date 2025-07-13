/**
 * Robot Control Panel Component
 * 
 * This component provides an interactive control panel for the robot,
 * allowing users to send movement commands and view robot state.
 */

import { useState } from 'react';
import { useRobotControl } from '~/hooks/use-robot-control';
import { cn } from '~@/utils/cn';

export function RobotControlPanel() {
    const [robotState, robotActions] = useRobotControl();
    const [distance, setDistance] = useState(0.3);
    const [angle, setAngle] = useState(45);
    const [circleRadius, setCircleRadius] = useState(2.0);
    const [circleDuration, setCircleDuration] = useState(10.0);

    const formatPosition = (pos: { x: number; y: number; z: number }) => 
        `(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;

    const formatTimestamp = (timestamp: number) => 
        new Date(timestamp * 1000).toLocaleTimeString();

    return (
        <div className="p-4 space-y-6">
            {/* Connection Status */}
            <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold text-stone-800">Robot Control</h3>
                <div className="flex items-center gap-2">
                    <div className={cn(
                        "w-3 h-3 rounded-full",
                        robotState.isConnected ? "bg-green-500" : "bg-red-500"
                    )} />
                    <span className={cn(
                        "text-sm font-medium",
                        robotState.isConnected ? "text-green-700" : "text-red-700"
                    )}>
                        {robotState.isConnected ? "Connected" : "Disconnected"}
                    </span>
                </div>
            </div>

            {/* Connection Controls */}
            {!robotState.isConnected && (
                <button
                    onClick={robotActions.connect}
                    disabled={robotState.isLoading}
                    className="w-full px-4 py-2 bg-orange-500 text-white rounded-lg hover:bg-orange-600 disabled:opacity-50 disabled:cursor-not-allowed"
                >
                    {robotState.isLoading ? "Connecting..." : "Connect to Robot"}
                </button>
            )}

            {/* Error Display */}
            {robotState.error && (
                <div className="p-3 bg-red-50 border border-red-200 rounded-lg">
                    <p className="text-sm text-red-600">{robotState.error}</p>
                </div>
            )}

            {/* Robot State Display */}
            {robotState.isConnected && robotState.robotState && (
                <div className="bg-stone-50 rounded-lg p-4 space-y-3">
                    <h4 className="font-medium text-stone-800">Robot Status</h4>
                    <div className="grid grid-cols-1 gap-2 text-sm">
                        <div>
                            <span className="font-medium text-stone-600">Position:</span>
                            <span className="ml-2 font-mono text-stone-800">
                                {formatPosition(robotState.robotState.position)}
                            </span>
                        </div>
                        <div>
                            <span className="font-medium text-stone-600">Mode:</span>
                            <span className="ml-2 text-stone-800">
                                {robotState.robotState.current_mode}
                            </span>
                        </div>
                        <div>
                            <span className="font-medium text-stone-600">Updated:</span>
                            <span className="ml-2 text-stone-800">
                                {formatTimestamp(robotState.robotState.timestamp)}
                            </span>
                        </div>
                    </div>
                </div>
            )}

            {/* Movement Controls */}
            {robotState.isConnected && (
                <div className="space-y-4">
                    <h4 className="font-medium text-stone-800">Movement Controls</h4>
                    
                    {/* Distance Settings */}
                    <div className="space-y-2">
                        <label className="block text-sm font-medium text-stone-600">
                            Distance (meters): {distance}
                        </label>
                        <input
                            type="range"
                            min="0.1"
                            max="2.0"
                            step="0.1"
                            value={distance}
                            onChange={(e) => setDistance(parseFloat(e.target.value))}
                            className="w-full"
                        />
                    </div>

                    {/* Directional Movement */}
                    <div className="grid grid-cols-3 gap-2">
                        <div></div>
                        <button
                            onClick={() => robotActions.moveForward(distance)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
                        >
                            ↑ Forward
                        </button>
                        <div></div>
                        
                        <button
                            onClick={() => robotActions.moveLeft(distance)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
                        >
                            ← Left
                        </button>
                        <button
                            onClick={robotActions.stopAndStay}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-red-500 text-white rounded hover:bg-red-600 disabled:opacity-50 text-sm"
                        >
                            ⏹ Stop
                        </button>
                        <button
                            onClick={() => robotActions.moveRight(distance)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
                        >
                            Right →
                        </button>
                        
                        <div></div>
                        <button
                            onClick={() => robotActions.moveBackward(distance)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 disabled:opacity-50 text-sm"
                        >
                            ↓ Backward
                        </button>
                        <div></div>
                    </div>

                    {/* Rotation Controls */}
                    <div className="space-y-2">
                        <label className="block text-sm font-medium text-stone-600">
                            Rotation Angle (degrees): {angle}
                        </label>
                        <input
                            type="range"
                            min="15"
                            max="180"
                            step="15"
                            value={angle}
                            onChange={(e) => setAngle(parseInt(e.target.value))}
                            className="w-full"
                        />
                    </div>

                    <div className="grid grid-cols-2 gap-2">
                        <button
                            onClick={() => robotActions.rotateLeft(angle)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-purple-500 text-white rounded hover:bg-purple-600 disabled:opacity-50 text-sm"
                        >
                            ↶ Rotate Left
                        </button>
                        <button
                            onClick={() => robotActions.rotateRight(angle)}
                            disabled={robotState.isLoading}
                            className="px-3 py-2 bg-purple-500 text-white rounded hover:bg-purple-600 disabled:opacity-50 text-sm"
                        >
                            ↷ Rotate Right
                        </button>
                    </div>

                    {/* Advanced Actions */}
                    <div className="space-y-2">
                        <h5 className="font-medium text-stone-700">Advanced Actions</h5>
                        
                        {/* Circle Movement */}
                        <div className="space-y-2">
                            <div className="grid grid-cols-2 gap-2">
                                <div>
                                    <label className="block text-xs text-stone-600">Radius (m)</label>
                                    <input
                                        type="number"
                                        min="0.5"
                                        max="5"
                                        step="0.5"
                                        value={circleRadius}
                                        onChange={(e) => setCircleRadius(parseFloat(e.target.value))}
                                        className="w-full px-2 py-1 border rounded text-sm"
                                    />
                                </div>
                                <div>
                                    <label className="block text-xs text-stone-600">Duration (s)</label>
                                    <input
                                        type="number"
                                        min="5"
                                        max="30"
                                        step="5"
                                        value={circleDuration}
                                        onChange={(e) => setCircleDuration(parseFloat(e.target.value))}
                                        className="w-full px-2 py-1 border rounded text-sm"
                                    />
                                </div>
                            </div>
                            <button
                                onClick={() => robotActions.runInCircle(circleRadius, circleDuration)}
                                disabled={robotState.isLoading}
                                className="w-full px-3 py-2 bg-green-500 text-white rounded hover:bg-green-600 disabled:opacity-50 text-sm"
                            >
                                🔄 Run in Circle
                            </button>
                        </div>

                        {/* Flip Action */}
                        <button
                            onClick={robotActions.doFlip}
                            disabled={robotState.isLoading}
                            className="w-full px-3 py-2 bg-orange-500 text-white rounded hover:bg-orange-600 disabled:opacity-50 text-sm"
                        >
                            🤸 Perform Flip
                        </button>
                    </div>
                </div>
            )}

            {/* Last Response */}
            {robotState.lastResponse && (
                <div className={cn(
                    "p-3 rounded-lg border",
                    robotState.lastResponse.success 
                        ? "bg-green-50 border-green-200" 
                        : "bg-red-50 border-red-200"
                )}>
                    <h5 className={cn(
                        "font-medium text-sm",
                        robotState.lastResponse.success ? "text-green-800" : "text-red-800"
                    )}>
                        {robotState.lastResponse.success ? "✅ Success" : "❌ Error"}
                    </h5>
                    <p className={cn(
                        "text-sm mt-1",
                        robotState.lastResponse.success ? "text-green-700" : "text-red-700"
                    )}>
                        {robotState.lastResponse.result || robotState.lastResponse.error}
                    </p>
                </div>
            )}

            {/* Loading Indicator */}
            {robotState.isLoading && (
                <div className="flex items-center justify-center p-4">
                    <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-orange-500"></div>
                    <span className="ml-2 text-stone-600">Processing...</span>
                </div>
            )}
        </div>
    );
}
