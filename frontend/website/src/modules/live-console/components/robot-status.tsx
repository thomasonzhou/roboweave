/**
 * Robot Status Component
 * Shows current robot state and connection status
 */

import { useState, useEffect } from 'react';
import type { UseRobotControlState } from '~/hooks/use-robot-control';

interface RobotStatusProps {
  robotState: UseRobotControlState;
  onRefreshState: () => void;
}

export function RobotStatus({ robotState, onRefreshState }: RobotStatusProps) {
  const [autoRefresh, setAutoRefresh] = useState(true);

  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      if (robotState.isConnected && !robotState.isLoading) {
        onRefreshState();
      }
    }, 2000);

    return () => clearInterval(interval);
  }, [autoRefresh, robotState.isConnected, robotState.isLoading, onRefreshState]);

  const formatPosition = (pos: number) => pos?.toFixed(2) || '0.00';
  const formatQuaternion = (q: number) => q?.toFixed(3) || '0.000';

  return (
    <div className="h-full flex flex-col">
      {/* Header */}
      <div className="p-4 border-b border-gray-200">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-semibold text-gray-700">Robot Status</h3>
          
          <div className="flex items-center space-x-2">
            <label className="flex items-center space-x-1 text-xs">
              <input
                type="checkbox"
                checked={autoRefresh}
                onChange={(e) => setAutoRefresh(e.target.checked)}
                className="rounded text-blue-600 focus:ring-blue-500"
              />
              <span className="text-gray-600">Auto-refresh</span>
            </label>
            
            <button
              onClick={onRefreshState}
              disabled={robotState.isLoading}
              className="text-xs text-blue-600 hover:text-blue-800 disabled:opacity-50"
            >
              {robotState.isLoading ? 'Refreshing...' : 'Refresh'}
            </button>
          </div>
        </div>
      </div>

      {/* Connection Status */}
      <div className="p-4 border-b border-gray-200">
        <div className="flex items-center justify-between">
          <span className="text-sm font-medium text-gray-700">Connection</span>
          <div className="flex items-center space-x-2">
            <div className={`w-2 h-2 rounded-full ${
              robotState.isConnected ? 'bg-green-500' : 'bg-red-500'
            }`}></div>
            <span className={`text-sm ${
              robotState.isConnected ? 'text-green-600' : 'text-red-600'
            }`}>
              {robotState.isConnected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
        </div>

        {robotState.error && (
          <div className="mt-2 p-2 bg-red-50 border border-red-200 rounded text-xs text-red-700">
            Error: {robotState.error}
          </div>
        )}
      </div>

      {/* Robot State */}
      <div className="flex-1 overflow-auto">
        {!robotState.isConnected ? (
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <div className="text-4xl mb-2">🔌</div>
            <p className="text-center">Robot not connected</p>
            <p className="text-sm text-center mt-1">
              Check your robot control server
            </p>
          </div>
        ) : robotState.isLoading ? (
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <div className="text-4xl mb-2 animate-spin">⏳</div>
            <p>Loading robot state...</p>
          </div>
        ) : robotState.robotState ? (
          <div className="p-4 space-y-4">
            {/* Position */}
            <div className="bg-blue-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-blue-800 mb-3">Position</h4>
              <div className="grid grid-cols-3 gap-4">
                <div className="text-center">
                  <div className="text-lg font-mono font-bold text-blue-600">
                    {formatPosition(robotState.robotState.position.x)}
                  </div>
                  <div className="text-xs text-blue-800">X (m)</div>
                </div>
                <div className="text-center">
                  <div className="text-lg font-mono font-bold text-blue-600">
                    {formatPosition(robotState.robotState.position.y)}
                  </div>
                  <div className="text-xs text-blue-800">Y (m)</div>
                </div>
                <div className="text-center">
                  <div className="text-lg font-mono font-bold text-blue-600">
                    {formatPosition(robotState.robotState.position.z)}
                  </div>
                  <div className="text-xs text-blue-800">Z (m)</div>
                </div>
              </div>
            </div>

            {/* Orientation */}
            <div className="bg-green-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-green-800 mb-3">Orientation (Quaternion)</h4>
              <div className="grid grid-cols-2 gap-4">
                <div className="text-center">
                  <div className="text-sm font-mono font-bold text-green-600">
                    {formatQuaternion(robotState.robotState.orientation.w)}
                  </div>
                  <div className="text-xs text-green-800">W</div>
                </div>
                <div className="text-center">
                  <div className="text-sm font-mono font-bold text-green-600">
                    {formatQuaternion(robotState.robotState.orientation.x)}
                  </div>
                  <div className="text-xs text-green-800">X</div>
                </div>
                <div className="text-center">
                  <div className="text-sm font-mono font-bold text-green-600">
                    {formatQuaternion(robotState.robotState.orientation.y)}
                  </div>
                  <div className="text-xs text-green-800">Y</div>
                </div>
                <div className="text-center">
                  <div className="text-sm font-mono font-bold text-green-600">
                    {formatQuaternion(robotState.robotState.orientation.z)}
                  </div>
                  <div className="text-xs text-green-800">Z</div>
                </div>
              </div>
            </div>

            {/* Mode */}
            <div className="bg-purple-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-purple-800 mb-3">Current Mode</h4>
              <div className="text-center">
                <div className="text-xl font-bold text-purple-600">
                  {robotState.robotState.current_mode || 'Unknown'}
                </div>
                <div className="text-xs text-purple-800 mt-1">Operating Mode</div>
              </div>
            </div>

            {/* Last Response */}
            {robotState.lastResponse && (
              <div className="bg-gray-50 rounded-lg p-4">
                <h4 className="text-sm font-semibold text-gray-700 mb-3">Last Command Response</h4>
                <div className="text-xs space-y-2">
                  <div className="flex justify-between">
                    <span className="text-gray-600">Success:</span>
                    <span className={robotState.lastResponse.success ? 'text-green-600' : 'text-red-600'}>
                      {robotState.lastResponse.success ? 'Yes' : 'No'}
                    </span>
                  </div>
                  <div className="flex justify-between">
                    <span className="text-gray-600">Timestamp:</span>
                    <span className="text-gray-800">
                      {new Date(robotState.lastResponse.timestamp).toLocaleTimeString()}
                    </span>
                  </div>
                  {robotState.lastResponse.data && (
                    <div className="mt-2">
                      <div className="text-gray-600 mb-1">Response:</div>
                      <div className="bg-white border rounded p-2 text-gray-800 max-h-20 overflow-auto">
                        {typeof robotState.lastResponse.data === 'string' 
                          ? robotState.lastResponse.data 
                          : JSON.stringify(robotState.lastResponse.data, null, 2)
                        }
                      </div>
                    </div>
                  )}
                </div>
              </div>
            )}
          </div>
        ) : (
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <div className="text-4xl mb-2">❓</div>
            <p className="text-center">No robot state available</p>
            <p className="text-sm text-center mt-1">
              Try refreshing or check connection
            </p>
          </div>
        )}
      </div>

      {/* Quick Actions */}
      {robotState.isConnected && (
        <div className="border-t border-gray-200 p-4 bg-gray-50">
          <h4 className="text-xs font-semibold text-gray-600 mb-2">Quick Actions</h4>
          <div className="grid grid-cols-2 gap-2">
            <button className="text-xs bg-blue-600 text-white py-2 px-3 rounded hover:bg-blue-700">
              Get State
            </button>
            <button className="text-xs bg-red-600 text-white py-2 px-3 rounded hover:bg-red-700">
              Emergency Stop
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
