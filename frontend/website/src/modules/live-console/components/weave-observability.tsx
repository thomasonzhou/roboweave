/**
 * Weave Observability Component
 * Shows W&B Weave tracking data and session analytics
 */

import { useState } from 'react';
import type { LiveConsoleStats } from '../hooks/use-live-console';
import type { GeminiResponse } from '../services/gemini-live';

interface WeaveObservabilityProps {
  sessionId: string;
  stats: LiveConsoleStats;
  recentResponses: GeminiResponse[];
  onResetSession: () => void;
}

type MetricView = 'overview' | 'performance' | 'commands' | 'errors';

export function WeaveObservability({ 
  sessionId, 
  stats, 
  recentResponses, 
  onResetSession 
}: WeaveObservabilityProps) {
  const [activeView, setActiveView] = useState<MetricView>('overview');

  const avgProcessingTime = stats.commandCount > 0 
    ? Math.round(stats.totalProcessingTime / stats.commandCount) 
    : 0;

  const successRate = stats.commandCount > 0 
    ? Math.round((stats.successfulCommands / stats.commandCount) * 100) 
    : 0;

  const formatDuration = (ms: number) => {
    const seconds = Math.floor(ms / 1000);
    const minutes = Math.floor(seconds / 60);
    const hours = Math.floor(minutes / 60);

    if (hours > 0) {
      return `${hours}h ${minutes % 60}m`;
    } else if (minutes > 0) {
      return `${minutes}m ${seconds % 60}s`;
    } else {
      return `${seconds}s`;
    }
  };

  const commandDistribution = recentResponses.reduce((dist, response) => {
    response.commands.forEach(cmd => {
      dist[cmd.action] = (dist[cmd.action] || 0) + 1;
    });
    return dist;
  }, {} as Record<string, number>);

  const topCommands = Object.entries(commandDistribution)
    .sort(([,a], [,b]) => b - a)
    .slice(0, 5);

  const views = [
    { id: 'overview' as MetricView, label: 'Overview', icon: '📊' },
    { id: 'performance' as MetricView, label: 'Performance', icon: '⚡' },
    { id: 'commands' as MetricView, label: 'Commands', icon: '🎯' },
    { id: 'errors' as MetricView, label: 'Errors', icon: '⚠️' }
  ];

  return (
    <div className="h-full flex flex-col">
      {/* Header */}
      <div className="p-4 border-b border-gray-200">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-sm font-semibold text-gray-700">
              Weave Observability
            </h3>
            <p className="text-xs text-gray-500 mt-1">
              Session: {sessionId.split('-').pop()}
            </p>
          </div>
          
          <button
            onClick={onResetSession}
            className="text-xs text-red-600 hover:text-red-800 border border-red-200 px-3 py-1 rounded hover:bg-red-50"
          >
            Reset Session
          </button>
        </div>
      </div>

      {/* View Tabs */}
      <div className="flex border-b border-gray-200 bg-gray-50">
        {views.map((view) => (
          <button
            key={view.id}
            onClick={() => setActiveView(view.id)}
            className={`flex-1 px-3 py-2 text-xs font-medium transition-colors ${
              activeView === view.id
                ? 'text-purple-600 bg-white border-b-2 border-purple-500'
                : 'text-gray-600 hover:text-purple-600 hover:bg-gray-100'
            }`}
          >
            <div className="flex items-center justify-center space-x-1">
              <span>{view.icon}</span>
              <span>{view.label}</span>
            </div>
          </button>
        ))}
      </div>

      {/* Content */}
      <div className="flex-1 overflow-auto p-4">
        {activeView === 'overview' && (
          <div className="space-y-4">
            {/* Key Metrics */}
            <div className="grid grid-cols-2 gap-4">
              <div className="bg-blue-50 rounded-lg p-4">
                <div className="text-2xl font-bold text-blue-600">
                  {formatDuration(stats.sessionDuration)}
                </div>
                <div className="text-sm text-blue-800">Session Duration</div>
              </div>
              
              <div className="bg-green-50 rounded-lg p-4">
                <div className="text-2xl font-bold text-green-600">
                  {stats.commandCount}
                </div>
                <div className="text-sm text-green-800">Commands Processed</div>
              </div>
              
              <div className="bg-purple-50 rounded-lg p-4">
                <div className="text-2xl font-bold text-purple-600">
                  {successRate}%
                </div>
                <div className="text-sm text-purple-800">Success Rate</div>
              </div>
              
              <div className="bg-orange-50 rounded-lg p-4">
                <div className="text-2xl font-bold text-orange-600">
                  {avgProcessingTime}ms
                </div>
                <div className="text-sm text-orange-800">Avg Processing</div>
              </div>
            </div>

            {/* Session Info */}
            <div className="bg-gray-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-gray-700 mb-3">Session Details</h4>
              <div className="space-y-2 text-xs">
                <div className="flex justify-between">
                  <span className="text-gray-600">Session ID:</span>
                  <span className="font-mono text-gray-800">{sessionId}</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-600">Started:</span>
                  <span className="text-gray-800">
                    {new Date(Date.now() - stats.sessionDuration).toLocaleString()}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-600">Total Processing Time:</span>
                  <span className="text-gray-800">{stats.totalProcessingTime}ms</span>
                </div>
              </div>
            </div>
          </div>
        )}

        {activeView === 'performance' && (
          <div className="space-y-4">
            <div className="bg-gray-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-gray-700 mb-3">Performance Metrics</h4>
              
              {recentResponses.length > 0 ? (
                <div className="space-y-3">
                  {recentResponses.slice(0, 10).map((response, index) => (
                    <div key={index} className="flex items-center justify-between py-2 border-b border-gray-200 last:border-0">
                      <div className="flex-1">
                        <div className="text-xs text-gray-600">
                          {response.commands.length} command(s)
                        </div>
                        <div className="text-sm font-medium text-gray-800 truncate">
                          {response.explanation}
                        </div>
                      </div>
                      <div className="text-right ml-4">
                        <div className={`text-sm font-semibold ${
                          response.processingTime < 1000 ? 'text-green-600' :
                          response.processingTime < 3000 ? 'text-yellow-600' : 'text-red-600'
                        }`}>
                          {response.processingTime}ms
                        </div>
                        <div className="text-xs text-gray-500">
                          {new Date(response.timestamp).toLocaleTimeString()}
                        </div>
                      </div>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <div className="text-2xl mb-2">📈</div>
                  <p>No performance data yet</p>
                </div>
              )}
            </div>
          </div>
        )}

        {activeView === 'commands' && (
          <div className="space-y-4">
            <div className="bg-gray-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-gray-700 mb-3">Command Distribution</h4>
              
              {topCommands.length > 0 ? (
                <div className="space-y-2">
                  {topCommands.map(([command, count]) => (
                    <div key={command} className="flex items-center justify-between">
                      <span className="text-sm text-gray-700 capitalize">
                        {command.replace('_', ' ')}
                      </span>
                      <div className="flex items-center space-x-2">
                        <div className="bg-blue-200 rounded-full h-2 w-20 overflow-hidden">
                          <div 
                            className="bg-blue-600 h-full transition-all duration-300"
                            style={{ 
                              width: `${(count / Math.max(...Object.values(commandDistribution))) * 100}%` 
                            }}
                          />
                        </div>
                        <span className="text-sm font-medium text-gray-800 w-8 text-right">
                          {count}
                        </span>
                      </div>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <div className="text-2xl mb-2">🎯</div>
                  <p>No commands executed yet</p>
                </div>
              )}
            </div>

            {/* Command Timeline */}
            {recentResponses.length > 0 && (
              <div className="bg-gray-50 rounded-lg p-4">
                <h4 className="text-sm font-semibold text-gray-700 mb-3">Recent Commands</h4>
                <div className="space-y-2 max-h-64 overflow-auto">
                  {recentResponses.slice(0, 20).map((response, index) => (
                    <div key={index} className="text-xs border-l-2 border-blue-500 pl-3 py-1">
                      <div className="flex items-center justify-between">
                        <span className="font-medium text-gray-800">
                          {response.commands.map(cmd => cmd.action).join(', ')}
                        </span>
                        <span className="text-gray-500">
                          {new Date(response.timestamp).toLocaleTimeString()}
                        </span>
                      </div>
                      <div className="text-gray-600 mt-1">
                        {response.explanation}
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>
        )}

        {activeView === 'errors' && (
          <div className="space-y-4">
            <div className="bg-gray-50 rounded-lg p-4">
              <h4 className="text-sm font-semibold text-gray-700 mb-3">Error Analysis</h4>
              
              <div className="grid grid-cols-2 gap-4 mb-4">
                <div className="text-center">
                  <div className="text-lg font-bold text-red-600">{stats.errorCount}</div>
                  <div className="text-xs text-red-800">Total Errors</div>
                </div>
                <div className="text-center">
                  <div className="text-lg font-bold text-green-600">
                    {stats.commandCount - stats.errorCount}
                  </div>
                  <div className="text-xs text-green-800">Successful</div>
                </div>
              </div>

              {stats.errorCount === 0 ? (
                <div className="text-center py-8 text-gray-500">
                  <div className="text-2xl mb-2">✅</div>
                  <p>No errors recorded</p>
                  <p className="text-xs mt-1">Great job!</p>
                </div>
              ) : (
                <div className="space-y-2">
                  <div className="text-xs text-gray-600">
                    Error rate: {Math.round((stats.errorCount / stats.commandCount) * 100)}%
                  </div>
                  {/* Add specific error details here if tracked */}
                </div>
              )}
            </div>
          </div>
        )}
      </div>

      {/* Footer */}
      <div className="border-t border-gray-200 p-3 bg-gray-50">
        <div className="text-xs text-gray-600 text-center">
          <span className="font-medium">Weights & Biases Weave</span> observability enabled
        </div>
      </div>
    </div>
  );
}
