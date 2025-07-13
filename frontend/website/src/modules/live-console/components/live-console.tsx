/**
 * Live Console Component
 * Main interface for voice-controlled robot interaction with Gemini and Weave observability
 */

import React, { useState, useEffect } from 'react';
import { useLiveConsole } from '../hooks/use-live-console';
import { VoiceInput } from './voice-input';
import { CommandHistory } from './command-history';
import { WeaveObservability } from './weave-observability';
import { RobotStatus } from './robot-status';

interface LiveConsoleProps {
  geminiApiKey?: string;
  autoExecuteCommands?: boolean;
  className?: string;
}

type TabType = 'voice' | 'history' | 'weave' | 'robot';

export function LiveConsole({ 
  geminiApiKey, 
  autoExecuteCommands = true,
  className = '' 
}: LiveConsoleProps) {
  const [activeTab, setActiveTab] = useState<TabType>('voice');
  const [apiKey, setApiKey] = useState(geminiApiKey || '');
  const [showApiKeyInput, setShowApiKeyInput] = useState(!geminiApiKey);

  const liveConsole = useLiveConsole({
    geminiApiKey: apiKey,
    autoExecuteCommands,
    confidenceThreshold: 0.7
  });

  const { state, stats, recentResponses, executionHistory, robotState, actions, isSupported } = liveConsole;

  useEffect(() => {
    // Try to get API key from localStorage if not provided
    if (!apiKey) {
      const savedKey = localStorage.getItem('gemini_api_key');
      if (savedKey) {
        setApiKey(savedKey);
        setShowApiKeyInput(false);
      }
    }
  }, [apiKey]);

  const handleApiKeySubmit = (key: string) => {
    setApiKey(key);
    localStorage.setItem('gemini_api_key', key);
    setShowApiKeyInput(false);
  };

  const tabs = [
    { 
      id: 'voice' as TabType, 
      label: 'Voice Control', 
      icon: '🎤', 
      count: state.isListening ? 1 : 0,
      color: state.isListening ? 'text-green-600' : 'text-gray-600'
    },
    { 
      id: 'history' as TabType, 
      label: 'Commands', 
      icon: '📝', 
      count: executionHistory.length,
      color: 'text-blue-600'
    },
    { 
      id: 'weave' as TabType, 
      label: 'Observability', 
      icon: '📊', 
      count: stats.commandCount,
      color: 'text-purple-600'
    },
    { 
      id: 'robot' as TabType, 
      label: 'Robot Status', 
      icon: '🤖', 
      count: robotState.isConnected ? 1 : 0,
      color: robotState.isConnected ? 'text-green-600' : 'text-red-600'
    }
  ];

  if (!isSupported) {
    return (
      <div className={`bg-red-50 border border-red-200 rounded-lg p-6 ${className}`}>
        <div className="text-center">
          <div className="text-4xl mb-4">⚠️</div>
          <h3 className="text-lg font-semibold text-red-800 mb-2">
            Speech Recognition Not Supported
          </h3>
          <p className="text-red-600">
            Your browser doesn't support the Web Speech API. Please use Chrome, Safari, or Edge.
          </p>
        </div>
      </div>
    );
  }

  if (showApiKeyInput) {
    return (
      <div className={`bg-white rounded-lg border shadow-sm p-6 ${className}`}>
        <div className="text-center">
          <div className="text-4xl mb-4">🔑</div>
          <h3 className="text-lg font-semibold text-gray-800 mb-2">
            Gemini API Key Required
          </h3>
          <p className="text-gray-600 mb-4">
            Enter your Gemini API key to enable voice-controlled robot commands.
          </p>
          <div className="max-w-md mx-auto">
            <input
              type="password"
              placeholder="Gemini API Key"
              className="w-full px-4 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-blue-500 focus:border-transparent"
              onKeyDown={(e) => {
                if (e.key === 'Enter') {
                  handleApiKeySubmit(e.currentTarget.value);
                }
              }}
            />
            <button
              onClick={() => {
                const input = document.querySelector('input[type="password"]') as HTMLInputElement;
                if (input?.value) {
                  handleApiKeySubmit(input.value);
                }
              }}
              className="mt-2 w-full bg-blue-600 text-white py-2 px-4 rounded-lg hover:bg-blue-700 transition-colors"
            >
              Connect
            </button>
          </div>
          <p className="text-xs text-gray-500 mt-4">
            Your API key is stored locally and never sent to our servers.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className={`bg-white rounded-lg border shadow-sm flex flex-col h-full ${className}`}>
      {/* Header */}
      <div className="border-b border-gray-200 p-4">
        <div className="flex items-center justify-between">
          <div>
            <h2 className="text-lg font-semibold text-gray-800">
              RoboWeave Live Console
            </h2>
            <p className="text-sm text-gray-600">
              Voice-controlled robot interaction with AI observability
            </p>
          </div>
          
          {/* Connection Status */}
          <div className="flex items-center space-x-2">
            <div className={`w-2 h-2 rounded-full ${
              state.isConnected 
                ? 'bg-green-500' 
                : 'bg-red-500'
            }`}></div>
            <span className="text-xs text-gray-600">
              {state.isConnected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
        </div>

        {/* Session Stats */}
        <div className="mt-3 flex items-center space-x-4 text-xs text-gray-500">
          <span>Session: {Math.floor(stats.sessionDuration / 1000)}s</span>
          <span>Commands: {stats.commandCount}</span>
          <span>Success: {stats.successfulCommands}/{stats.commandCount}</span>
          {stats.totalProcessingTime > 0 && (
            <span>Avg: {Math.round(stats.totalProcessingTime / stats.commandCount)}ms</span>
          )}
        </div>
      </div>

      {/* Tab Navigation */}
      <div className="flex border-b border-gray-200 bg-gray-50">
        {tabs.map((tab) => (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`flex-1 px-4 py-3 text-sm font-medium transition-colors relative ${
              activeTab === tab.id
                ? 'text-blue-600 bg-white border-b-2 border-blue-500'
                : 'text-gray-600 hover:text-blue-600 hover:bg-gray-100'
            }`}
          >
            <div className="flex items-center justify-center space-x-2">
              <span className={tab.color}>{tab.icon}</span>
              <span>{tab.label}</span>
              {tab.count > 0 && (
                <span className="bg-gray-200 text-gray-700 px-1.5 py-0.5 rounded-full text-xs">
                  {tab.count}
                </span>
              )}
            </div>
          </button>
        ))}
      </div>

      {/* Error Display */}
      {state.error && (
        <div className="bg-red-50 border-l-4 border-red-400 p-4 m-4 rounded">
          <div className="flex">
            <div className="flex-shrink-0">
              <span className="text-red-400">⚠️</span>
            </div>
            <div className="ml-3">
              <p className="text-sm text-red-700">{state.error}</p>
              <button
                onClick={actions.clearError}
                className="mt-2 text-xs text-red-600 hover:text-red-800 underline"
              >
                Dismiss
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Tab Content */}
      <div className="flex-1 overflow-hidden">
        {activeTab === 'voice' && (
          <VoiceInput
            state={state}
            actions={actions}
            recentResponses={recentResponses}
            autoExecute={autoExecuteCommands}
          />
        )}
        
        {activeTab === 'history' && (
          <CommandHistory
            executionHistory={executionHistory}
            onExecuteCommand={actions.executeCommand}
          />
        )}
        
        {activeTab === 'weave' && (
          <WeaveObservability
            sessionId={state.sessionId}
            stats={stats}
            recentResponses={recentResponses}
            onResetSession={actions.resetSession}
          />
        )}
        
        {activeTab === 'robot' && (
          <RobotStatus
            robotState={robotState}
            onRefreshState={() => {
              // Trigger robot state refresh if needed
            }}
          />
        )}
      </div>
    </div>
  );
}
