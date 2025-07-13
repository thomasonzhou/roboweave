/**
 * Text Input Component
 * Handles text command interface and real-time feedback
 */

import { useState } from 'react';
import type { LiveConsoleState } from '../hooks/use-live-console';
import type { GeminiResponse, RobotCommand } from '../services/gemini-live';

interface TextInputProps {
  state: LiveConsoleState;
  actions: {
    executeCommand: (commands: RobotCommand[]) => Promise<void>;
    processTextCommand: (text: string) => Promise<GeminiResponse | void>;
  };
  recentResponses: GeminiResponse[];
  autoExecute: boolean;
}

export function TextInput({ state, actions, recentResponses, autoExecute }: TextInputProps) {
  const [selectedResponse, setSelectedResponse] = useState<GeminiResponse | null>(null);
  const [textInput, setTextInput] = useState('');
  const [isProcessingText, setIsProcessingText] = useState(false);

  const handleExecuteCommands = async (response: GeminiResponse) => {
    if (response.commands.length > 0) {
      await actions.executeCommand(response.commands);
    }
  };

  const handleTextSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!textInput.trim() || isProcessingText || !state.isConnected) return;

    setIsProcessingText(true);
    try {
      await actions.processTextCommand(textInput.trim());
      setTextInput('');
    } catch (error) {
      console.error('Text command failed:', error);
    } finally {
      setIsProcessingText(false);
    }
  };

  return (
    <div className="h-full flex flex-col">
      {/* Text Input Section */}
      <div className="p-6 border-b border-gray-200">
        <h3 className="text-lg font-semibold text-gray-700 mb-4">Robot Commands</h3>
        <form onSubmit={handleTextSubmit} className="flex space-x-2">
          <input
            type="text"
            value={textInput}
            onChange={(e) => setTextInput(e.target.value)}
            placeholder="Type a command (e.g., 'move forward', 'turn left', 'stop')"
            disabled={isProcessingText || !state.isConnected}
            className="flex-1 px-4 py-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent disabled:bg-gray-100 disabled:cursor-not-allowed text-gray-900 placeholder-gray-500 text-lg"
          />
          <button
            type="submit"
            disabled={!textInput.trim() || isProcessingText || !state.isConnected}
            className="px-6 py-3 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:bg-gray-300 disabled:cursor-not-allowed transition-colors font-medium"
          >
            {isProcessingText ? '⏳' : 'Send'}
          </button>
        </form>
        
        <div className="mt-3 text-sm text-gray-600">
          {state.isConnected ? (
            <>Type commands or use quick actions below. {autoExecute ? 'Commands will execute automatically.' : 'Commands will be queued for manual execution.'}</>
          ) : (
            <span className="text-red-600">⚠️ Backend service disconnected</span>
          )}
        </div>
        
        {/* Quick Command Buttons */}
        <div className="mt-4">
          <p className="text-sm text-gray-700 mb-3 font-medium">Quick Commands:</p>
          <div className="grid grid-cols-3 gap-2">
            {[
              'move forward',
              'move backward', 
              'turn left',
              'turn right',
              'stop',
              'do a flip'
            ].map((cmd) => (
              <button
                key={cmd}
                onClick={() => {
                  setTextInput(cmd);
                  const form = document.querySelector('form') as HTMLFormElement;
                  if (form) form.requestSubmit();
                }}
                disabled={isProcessingText || !state.isConnected}
                className="px-3 py-2 text-sm bg-white border border-gray-300 rounded-lg hover:bg-gray-50 disabled:bg-gray-100 disabled:cursor-not-allowed transition-colors text-gray-700 hover:text-gray-900 hover:border-gray-400"
              >
                {cmd}
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* Recent Responses */}
      <div className="flex-1 overflow-auto p-4">
        <h3 className="text-sm font-semibold text-gray-700 mb-3">Recent Commands</h3>
        
        {recentResponses.length === 0 ? (
          <div className="text-center py-8 text-gray-500">
            <div className="text-4xl mb-2">🎯</div>
            <p>No commands yet. Try typing:</p>
            <div className="mt-2 text-sm space-y-1">
              <p>"Move forward"</p>
              <p>"Turn left"</p>
              <p>"Stop the robot"</p>
              <p>"Do a flip"</p>
            </div>
          </div>
        ) : (
          <div className="space-y-3">
            {recentResponses.map((response, index) => (
              <div
                key={index}
                className={`border rounded-lg p-3 cursor-pointer transition-colors ${
                  selectedResponse === response
                    ? 'border-blue-500 bg-blue-50'
                    : 'border-gray-200 hover:border-gray-300'
                }`}
                onClick={() => setSelectedResponse(selectedResponse === response ? null : response)}
              >
                <div className="flex items-center justify-between">
                  <span className="text-sm font-medium text-gray-800">
                    {response.commands.length} command(s)
                  </span>
                  <span className="text-xs text-gray-500">
                    {new Date(response.timestamp).toLocaleTimeString()}
                  </span>
                </div>
                
                <p className="text-sm text-gray-600 mt-1">
                  {response.explanation}
                </p>

                {selectedResponse === response && (
                  <div className="mt-3 pt-3 border-t border-gray-200">
                    <div className="space-y-2">
                      {response.commands.map((command, cmdIndex) => (
                        <div key={cmdIndex} className="flex items-center justify-between bg-gray-50 rounded p-2">
                          <div>
                            <span className="text-sm font-medium text-gray-800">
                              {command.action}
                            </span>
                            {command.parameters && Object.keys(command.parameters).length > 0 && (
                              <span className="text-xs text-gray-600 ml-2">
                                {JSON.stringify(command.parameters)}
                              </span>
                            )}
                          </div>
                          <span className="text-xs text-gray-500">
                            {Math.round(command.confidence * 100)}%
                          </span>
                        </div>
                      ))}
                    </div>

                    {!autoExecute && (
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          handleExecuteCommands(response);
                        }}
                        className="mt-3 w-full bg-blue-600 text-white py-2 px-4 rounded text-sm hover:bg-blue-700 transition-colors"
                      >
                        Execute Commands
                      </button>
                    )}
                  </div>
                )}
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Live Feedback Section */}
      {recentResponses.length > 0 && (
        <div className="border-t border-gray-200 p-4 bg-blue-50">
          <h4 className="text-sm font-semibold text-blue-700 mb-2">Latest Response Feedback</h4>
          <div className="text-sm text-gray-700">
            <p className="font-medium text-blue-600 mb-1">
              AI Explanation: "{recentResponses[0].explanation}"
            </p>
            {recentResponses[0].commands.length > 0 && (
              <div className="mt-2">
                <p className="text-xs text-gray-600 mb-1">Generated Commands:</p>
                <div className="space-y-1">
                  {recentResponses[0].commands.map((cmd, idx) => (
                    <div key={idx} className="flex items-center space-x-2 text-xs">
                      <span className="bg-blue-100 text-blue-700 px-2 py-1 rounded font-mono">
                        {cmd.action}
                      </span>
                      {cmd.parameters && Object.keys(cmd.parameters).length > 0 && (
                        <span className="text-gray-600">
                          {Object.entries(cmd.parameters).map(([key, value]) => 
                            `${key}: ${value}`
                          ).join(', ')}
                        </span>
                      )}
                      {cmd.reasoning && (
                        <span className="text-gray-500 italic">- {cmd.reasoning}</span>
                      )}
                      <span className="text-blue-600 font-medium">
                        {Math.round(cmd.confidence * 100)}%
                      </span>
                    </div>
                  ))}
                </div>
              </div>
            )}
            <div className="mt-2 text-xs text-gray-500">
              Processing time: {recentResponses[0].processingTime.toFixed(0)}ms
            </div>
          </div>
        </div>
      )}

      {/* Instructions */}
      <div className="border-t border-gray-200 p-4 bg-gray-50">
        <div className="text-xs text-gray-600">
          <p className="font-medium mb-1">Available Commands:</p>
          <div className="grid grid-cols-2 gap-1">
            <span>• "Move forward/back"</span>
            <span>• "Turn left/right"</span>
            <span>• "Stop the robot"</span>
            <span>• "Do a flip"</span>
            <span>• "Move in a circle"</span>
            <span>• "Go to position X Y"</span>
          </div>
        </div>
      </div>
    </div>
  );
}
