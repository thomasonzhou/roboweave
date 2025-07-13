/**
 * Command History Component
 * Shows execution history and allows re-running commands
 */

import { useState } from 'react';
import type { RobotCommand } from '../services/gemini-live';

interface ExecutionHistoryItem {
  command: RobotCommand;
  timestamp: number;
  success: boolean;
  error?: string;
}

interface CommandHistoryProps {
  executionHistory: ExecutionHistoryItem[];
  onExecuteCommand: (commands: RobotCommand[]) => Promise<void>;
}

export function CommandHistory({ executionHistory, onExecuteCommand }: CommandHistoryProps) {
  const [selectedItems, setSelectedItems] = useState<Set<number>>(new Set());
  const [isExecuting, setIsExecuting] = useState(false);

  const toggleSelection = (index: number) => {
    const newSelection = new Set(selectedItems);
    if (newSelection.has(index)) {
      newSelection.delete(index);
    } else {
      newSelection.add(index);
    }
    setSelectedItems(newSelection);
  };

  const selectAll = () => {
    if (selectedItems.size === executionHistory.length) {
      setSelectedItems(new Set());
    } else {
      setSelectedItems(new Set(executionHistory.map((_, index) => index)));
    }
  };

  const executeSelected = async () => {
    if (selectedItems.size === 0) return;

    setIsExecuting(true);
    try {
      const commands = Array.from(selectedItems)
        .map(index => executionHistory[index].command)
        .filter(Boolean);
      
      await onExecuteCommand(commands);
      setSelectedItems(new Set());
    } catch (error) {
      console.error('Failed to execute selected commands:', error);
    } finally {
      setIsExecuting(false);
    }
  };

  const getActionIcon = (action: string) => {
    const icons: Record<string, string> = {
      move_forward: '⬆️',
      move_backward: '⬇️',
      move_left: '⬅️',
      move_right: '➡️',
      rotate_left: '↺',
      rotate_right: '↻',
      stop: '⏹️',
      flip: '🤸',
      circle: '🔄'
    };
    return icons[action] || '❓';
  };

  const formatParameters = (params?: Record<string, any>) => {
    if (!params || Object.keys(params).length === 0) return '';
    
    const formatted = Object.entries(params)
      .map(([key, value]) => {
        if (typeof value === 'number') {
          return `${key}: ${value.toFixed(1)}`;
        }
        return `${key}: ${value}`;
      })
      .join(', ');
    
    return ` (${formatted})`;
  };

  const groupedHistory = executionHistory.reduce((groups, item, index) => {
    const date = new Date(item.timestamp).toDateString();
    if (!groups[date]) {
      groups[date] = [];
    }
    groups[date].push({ ...item, originalIndex: index });
    return groups;
  }, {} as Record<string, (ExecutionHistoryItem & { originalIndex: number })[]>);

  return (
    <div className="h-full flex flex-col">
      {/* Header */}
      <div className="p-4 border-b border-gray-200">
        <div className="flex items-center justify-between">
          <h3 className="text-sm font-semibold text-gray-700">
            Command History ({executionHistory.length})
          </h3>
          
          {executionHistory.length > 0 && (
            <div className="flex items-center space-x-2">
              <button
                onClick={selectAll}
                className="text-xs text-blue-600 hover:text-blue-800"
              >
                {selectedItems.size === executionHistory.length ? 'Deselect All' : 'Select All'}
              </button>
              
              {selectedItems.size > 0 && (
                <button
                  onClick={executeSelected}
                  disabled={isExecuting}
                  className="bg-blue-600 text-white px-3 py-1 rounded text-xs hover:bg-blue-700 disabled:opacity-50"
                >
                  {isExecuting ? 'Executing...' : `Execute ${selectedItems.size}`}
                </button>
              )}
            </div>
          )}
        </div>

        {selectedItems.size > 0 && (
          <div className="mt-2 text-xs text-gray-600">
            {selectedItems.size} command(s) selected
          </div>
        )}
      </div>

      {/* History List */}
      <div className="flex-1 overflow-auto">
        {executionHistory.length === 0 ? (
          <div className="flex flex-col items-center justify-center h-full text-gray-500">
            <div className="text-4xl mb-2">📝</div>
            <p className="text-center">No commands executed yet</p>
            <p className="text-sm text-center mt-1">
              Use voice control to start building your command history
            </p>
          </div>
        ) : (
          <div className="space-y-4 p-4">
            {Object.entries(groupedHistory)
              .sort(([a], [b]) => new Date(b).getTime() - new Date(a).getTime())
              .map(([date, items]) => (
                <div key={date}>
                  <h4 className="text-xs font-medium text-gray-600 mb-2 sticky top-0 bg-white">
                    {date === new Date().toDateString() ? 'Today' : date}
                  </h4>
                  
                  <div className="space-y-2">
                    {items.map((item) => (
                      <div
                        key={item.originalIndex}
                        className={`border rounded-lg p-3 transition-colors cursor-pointer ${
                          selectedItems.has(item.originalIndex)
                            ? 'border-blue-500 bg-blue-50'
                            : item.success
                            ? 'border-green-200 bg-green-50'
                            : 'border-red-200 bg-red-50'
                        }`}
                        onClick={() => toggleSelection(item.originalIndex)}
                      >
                        <div className="flex items-center justify-between">
                          <div className="flex items-center space-x-2">
                            <span className="text-lg">
                              {getActionIcon(item.command.action)}
                            </span>
                            <div>
                              <span className="text-sm font-medium text-gray-800">
                                {item.command.action.replace('_', ' ')}
                                {formatParameters(item.command.parameters)}
                              </span>
                              {item.command.reasoning && (
                                <p className="text-xs text-gray-600 mt-1">
                                  {item.command.reasoning}
                                </p>
                              )}
                            </div>
                          </div>
                          
                          <div className="flex items-center space-x-2">
                            <span className={`text-xs px-2 py-1 rounded-full ${
                              item.success
                                ? 'bg-green-100 text-green-700'
                                : 'bg-red-100 text-red-700'
                            }`}>
                              {item.success ? 'Success' : 'Failed'}
                            </span>
                            
                            <span className="text-xs text-gray-500">
                              {new Date(item.timestamp).toLocaleTimeString()}
                            </span>
                          </div>
                        </div>

                        {item.error && (
                          <div className="mt-2 text-xs text-red-600 bg-red-100 rounded p-2">
                            Error: {item.error}
                          </div>
                        )}

                        {item.command.confidence && (
                          <div className="mt-2 text-xs text-gray-600">
                            Confidence: {Math.round(item.command.confidence * 100)}%
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              ))}
          </div>
        )}
      </div>

      {/* Statistics */}
      {executionHistory.length > 0 && (
        <div className="border-t border-gray-200 p-4 bg-gray-50">
          <div className="grid grid-cols-3 gap-4 text-center">
            <div>
              <div className="text-lg font-semibold text-gray-800">
                {executionHistory.length}
              </div>
              <div className="text-xs text-gray-600">Total</div>
            </div>
            <div>
              <div className="text-lg font-semibold text-green-600">
                {executionHistory.filter(item => item.success).length}
              </div>
              <div className="text-xs text-gray-600">Successful</div>
            </div>
            <div>
              <div className="text-lg font-semibold text-red-600">
                {executionHistory.filter(item => !item.success).length}
              </div>
              <div className="text-xs text-gray-600">Failed</div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
