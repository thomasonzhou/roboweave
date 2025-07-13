/**
 * Live Console Hook
 * Integrates text commands, Gemini processing, and robot control
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { GeminiLiveService, type GeminiResponse, type RobotCommand } from '../services/gemini-live';
import { useRobotControl } from '~/hooks/use-robot-control';

export interface LiveConsoleState {
  isProcessing: boolean;
  isConnected: boolean;
  currentCommand: string;
  lastCommand: string;
  error: string | null;
  sessionId: string;
}

export interface LiveConsoleStats {
  sessionDuration: number;
  commandCount: number;
  successfulCommands: number;
  errorCount: number;
  totalProcessingTime: number;
}

interface LiveConsoleConfig {
  backendUrl?: string;
  autoExecuteCommands?: boolean;
  maxProcessingTime?: number;
}

export function useLiveConsole(config: LiveConsoleConfig = {}) {
  const [state, setState] = useState<LiveConsoleState>({
    isProcessing: false,
    isConnected: false,
    currentCommand: '',
    lastCommand: '',
    error: null,
    sessionId: ''
  });

  const [stats, setStats] = useState<LiveConsoleStats>({
    sessionDuration: 0,
    commandCount: 0,
    successfulCommands: 0,
    errorCount: 0,
    totalProcessingTime: 0
  });

  const [recentResponses, setRecentResponses] = useState<GeminiResponse[]>([]);
  const [executionHistory, setExecutionHistory] = useState<{
    command: RobotCommand;
    timestamp: number;
    success: boolean;
    error?: string;
  }[]>([]);

  const geminiServiceRef = useRef<GeminiLiveService | null>(null);
  const sessionStartTimeRef = useRef<number>(Date.now());
  const [robotState, robotActions] = useRobotControl();

  // Initialize Gemini service
  useEffect(() => {
    const initializeService = async () => {
      try {
        console.log('🚀 Initializing Gemini service with backend URL:', config.backendUrl);
        
        geminiServiceRef.current = new GeminiLiveService({
          backendUrl: config.backendUrl
        });

        console.log('🔌 Checking backend connection...');
        // Check if backend is actually reachable
        const isConnected = await geminiServiceRef.current.checkConnection();
        
        console.log('🔌 Connection result:', isConnected);

        setState(prev => ({
          ...prev,
          isConnected,
          sessionId: geminiServiceRef.current!.getSessionId(),
          error: isConnected ? null : 'Backend service is not reachable'
        }));

        sessionStartTimeRef.current = Date.now();
      } catch (error) {
        console.error('🚫 Failed to initialize Gemini service:', error);
        setState(prev => ({
          ...prev,
          error: `Failed to initialize Gemini service: ${error instanceof Error ? error.message : 'Unknown error'}`
        }));
      }
    };

    initializeService();
  }, [config.backendUrl]);

  // Update session stats periodically
  useEffect(() => {
    const interval = setInterval(() => {
      setStats(prev => ({
        ...prev,
        sessionDuration: Date.now() - sessionStartTimeRef.current
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const executeRobotCommands = async (commands: RobotCommand[]) => {
    for (const command of commands) {
      try {
        const timestamp = Date.now();

        switch (command.action) {
          case 'move_forward':
            await robotActions.moveForward(command.parameters?.distance);
            break;
          case 'move_backward':
            await robotActions.moveBackward(command.parameters?.distance);
            break;
          case 'move_left':
            await robotActions.moveLeft(command.parameters?.distance);
            break;
          case 'move_right':
            await robotActions.moveRight(command.parameters?.distance);
            break;
          case 'rotate_left':
            await robotActions.rotateLeft(command.parameters?.angle);
            break;
          case 'rotate_right':
            await robotActions.rotateRight(command.parameters?.angle);
            break;
          case 'stop':
            await robotActions.stopAndStay();
            break;
          case 'flip':
            await robotActions.doFlip();
            break;
          case 'circle':
            await robotActions.runInCircle(command.parameters?.radius, command.parameters?.duration);
            break;
          default:
            throw new Error(`Unknown command: ${command.action}`);
        }

        setExecutionHistory(prev => [...prev, {
          command,
          timestamp,
          success: true
        }].slice(-20)); // Keep last 20 executions

        setStats(prev => ({ ...prev, successfulCommands: prev.successfulCommands + 1 }));

      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'Unknown error';
        
        setExecutionHistory(prev => [...prev, {
          command,
          timestamp: Date.now(),
          success: false,
          error: errorMessage
        }].slice(-20));

        console.error(`Failed to execute command ${command.action}:`, error);
      }
    }
  };

  const clearError = useCallback(() => {
    setState(prev => ({ ...prev, error: null }));
  }, []);

  const resetSession = useCallback(() => {
    geminiServiceRef.current?.resetSession();
    sessionStartTimeRef.current = Date.now();
    setStats({
      sessionDuration: 0,
      commandCount: 0,
      successfulCommands: 0,
      errorCount: 0,
      totalProcessingTime: 0
    });
    setRecentResponses([]);
    setExecutionHistory([]);
    setState(prev => ({
      ...prev,
      sessionId: geminiServiceRef.current?.getSessionId() || '',
      currentCommand: '',
      lastCommand: '',
      error: null
    }));
  }, []);

  const executeCommand = useCallback(async (commands: RobotCommand[]) => {
    if (commands.length > 0) {
      await executeRobotCommands(commands);
    }
  }, []);

  const processTextCommand = useCallback(async (text: string) => {
    if (!text.trim()) return;

    setState(prev => ({ 
      ...prev, 
      isProcessing: true, 
      lastCommand: text.trim(),
      currentCommand: text.trim()
    }));

    try {
      if (!geminiServiceRef.current) {
        throw new Error('Gemini service not initialized');
      }

      // Create a synthetic speech result for text input
      const textResult = {
        transcript: text.trim(),
        confidence: 1.0,
        isFinal: true,
        timestamp: Date.now()
      };

      const response = await geminiServiceRef.current.processVoiceCommand(textResult);
      
      setRecentResponses(prev => [response, ...prev.slice(0, 9)]); // Keep last 10 responses
      
      setStats(prev => ({
        ...prev,
        commandCount: prev.commandCount + 1,
        totalProcessingTime: prev.totalProcessingTime + response.processingTime
      }));

      // Execute robot commands if auto-execute is enabled
      if (config.autoExecuteCommands !== false && response.commands.length > 0) {
        await executeRobotCommands(response.commands);
      }

      setState(prev => ({ ...prev, isProcessing: false, error: null }));

      return response;

    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      setState(prev => ({ 
        ...prev, 
        isProcessing: false, 
        error: `Processing failed: ${errorMessage}`
      }));
      
      setStats(prev => ({ ...prev, errorCount: prev.errorCount + 1 }));
      throw error;
    }
  }, [config.autoExecuteCommands]);

  return {
    state,
    stats,
    recentResponses,
    executionHistory,
    robotState,
    actions: {
      clearError,
      resetSession,
      executeCommand,
      processTextCommand
    }
  };
}
