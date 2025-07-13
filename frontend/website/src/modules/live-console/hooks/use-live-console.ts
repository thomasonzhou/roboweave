/**
 * Live Consol}

export interface LiveConsoleStats {
  sessionDuration: number;
  commandCount: number;
  successfulCommands: number;
  errorCount: number;
  totalProcessingTime: number;
}

export interface LiveConsoleConfig {ntegrates speech recognition, Gemini processing, and robot control
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { speechRecognitionService, type SpeechRecognitionResult } from '../services/speech-recognition';
import { GeminiLiveService, type GeminiResponse, type RobotCommand } from '../services/gemini-live';
import { useRobotControl } from '~/hooks/use-robot-control';

export interface LiveConsoleState {
  isListening: boolean;
  isProcessing: boolean;
  isConnected: boolean;
  currentTranscript: string;
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
  confidenceThreshold?: number;
  maxProcessingTime?: number;
}

export function useLiveConsole(config: LiveConsoleConfig = {}) {
  const [state, setState] = useState<LiveConsoleState>({
    isListening: false,
    isProcessing: false,
    isConnected: false,
    currentTranscript: '',
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
        geminiServiceRef.current = new GeminiLiveService({
          backendUrl: config.backendUrl
        });

        // Check if backend is actually reachable
        const isConnected = await geminiServiceRef.current.checkConnection();

        setState(prev => ({
          ...prev,
          isConnected,
          sessionId: geminiServiceRef.current!.getSessionId(),
          error: isConnected ? null : 'Backend service is not reachable'
        }));

        sessionStartTimeRef.current = Date.now();
      } catch (error) {
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

  // Set up speech recognition callbacks
  useEffect(() => {
    if (!speechRecognitionService.getIsSupported()) {
      setState(prev => ({ ...prev, error: 'Speech recognition not supported in this browser' }));
      return;
    }

    speechRecognitionService.onResult(handleSpeechResult);
    speechRecognitionService.onError(handleSpeechError);
    speechRecognitionService.onStatusChange(handleSpeechStatusChange);

    return () => {
      speechRecognitionService.destroy();
    };
  }, []);

  const handleSpeechResult = useCallback(async (result: SpeechRecognitionResult) => {
    console.log('Speech result received:', result);
    setState(prev => ({ ...prev, currentTranscript: result.transcript }));

    // Only process final results with sufficient confidence
    if (result.isFinal && result.confidence >= (config.confidenceThreshold || 0.7)) {
      console.log('Processing final speech result:', result.transcript);
      setState(prev => ({ ...prev, isProcessing: true, lastCommand: result.transcript }));

      try {
        if (!geminiServiceRef.current) {
          throw new Error('Gemini service not initialized');
        }

        const response = await geminiServiceRef.current.processVoiceCommand(result);
        
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

      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'Unknown error';
        setState(prev => ({ 
          ...prev, 
          isProcessing: false, 
          error: `Processing failed: ${errorMessage}`
        }));
        
        setStats(prev => ({ ...prev, errorCount: prev.errorCount + 1 }));
      }
    }
  }, [config.autoExecuteCommands, config.confidenceThreshold]);

  const handleSpeechError = useCallback((error: string) => {
    setState(prev => ({ ...prev, error: `Speech recognition error: ${error}` }));
  }, []);

  const handleSpeechStatusChange = useCallback((status: 'listening' | 'stopped' | 'error') => {
    setState(prev => ({ 
      ...prev, 
      isListening: status === 'listening',
      error: status === 'error' ? 'Speech recognition error' : prev.error
    }));
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

  const startListening = useCallback(() => {
    console.log('startListening called');
    if (!speechRecognitionService.getIsSupported()) {
      console.error('Speech recognition not supported');
      setState(prev => ({ ...prev, error: 'Speech recognition not supported' }));
      return;
    }

    console.log('Starting speech recognition...');
    setState(prev => ({ ...prev, error: null }));
    speechRecognitionService.startListening();
  }, []);

  const stopListening = useCallback(() => {
    speechRecognitionService.stopListening();
  }, []);

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
      currentTranscript: '',
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
      currentTranscript: text.trim()
    }));

    try {
      if (!geminiServiceRef.current) {
        throw new Error('Gemini service not initialized');
      }

      // Create a synthetic speech result for text input
      const textResult: SpeechRecognitionResult = {
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
      startListening,
      stopListening,
      clearError,
      resetSession,
      executeCommand,
      processTextCommand
    },
    isSupported: speechRecognitionService.getIsSupported()
  };
}
