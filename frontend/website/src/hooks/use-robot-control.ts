/**
 * React Hook for Robot Control via MCP
 * 
 * This hook provides a React-friendly interface for controlling the robot
 * through the MCP server with state management and error handling.
 */

import { useState, useEffect, useCallback } from 'react';
import { mcpRobotClient, type RobotState, type MCPToolResponse } from '~/services/mcp-client';

export interface UseRobotControlState {
    isConnected: boolean;
    isLoading: boolean;
    robotState: RobotState | null;
    lastResponse: MCPToolResponse | null;
    error: string | null;
}

export interface UseRobotControlActions {
    connect: () => Promise<boolean>;
    disconnect: () => void;
    refreshState: () => Promise<void>;
    moveForward: (distance?: number) => Promise<MCPToolResponse>;
    moveBackward: (distance?: number) => Promise<MCPToolResponse>;
    moveLeft: (distance?: number) => Promise<MCPToolResponse>;
    moveRight: (distance?: number) => Promise<MCPToolResponse>;
    rotateLeft: (angle?: number) => Promise<MCPToolResponse>;
    rotateRight: (angle?: number) => Promise<MCPToolResponse>;
    runInCircle: (radius?: number, duration?: number) => Promise<MCPToolResponse>;
    stopAndStay: () => Promise<MCPToolResponse>;
    doFlip: () => Promise<MCPToolResponse>;
}

export function useRobotControl(): [UseRobotControlState, UseRobotControlActions] {
    const [state, setState] = useState<UseRobotControlState>({
        isConnected: false,
        isLoading: false,
        robotState: null,
        lastResponse: null,
        error: null,
    });

    // Auto-connect on mount
    useEffect(() => {
        connect();
    }, []);

    // Auto-refresh robot state when connected
    useEffect(() => {
        if (!state.isConnected) return;

        const interval = setInterval(() => {
            refreshState();
        }, 2000); // Refresh every 2 seconds

        return () => clearInterval(interval);
    }, [state.isConnected]);

    const connect = useCallback(async (): Promise<boolean> => {
        setState(prev => ({ ...prev, isLoading: true, error: null }));

        try {
            const connected = await mcpRobotClient.connect();
            setState(prev => ({
                ...prev,
                isConnected: connected,
                isLoading: false,
                error: connected ? null : 'Failed to connect to robot control server'
            }));

            if (connected) {
                // Immediately get initial state
                await refreshState();
            }

            return connected;
        } catch (error) {
            const errorMessage = error instanceof Error ? error.message : 'Unknown connection error';
            setState(prev => ({
                ...prev,
                isConnected: false,
                isLoading: false,
                error: errorMessage
            }));
            return false;
        }
    }, []);

    const disconnect = useCallback(() => {
        mcpRobotClient.disconnect();
        setState(prev => ({
            ...prev,
            isConnected: false,
            robotState: null,
            error: null
        }));
    }, []);

    const refreshState = useCallback(async (): Promise<void> => {
        try {
            const robotState = await mcpRobotClient.getRobotState();
            setState(prev => ({
                ...prev,
                robotState,
                error: robotState ? null : 'Failed to get robot state'
            }));
        } catch (error) {
            console.error('Failed to refresh robot state:', error);
        }
    }, []);

    const executeCommand = useCallback(async (
        commandFn: () => Promise<MCPToolResponse>,
        description: string
    ): Promise<MCPToolResponse> => {
        setState(prev => ({ ...prev, isLoading: true, error: null }));

        try {
            const response = await commandFn();
            
            setState(prev => ({
                ...prev,
                lastResponse: response,
                isLoading: false,
                error: response.success ? null : response.error || `Failed to ${description}`
            }));

            // Refresh state after successful command
            if (response.success) {
                setTimeout(refreshState, 500); // Small delay to let robot update
            }

            return response;
        } catch (error) {
            const errorMessage = error instanceof Error ? error.message : `Unknown error during ${description}`;
            setState(prev => ({
                ...prev,
                isLoading: false,
                error: errorMessage,
                lastResponse: {
                    success: false,
                    error: errorMessage
                }
            }));

            return {
                success: false,
                error: errorMessage
            };
        }
    }, [refreshState]);

    const actions: UseRobotControlActions = {
        connect,
        disconnect,
        refreshState,
        
        moveForward: useCallback((distance?: number) => 
            executeCommand(() => mcpRobotClient.moveForward(distance), 'move forward'), [executeCommand]),
        
        moveBackward: useCallback((distance?: number) => 
            executeCommand(() => mcpRobotClient.moveBackward(distance), 'move backward'), [executeCommand]),
        
        moveLeft: useCallback((distance?: number) => 
            executeCommand(() => mcpRobotClient.moveLeft(distance), 'move left'), [executeCommand]),
        
        moveRight: useCallback((distance?: number) => 
            executeCommand(() => mcpRobotClient.moveRight(distance), 'move right'), [executeCommand]),
        
        rotateLeft: useCallback((angle?: number) => 
            executeCommand(() => mcpRobotClient.rotateLeft(angle), 'rotate left'), [executeCommand]),
        
        rotateRight: useCallback((angle?: number) => 
            executeCommand(() => mcpRobotClient.rotateRight(angle), 'rotate right'), [executeCommand]),
        
        runInCircle: useCallback((radius?: number, duration?: number) => 
            executeCommand(() => mcpRobotClient.runInCircle(radius, duration), 'run in circle'), [executeCommand]),
        
        stopAndStay: useCallback(() => 
            executeCommand(() => mcpRobotClient.stopAndStay(), 'stop and stay'), [executeCommand]),
        
        doFlip: useCallback(() => 
            executeCommand(() => mcpRobotClient.doFlip(), 'perform flip'), [executeCommand]),
    };

    return [state, actions];
}
