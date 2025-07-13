import { useCallback, useEffect, useState } from 'react';
import { mcpRobotClient, type RobotState, type MCPToolResponse } from '~/services/mcp-client';

export interface UseMCPRobotResult {
    // Connection state
    connected: boolean;
    connecting: boolean;
    
    // Robot state
    robotState: RobotState | null;
    
    // Actions
    connect: () => Promise<boolean>;
    disconnect: () => void;
    refreshState: () => Promise<void>;
    
    // Movement commands
    moveForward: (distance?: number) => Promise<MCPToolResponse>;
    moveBackward: (distance?: number) => Promise<MCPToolResponse>;
    moveLeft: (distance?: number) => Promise<MCPToolResponse>;
    moveRight: (distance?: number) => Promise<MCPToolResponse>;
    
    // Rotation commands
    rotateLeft: (angle?: number) => Promise<MCPToolResponse>;
    rotateRight: (angle?: number) => Promise<MCPToolResponse>;
    
    // Complex motions
    runInCircle: (radius?: number, duration?: number) => Promise<MCPToolResponse>;
    stopAndStay: () => Promise<MCPToolResponse>;
    doFlip: () => Promise<MCPToolResponse>;
    
    // Loading states
    loading: {
        move: boolean;
        rotate: boolean;
        complex: boolean;
    };
}

export function useMCPRobot(): UseMCPRobotResult {
    const [connected, setConnected] = useState(false);
    const [connecting, setConnecting] = useState(false);
    const [robotState, setRobotState] = useState<RobotState | null>(null);
    const [loading, setLoading] = useState({
        move: false,
        rotate: false,
        complex: false
    });

    // Connect to MCP server
    const connect = useCallback(async (): Promise<boolean> => {
        setConnecting(true);
        try {
            const success = await mcpRobotClient.connect();
            setConnected(success);
            if (success) {
                await refreshState();
            }
            return success;
        } catch (error) {
            console.error('Failed to connect to MCP server:', error);
            setConnected(false);
            return false;
        } finally {
            setConnecting(false);
        }
    }, []);

    // Disconnect from MCP server
    const disconnect = useCallback(() => {
        mcpRobotClient.disconnect();
        setConnected(false);
        setRobotState(null);
    }, []);

    // Refresh robot state
    const refreshState = useCallback(async () => {
        if (!connected) return;
        
        try {
            const state = await mcpRobotClient.getRobotState();
            setRobotState(state);
        } catch (error) {
            console.error('Failed to refresh robot state:', error);
        }
    }, [connected]);

    // Movement commands with loading states
    const moveForward = useCallback(async (distance?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, move: true }));
        try {
            const result = await mcpRobotClient.moveForward(distance);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, move: false }));
        }
    }, [refreshState]);

    const moveBackward = useCallback(async (distance?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, move: true }));
        try {
            const result = await mcpRobotClient.moveBackward(distance);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, move: false }));
        }
    }, [refreshState]);

    const moveLeft = useCallback(async (distance?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, move: true }));
        try {
            const result = await mcpRobotClient.moveLeft(distance);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, move: false }));
        }
    }, [refreshState]);

    const moveRight = useCallback(async (distance?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, move: true }));
        try {
            const result = await mcpRobotClient.moveRight(distance);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, move: false }));
        }
    }, [refreshState]);

    // Rotation commands
    const rotateLeft = useCallback(async (angle?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, rotate: true }));
        try {
            const result = await mcpRobotClient.rotateLeft(angle);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, rotate: false }));
        }
    }, [refreshState]);

    const rotateRight = useCallback(async (angle?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, rotate: true }));
        try {
            const result = await mcpRobotClient.rotateRight(angle);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, rotate: false }));
        }
    }, [refreshState]);

    // Complex motions
    const runInCircle = useCallback(async (radius?: number, duration?: number): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, complex: true }));
        try {
            const result = await mcpRobotClient.runInCircle(radius, duration);
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, complex: false }));
        }
    }, [refreshState]);

    const stopAndStay = useCallback(async (): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, complex: true }));
        try {
            const result = await mcpRobotClient.stopAndStay();
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, complex: false }));
        }
    }, [refreshState]);

    const doFlip = useCallback(async (): Promise<MCPToolResponse> => {
        setLoading(prev => ({ ...prev, complex: true }));
        try {
            const result = await mcpRobotClient.doFlip();
            if (result.success) {
                await refreshState();
            }
            return result;
        } finally {
            setLoading(prev => ({ ...prev, complex: false }));
        }
    }, [refreshState]);

    // Auto-refresh robot state periodically when connected
    useEffect(() => {
        if (!connected) return;

        const interval = setInterval(refreshState, 1000); // Refresh every second
        return () => clearInterval(interval);
    }, [connected, refreshState]);

    return {
        connected,
        connecting,
        robotState,
        connect,
        disconnect,
        refreshState,
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
    };
}
