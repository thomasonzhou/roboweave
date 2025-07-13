/**
 * MCP Client for Robot Control
 * 
 * This service connects to the Robot Control MCP Server running on the backend
 * and provides a React-friendly interface for robot control operations.
 */

export interface RobotState {
    position: {
        x: number;
        y: number;
        z: number;
    };
    orientation: {
        w: number;
        x: number;
        y: number;
        z: number;
    };
    current_mode: string;
    timestamp: number;
}

export interface MCPToolResponse {
    success: boolean;
    result?: string;
    error?: string;
}

export class MCPRobotClient {
    private baseUrl: string;
    private isConnected: boolean = false;

    constructor(baseUrl: string = 'http://localhost:8080') {
        this.baseUrl = baseUrl;
    }

    /**
     * Initialize connection to MCP server
     */
    async connect(): Promise<boolean> {
        try {
            const response = await fetch(`${this.baseUrl}/health`);
            this.isConnected = response.ok;
            return this.isConnected;
        } catch (error) {
            console.error('Failed to connect to MCP server:', error);
            this.isConnected = false;
            return false;
        }
    }

    /**
     * Get current robot state
     */
    async getRobotState(): Promise<RobotState | null> {
        try {
            const response = await fetch(`${this.baseUrl}/robot/state`);
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            const result = await response.json();
            if (result.success && result.data) {
                return result.data;
            }
            return null;
        } catch (error) {
            console.error('Failed to get robot state:', error);
            return null;
        }
    }

    /**
     * Move robot forward
     */
    async moveForward(distance: number = 0.3): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/move/forward', { distance });
    }

    /**
     * Move robot backward
     */
    async moveBackward(distance: number = 0.3): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/move/backward', { distance });
    }

    /**
     * Move robot left
     */
    async moveLeft(distance: number = 0.3): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/move/left', { distance });
    }

    /**
     * Move robot right
     */
    async moveRight(distance: number = 0.3): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/move/right', { distance });
    }

    /**
     * Rotate robot left
     */
    async rotateLeft(angle: number = 45.0): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/rotate/left', { angle_degrees: angle });
    }

    /**
     * Rotate robot right
     */
    async rotateRight(angle: number = 45.0): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/rotate/right', { angle_degrees: angle });
    }

    /**
     * Make robot run in circle
     */
    async runInCircle(radius: number = 2.0, duration: number = 10.0): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/circle', { radius, duration });
    }

    /**
     * Stop robot and make it stay
     */
    async stopAndStay(): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/stop', {});
    }

    /**
     * Make robot do a flip
     */
    async doFlip(): Promise<MCPToolResponse> {
        return this.callRobotAPI('/robot/flip', {});
    }

    /**
     * Generic method to call robot API endpoints
     */
    private async callRobotAPI(endpoint: string, params: Record<string, any>): Promise<MCPToolResponse> {
        try {
            const response = await fetch(`${this.baseUrl}${endpoint}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(params),
            });

            const result = await response.json();

            if (!response.ok) {
                return {
                    success: false,
                    error: result.detail || `HTTP ${response.status}: ${response.statusText}`
                };
            }

            return {
                success: result.success || true,
                result: result.message || result.result || 'Operation completed'
            };
        } catch (error) {
            return {
                success: false,
                error: error instanceof Error ? error.message : 'Unknown error'
            };
        }
    }

    /**
     * Check if client is connected
     */
    get connected(): boolean {
        return this.isConnected;
    }

    /**
     * Disconnect from MCP server
     */
    disconnect(): void {
        this.isConnected = false;
    }
}

// Singleton instance for the app
export const mcpRobotClient = new MCPRobotClient();
