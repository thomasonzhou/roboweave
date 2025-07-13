/**
 * Gemini Live API Service
 * Handles communication with Gemini backend service for processing voice commands and robot control
 */

import type { SpeechRecognitionResult } from './speech-recognition';

export interface GeminiConfig {
  backendUrl?: string;
}

export interface RobotCommand {
  action: 'move_forward' | 'move_backward' | 'move_left' | 'move_right' | 
          'rotate_left' | 'rotate_right' | 'stop' | 'flip' | 'circle';
  parameters?: {
    distance?: number;
    angle?: number;
    radius?: number;
    duration?: number;
  };
  reasoning?: string;
  confidence: number;
}

export interface GeminiResponse {
  commands: RobotCommand[];
  explanation: string;
  timestamp: number;
  processingTime: number;
  sessionId: string;
}

export interface WeaveObservability {
  sessionId: string;
  timestamp: number;
  input: {
    transcript: string;
    confidence: number;
    speechDuration: number;
  };
  processing: {
    model: string;
    systemInstruction: string;
    processingTime: number;
  };
  output: {
    commands: RobotCommand[];
    explanation: string;
    success: boolean;
  };
  metadata: {
    userAgent: string;
    sessionDuration: number;
    commandCount: number;
  };
}

export class GeminiLiveService {
  private backendUrl: string;
  private sessionId: string;
  private sessionStartTime: number;
  private commandCount: number = 0;

  constructor(config: GeminiConfig = {}) {
    this.backendUrl = config.backendUrl || 'http://localhost:8000';
    this.sessionId = this.generateSessionId();
    this.sessionStartTime = Date.now();
  }

  private generateSessionId(): string {
    return `roboweave-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  }

  public async processVoiceCommand(
    speechResult: SpeechRecognitionResult
  ): Promise<GeminiResponse> {
    const startTime = Date.now();
    
    try {
      const response = await this.callBackendService(speechResult);
      const processingTime = Date.now() - startTime;
      
      const geminiResponse: GeminiResponse = {
        commands: response.commands || [],
        explanation: response.explanation || 'Command processed',
        timestamp: Date.now(),
        processingTime,
        sessionId: response.session_id || this.sessionId
      };

      // Track with Weave observability (console logging for now)
      await this.trackWithWeave({
        sessionId: geminiResponse.sessionId,
        timestamp: Date.now(),
        input: {
          transcript: speechResult.transcript,
          confidence: speechResult.confidence,
          speechDuration: 0
        },
        processing: {
          model: 'gemini-1.5-flash',
          systemInstruction: 'Robot voice control system',
          processingTime,
        },
        output: {
          commands: geminiResponse.commands,
          explanation: geminiResponse.explanation,
          success: true
        },
        metadata: {
          userAgent: navigator.userAgent,
          sessionDuration: Date.now() - this.sessionStartTime,
          commandCount: ++this.commandCount
        }
      });

      return geminiResponse;

    } catch (error) {
      console.error('Error processing voice command:', error);
      
      // Track error with Weave
      await this.trackWithWeave({
        sessionId: this.sessionId,
        timestamp: Date.now(),
        input: {
          transcript: speechResult.transcript,
          confidence: speechResult.confidence,
          speechDuration: 0
        },
        processing: {
          model: 'gemini-1.5-flash',
          systemInstruction: 'Robot voice control system',
          processingTime: Date.now() - startTime
        },
        output: {
          commands: [],
          explanation: `Error: ${error instanceof Error ? error.message : 'Unknown error'}`,
          success: false
        },
        metadata: {
          userAgent: navigator.userAgent,
          sessionDuration: Date.now() - this.sessionStartTime,
          commandCount: this.commandCount
        }
      });

      throw error;
    }
  }

  private async callBackendService(speechResult: SpeechRecognitionResult): Promise<any> {
    const url = `${this.backendUrl}/process-voice-command`;
    
    const requestBody = {
      transcript: speechResult.transcript,
      confidence: speechResult.confidence,
      session_id: this.sessionId
    };

    const response = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`Backend service error: ${response.status} ${response.statusText} - ${errorText}`);
    }

    const data = await response.json();
    return data;
  }

  private async trackWithWeave(observabilityData: WeaveObservability): Promise<void> {
    try {
      // For now, just log to console. In production, this would send to W&B Weave
      console.log('🔍 Weave Observability:', {
        session: observabilityData.sessionId,
        command: observabilityData.input.transcript,
        commands: observabilityData.output.commands.length,
        success: observabilityData.output.success,
        processingTime: observabilityData.processing.processingTime + 'ms'
      });

      // TODO: Implement actual Weave tracking
      // This would typically use the Weave SDK or API
      /*
      await weave.track('robot_voice_command', {
        inputs: observabilityData.input,
        outputs: observabilityData.output,
        metadata: observabilityData.metadata
      });
      */
    } catch (error) {
      console.warn('Failed to track with Weave:', error);
    }
  }

  public async checkConnection(): Promise<boolean> {
    try {
      const url = `${this.backendUrl}/health`;
      console.log('🔌 Checking connection to:', url);
      console.log('🔌 Backend URL configured as:', this.backendUrl);
      
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000);
      
      const response = await fetch(url, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
        signal: controller.signal
      });
      
      clearTimeout(timeoutId);
      
      const isHealthy = response.ok;
      console.log('🔌 Connection check result:', isHealthy ? '✅ Connected' : '❌ Failed');
      console.log('🔌 Response status:', response.status, response.statusText);
      console.log('🔌 Response headers:', Object.fromEntries(response.headers.entries()));
      
      if (isHealthy) {
        const healthData = await response.json();
        console.log('🔌 Backend health:', healthData);
      } else {
        const errorText = await response.text();
        console.error('🔌 Health check failed with response:', errorText);
      }
      
      return isHealthy;
    } catch (error) {
      console.error('🔌 Connection check failed:', error);
      if (error instanceof Error) {
        console.error('🔌 Error name:', error.name);
        console.error('🔌 Error message:', error.message);
        
        if (error.name === 'AbortError') {
          console.error('🔌 Request timed out after 5 seconds');
        } else if (error.name === 'TypeError' && error.message.includes('Failed to fetch')) {
          console.error('🔌 Network error - likely CORS or connection refused');
          console.error('🔌 Check if backend is running on:', this.backendUrl);
          console.error('🔌 Check CORS configuration allows your frontend origin');
        }
      }
      return false;
    }
  }

  public getSessionId(): string {
    return this.sessionId;
  }

  public getSessionStats(): { duration: number; commandCount: number } {
    return {
      duration: Date.now() - this.sessionStartTime,
      commandCount: this.commandCount
    };
  }

  public resetSession(): void {
    this.sessionId = this.generateSessionId();
    this.sessionStartTime = Date.now();
    this.commandCount = 0;
  }
}
