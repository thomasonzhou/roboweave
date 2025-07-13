/**
 * Gemini Live API Service
 * Handles communication with Gemini for processing voice commands and robot control
 */

import type { SpeechRecognitionResult } from './speech-recognition';

export interface GeminiConfig {
  apiKey: string;
  model?: string;
  systemInstruction?: string;
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
    tokens?: {
      input: number;
      output: number;
    };
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

const DEFAULT_SYSTEM_INSTRUCTION = `You are RoboWeave Voice Controller, an AI assistant that converts natural language voice commands into structured robot control actions.

ROBOT CAPABILITIES:
- Movement: forward, backward, left, right (distance in meters, 0.1-2.0)
- Rotation: left, right (angle in degrees, 15-180)
- Actions: stop, flip, circle motion (radius 0.5-3.0m, duration 5-30s)

RESPONSE FORMAT (JSON only):
{
  "commands": [
    {
      "action": "move_forward",
      "parameters": {"distance": 0.5},
      "reasoning": "User requested forward movement",
      "confidence": 0.95
    }
  ],
  "explanation": "Moving the robot forward by 0.5 meters as requested."
}

GUIDELINES:
- Always respond with valid JSON
- Use safe parameters (small distances/angles for safety)
- Provide clear reasoning for each command
- If unclear, ask for clarification
- Support compound commands: "move forward then turn left"
- Default values: distance=0.3m, angle=45°, radius=2m, duration=10s

EXAMPLES:
"Move forward" → move_forward, distance: 0.3
"Turn around" → rotate_left, angle: 180
"Go backward a bit" → move_backward, distance: 0.2
"Do a flip" → flip
"Move in a circle" → circle, radius: 2, duration: 10
"Stop the robot" → stop`;

export class GeminiLiveService {
  private apiKey: string;
  private model: string;
  private systemInstruction: string;
  private sessionId: string;
  private sessionStartTime: number;
  private commandCount: number = 0;
  private baseUrl = 'https://generativelanguage.googleapis.com/v1beta';

  constructor(config: GeminiConfig) {
    this.apiKey = config.apiKey;
    this.model = config.model || 'gemini-1.5-flash';
    this.systemInstruction = config.systemInstruction || DEFAULT_SYSTEM_INSTRUCTION;
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
      const response = await this.callGeminiAPI(speechResult.transcript);
      const processingTime = Date.now() - startTime;
      
      const geminiResponse: GeminiResponse = {
        commands: response.commands || [],
        explanation: response.explanation || 'Command processed',
        timestamp: Date.now(),
        processingTime
      };

      // Track with Weave observability
      await this.trackWithWeave({
        sessionId: this.sessionId,
        timestamp: Date.now(),
        input: {
          transcript: speechResult.transcript,
          confidence: speechResult.confidence,
          speechDuration: 0 // Would need to track from speech start
        },
        processing: {
          model: this.model,
          systemInstruction: this.systemInstruction,
          processingTime,
          tokens: response.tokens
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
          model: this.model,
          systemInstruction: this.systemInstruction,
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

  private async callGeminiAPI(transcript: string): Promise<any> {
    const url = `${this.baseUrl}/models/${this.model}:generateContent`;
    
    const requestBody = {
      contents: [
        {
          parts: [
            {
              text: `${this.systemInstruction}\n\nUser voice command: "${transcript}"\n\nProvide a JSON response with robot commands.`
            }
          ]
        }
      ],
      generationConfig: {
        temperature: 0.1,
        maxOutputTokens: 1000,
        responseSchema: {
          type: 'object',
          properties: {
            commands: {
              type: 'array',
              items: {
                type: 'object',
                properties: {
                  action: { type: 'string' },
                  parameters: { type: 'object' },
                  reasoning: { type: 'string' },
                  confidence: { type: 'number' }
                }
              }
            },
            explanation: { type: 'string' }
          }
        }
      }
    };

    const response = await fetch(`${url}?key=${this.apiKey}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      throw new Error(`Gemini API error: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();
    
    if (!data.candidates?.[0]?.content?.parts?.[0]?.text) {
      throw new Error('Invalid response from Gemini API');
    }

    const responseText = data.candidates[0].content.parts[0].text;
    
    try {
      return JSON.parse(responseText);
    } catch (parseError) {
      console.error('Failed to parse Gemini response:', responseText);
      throw new Error('Invalid JSON response from Gemini');
    }
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
