/**
 * Speech Recognition Service
 * Handles voice input using the Web Speech API
 */

export interface SpeechRecognitionResult {
  transcript: string;
  confidence: number;
  isFinal: boolean;
  timestamp: number;
}

export interface SpeechRecognitionConfig {
  language?: string;
  continuous?: boolean;
  interimResults?: boolean;
  maxAlternatives?: number;
}

export class SpeechRecognitionService {
  private recognition: SpeechRecognition | null = null;
  private isSupported: boolean = false;
  private isListening: boolean = false;
  private onResultCallback?: (result: SpeechRecognitionResult) => void;
  private onErrorCallback?: (error: string) => void;
  private onStatusCallback?: (status: 'listening' | 'stopped' | 'error') => void;

  constructor() {
    this.checkSupport();
    this.initializeRecognition();
  }

  private checkSupport(): void {
    // Check for Web Speech API support
    const hasWebkitSpeechRecognition = 'webkitSpeechRecognition' in window;
    const hasSpeechRecognition = 'SpeechRecognition' in window;
    const hasUserMedia = 'mediaDevices' in navigator && 'getUserMedia' in navigator.mediaDevices;
    
    this.isSupported = (hasWebkitSpeechRecognition || hasSpeechRecognition) && hasUserMedia;
    
    console.log('Speech recognition support check:', {
      webkitSpeechRecognition: hasWebkitSpeechRecognition,
      speechRecognition: hasSpeechRecognition,
      userMedia: hasUserMedia,
      isSupported: this.isSupported,
      userAgent: navigator.userAgent
    });
  }

  private initializeRecognition(): void {
    if (!this.isSupported) {
      console.warn('Speech recognition not supported in this browser');
      return;
    }

    // Initialize with webkit prefix for Chrome/Safari compatibility
    const SpeechRecognition = window.SpeechRecognition || (window as any).webkitSpeechRecognition;
    this.recognition = new SpeechRecognition();

    if (!this.recognition) return;

    // Configure recognition
    this.recognition.continuous = true;
    this.recognition.interimResults = true;
    this.recognition.lang = 'en-US';
    this.recognition.maxAlternatives = 1;

    // Set up event handlers
    this.recognition.onstart = () => {
      this.isListening = true;
      this.onStatusCallback?.('listening');
      console.log('Speech recognition started');
    };

    this.recognition.onend = () => {
      this.isListening = false;
      this.onStatusCallback?.('stopped');
      console.log('Speech recognition ended');
    };

    this.recognition.onresult = (event: SpeechRecognitionEvent) => {
      let interimTranscript = '';
      let finalTranscript = '';

      for (let i = event.resultIndex; i < event.results.length; i++) {
        const result = event.results[i];
        const transcript = result[0].transcript;

        if (result.isFinal) {
          finalTranscript += transcript + ' ';
        } else {
          interimTranscript += transcript;
        }
      }

      if (finalTranscript || interimTranscript) {
        const transcript = finalTranscript || interimTranscript;
        const isFinal = !!finalTranscript;
        const confidence = event.results[event.resultIndex]?.[0]?.confidence || 0;

        this.onResultCallback?.({
          transcript: transcript.trim(),
          confidence,
          isFinal,
          timestamp: Date.now()
        });
      }
    };

    this.recognition.onerror = (event: SpeechRecognitionErrorEvent) => {
      console.error('Speech recognition error:', event.error);
      this.onErrorCallback?.(event.error);
      this.onStatusCallback?.('error');
    };
  }

  public configure(config: SpeechRecognitionConfig): void {
    if (!this.recognition) return;

    if (config.language) this.recognition.lang = config.language;
    if (config.continuous !== undefined) this.recognition.continuous = config.continuous;
    if (config.interimResults !== undefined) this.recognition.interimResults = config.interimResults;
    if (config.maxAlternatives !== undefined) this.recognition.maxAlternatives = config.maxAlternatives;
  }

  public startListening(): void {
    if (!this.isSupported || !this.recognition || this.isListening) {
      console.warn('Cannot start listening:', {
        isSupported: this.isSupported,
        hasRecognition: !!this.recognition,
        isListening: this.isListening
      });
      return;
    }

    // Request microphone permission first
    navigator.mediaDevices?.getUserMedia({ audio: true })
      .then(() => {
        console.log('Microphone permission granted');
        try {
          this.recognition!.start();
          console.log('Speech recognition start() called');
        } catch (error) {
          console.error('Failed to start speech recognition:', error);
          this.onErrorCallback?.('Failed to start listening');
        }
      })
      .catch((error) => {
        console.error('Microphone permission denied:', error);
        this.onErrorCallback?.('Microphone permission required for voice commands');
      });
  }

  public stopListening(): void {
    if (!this.recognition || !this.isListening) return;

    try {
      this.recognition.stop();
    } catch (error) {
      console.error('Failed to stop speech recognition:', error);
    }
  }

  public onResult(callback: (result: SpeechRecognitionResult) => void): void {
    this.onResultCallback = callback;
  }

  public onError(callback: (error: string) => void): void {
    this.onErrorCallback = callback;
  }

  public onStatusChange(callback: (status: 'listening' | 'stopped' | 'error') => void): void {
    this.onStatusCallback = callback;
  }

  public getIsSupported(): boolean {
    return this.isSupported;
  }

  public getIsListening(): boolean {
    return this.isListening;
  }

  public destroy(): void {
    if (this.recognition) {
      this.stopListening();
      this.recognition = null;
    }
    this.onResultCallback = undefined;
    this.onErrorCallback = undefined;
    this.onStatusCallback = undefined;
  }
}

// Global speech recognition service instance
export const speechRecognitionService = new SpeechRecognitionService();
