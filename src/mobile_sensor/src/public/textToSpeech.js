class TextToSpeech {
    constructor() {
        this.synth = window.speechSynthesis;
        this.voices = [];
        this.ws = null;
        this.isReady = false;
        this.autoReconnect = true;
    }

    async initVoices() {
        return new Promise((resolve) => {
            const loadVoices = () => {
                this.voices = this.synth.getVoices();
                if (this.voices.length > 0) {
                    this.isReady = true;
                    console.log("Voices loaded:", this.voices.length);
                    resolve();
                }
            };

            loadVoices(); // Try immediate loading
            
            // Set up event listener if voices aren't loaded yet
            if (this.voices.length === 0) {
                this.synth.onvoiceschanged = () => {
                    loadVoices();
                };
            }
        });
    }

    speak(text) {
        if (!this.synth || !this.isReady) {
            console.error("Speech synthesis not available or not ready");
            return;
        }

        // Cancel any ongoing speech
        this.synth.cancel();

        const utterance = new SpeechSynthesisUtterance(text);
        // Try to find an English voice, fallback to first available
        utterance.voice = this.voices.find(voice => voice.lang.startsWith('en')) || this.voices[0];
        utterance.rate = 1;
        utterance.pitch = 1;
        utterance.volume = 1;
        this.synth.speak(utterance);
    }

    async connectWebSocket() {
        // Wait for voices to be loaded before connecting
        await this.initVoices();
        this.autoReconnect = true;
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        this.ws = new WebSocket(`${protocol}//${host}/tts`);

        this.ws.onopen = () => {
            console.log('TTS WebSocket connected');
            this.speak("TTS system ready"); // Confirmation message
        };

        this.ws.onmessage = (event) => {
            console.log("Received text to speak:", event.data);
            this.speak(event.data);
        };

        this.ws.onerror = (error) => console.error('TTS WebSocket error:', error);
        this.ws.onclose = () => {
            console.log('TTS WebSocket closed');
            if (this.autoReconnect) {
                console.log('Attempting to reconnect...');
                setTimeout(() => this.connectWebSocket(), 1000);
            }
        };
    }

    disconnectWebSocket() {
        this.autoReconnect = false;  // Prevent reconnection attempts
        if (this.ws) {
            this.ws.close();
            this.ws = null;
            this.speak("TTS system disconnected"); // Add disconnect message
        }
    }
}
