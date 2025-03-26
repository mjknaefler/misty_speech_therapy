import sys
import os

local_bin = os.path.expanduser('~/.local/bin')
if os.path.exists(local_bin) and local_bin not in sys.path:
    sys.path.append(local_bin)
    os.environ['PATH'] = local_bin + ':' + os.environ.get('PATH', '')

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import requests
import json
import time
import tempfile

try:
    import whisper
except ImportError:
    print("Whisper module not found. Please install it with: pip install openai-whisper")
    whisper = None

class SpeechProcessor(Node):
    def __init__(self):
        super().__init__('speech_processor')
        
        # Publishers
        self.text_pub = self.create_publisher(String, 'processed/text', 10)
        self.llm_response_pub = self.create_publisher(String, 'llm/response', 10)
        
        # Subscribers
        self.create_subscription(String, 'misty/audio_data', self.process_audio_callback, 10)
        self.create_subscription(String, 'processed/text', self.process_text_callback, 10)
        
        # Parameters
        self.declare_parameter('whisper_model_size', 'base')
        self.declare_parameter('ollama_endpoint', 'http://localhost:11434/api/generate')
        self.declare_parameter('ollama_model', 'llama3')
        
        # Load whisper model
        model_size = self.get_parameter('whisper_model_size').value
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        try:
            if whisper is None:
                self.get_logger().error('Whisper module not properly loaded')
                self.whisper_model = None
            else:
                self.whisper_model = whisper.load_model(model_size)
                self.get_logger().info(f'Whisper model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Error loading Whisper model: {str(e)}')
            self.whisper_model = None
        
        # Ollama parameters
        self.ollama_endpoint = self.get_parameter('ollama_endpoint').value
        self.ollama_model = self.get_parameter('ollama_model').value
        
        self.get_logger().info('Speech processor initialized')
        
    def process_audio_callback(self, msg):
        """Process audio file and convert to text"""
        audio_path = msg.data
        self.get_logger().info(f'Processing audio from: {audio_path}')
        
        if self.whisper_model is None:
            self.get_logger().error('Cannot transcribe audio: Whisper model not loaded')
            # Provide a fallback response
            text_msg = String()
            text_msg.data = "I heard you speaking but couldn't understand the words."
            self.text_pub.publish(text_msg)
            return
        
        try:
            # Check if ffmpeg is available
            import subprocess
            try:
                subprocess.run(['ffmpeg', '-version'], capture_output=True, check=True)
            except (subprocess.SubprocessError, FileNotFoundError):
                self.get_logger().error('ffmpeg not found. Please install it with: sudo apt install ffmpeg')
                text_msg = String()
                text_msg.data = "I need ffmpeg to understand you. Please install it."
                self.text_pub.publish(text_msg)
                return
                
            # Transcribe using Whisper
            result = self.whisper_model.transcribe(audio_path)
            transcribed_text = result["text"].strip()
            
            self.get_logger().info(f'Transcribed text: {transcribed_text}')
            
            # Publish transcribed text
            text_msg = String()
            text_msg.data = transcribed_text
            self.text_pub.publish(text_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')
            # Provide a fallback response
            text_msg = String()
            text_msg.data = "I heard you speaking but couldn't process what you said."
            self.text_pub.publish(text_msg)
        
        # Clean up temporary file regardless of success or failure
        try:
            if os.path.exists(audio_path) and tempfile.gettempdir() in audio_path:
                os.remove(audio_path)
                self.get_logger().info(f'Removed temporary file: {audio_path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to clean up temp file: {str(e)}')
    def process_text_callback(self, msg):
        """Process text through LLM"""
        text = msg.data
        self.get_logger().info(f'Sending text to LLM: {text}')
        
        try:
            # Prepare prompt for speech therapy context
            prompt = self.create_therapy_prompt(text)
            
            # Send to Ollama
            response = self.send_to_ollama(prompt)
            
            if response:
                # Publish LLM response
                response_msg = String()
                response_msg.data = response
                self.llm_response_pub.publish(response_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing with LLM: {str(e)}')
    
    def create_therapy_prompt(self, text):
        """Create a prompt for the LLM appropriate for speech therapy"""
        return f"""
        You are Misty, a friendly robot helping a child with speech therapy. 
        The child has just said: "{text}"
        
        Please respond in a friendly, encouraging way. Keep your response brief (1-2 sentences).
        Be very patient and speak clearly. Don't correct pronunciation directly but model good speech.
        If the child introduced themselves, remember their name and use it.
        Ask simple follow-up questions to keep the conversation going.
        
        Your response:
        """
    
    def send_to_ollama(self, prompt):
        """Send text to Ollama and get response"""
        payload = {
            "model": self.ollama_model,
            "prompt": prompt,
            "stream": False
        }
        
        try:
            response = requests.post(self.ollama_endpoint, json=payload)
            if response.status_code == 200:
                result = response.json()
                llm_response = result.get("response", "").strip()
                return llm_response
            else:
                self.get_logger().error(f'Error from Ollama API: {response.status_code}, {response.text}')
                return "I'm thinking about what you said."
        except Exception as e:
            self.get_logger().error(f'Exception calling Ollama: {str(e)}')
            return "I'm having trouble thinking right now."

def main(args=None):
    rclpy.init(args=args)
    node = SpeechProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()