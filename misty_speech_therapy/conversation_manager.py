import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class ConversationManager(Node):
    def __init__(self):
        super().__init__('conversation_manager')
        
        # Publishers
        self.speak_pub = self.create_publisher(String, 'misty/speak', 10)
        self.led_pub = self.create_publisher(String, 'misty/led', 10)
        self.start_recording_pub = self.create_publisher(Bool, 'misty/start_recording', 10)
        self.stop_recording_pub = self.create_publisher(Bool, 'misty/stop_recording', 10)
        
        # Subscribers
        self.create_subscription(String, 'llm/response', self.llm_response_callback, 10)
        self.create_subscription(Bool, 'misty/recording_status', self.recording_status_callback, 10)
        
        # State variables
        self.child_name = None
        self.conversation_started = False
        self.is_recording = False
        self.waiting_for_response = False
        self.introduction_complete = False
        self.timer = None
        
        # Timer for starting conversation
        self.create_timer(5.0, self.start_conversation_callback)
        
        self.get_logger().info('Conversation manager initialized')
        
    def start_conversation_callback(self):
        """Start the conversation if not already started"""
        if not self.conversation_started:
            self.conversation_started = True
            self.introduction()
            # We can't destroy the timer in Foxy, but we can prevent it from doing anything else
            self.conversation_started = True  # This will prevent timer from triggering action again
    
    def introduction(self):
        """Have Misty introduce herself and ask for the child's name"""
        intro_message = "Hello! My name is Misty. I'm a friendly robot. What's your name?"
        self.speak(intro_message)
        # Set happy LED color (yellow)
        self.set_led("255,255,0")
    
    def speak(self, text):
        """Have Misty speak the given text"""
        msg = String()
        msg.data = text
        self.speak_pub.publish(msg)
        # Calculate approximate speaking time (5 chars per second + 1.5 second buffer)
        speak_time = (len(text) / 5) + 1.5
        self.get_logger().info(f'Speaking: "{text}" (estimated {speak_time:.1f}s)')
        
        # Start recording after Misty finishes speaking
        # Store the timer so we can cancel it if needed
        self.timer = self.create_timer(speak_time, self.start_recording_timer_callback)
    
    def start_recording_timer_callback(self):
        """Start recording after speaking is done"""
        if not self.is_recording and not self.waiting_for_response:
            self.start_recording()
            self.waiting_for_response = True
        
        # Cancel this timer so it only fires once
        self.timer.cancel()
    
    def start_recording(self):
        """Send command to start recording"""
        msg = Bool()
        msg.data = True
        self.start_recording_pub.publish(msg)
        
        # Set timeout for recording (15 seconds)
        self.recording_timer = self.create_timer(15.0, self.recording_timeout_callback)
    
    def recording_timeout_callback(self):
        """Stop recording if it's been going on too long"""
        if self.is_recording:
            self.stop_recording()
        
        # Cancel the timer
        self.recording_timer.cancel()
    
    def stop_recording(self):
        """Send command to stop recording"""
        msg = Bool()
        msg.data = True
        self.stop_recording_pub.publish(msg)
    
    def set_led(self, color):
        """Set LED color (format: 'R,G,B')"""
        msg = String()
        msg.data = color
        self.led_pub.publish(msg)
    
    def recording_status_callback(self, msg):
        """Handle recording status updates"""
        self.is_recording = msg.data
        if not self.is_recording and self.waiting_for_response:
            # Recording stopped, now we wait for text processing
            self.waiting_for_response = False
            # Set thinking LED color (purple)
            self.set_led("128,0,128")
    
    def llm_response_callback(self, msg):
        """Handle LLM response and continue conversation"""
        response = msg.data
        
        # Extract name from first interaction if not already known
        if not self.introduction_complete and not self.child_name:
            # Simple name extraction - find first word that could be a name
            words = response.split()
            for word in words:
                if word.lower() not in ["hi", "hello", "i'm", "am", "my", "name", "is", "i", "a", "am", "the"]:
                    self.child_name = word.strip(".,!?")
                    break
            
            self.introduction_complete = True
            
        # Speak the response
        self.speak(response)
        
        # Set happy LED color (green)
        self.set_led("0,255,0")

def main(args=None):
    rclpy.init(args=args)
    node = ConversationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()