import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Twist
import requests
import time
import base64
import tempfile
import os

class MistyROS2Wrapper(Node):
    def __init__(self):
        super().__init__('misty_wrapper')
        # define Misty's API base URL
        self.misty_ip = '192.168.1.125'  # Change accordingly
        self.base_url = f'http://{self.misty_ip}/api'
        
        # publishers
        self.battery_pub = self.create_publisher(Float32, 'misty/battery', 10)
        self.speech_pub = self.create_publisher(String, 'misty/speech_recognition', 10)
        self.audio_pub = self.create_publisher(String, 'misty/audio_data', 10)
        self.recording_status_pub = self.create_publisher(Bool, 'misty/recording_status', 10)
        
        # subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, 'misty/speak', self.speak_callback, 10)
        self.create_subscription(String, 'misty/led', self.led_callback, 10)
        self.create_subscription(Bool, 'misty/start_recording', self.start_recording_callback, 10)
        self.create_subscription(Bool, 'misty/stop_recording', self.stop_recording_callback, 10)
        
        # timers
        self.create_timer(10.0, self.publish_battery_status)
        
        # Audio recording parameters
        self.is_recording = False
        self.temp_dir = tempfile.gettempdir()
        self.get_logger().info(f'Using temp directory: {self.temp_dir}')

    def cmd_vel_callback(self, msg):
        # twist msg for misty api
        data = {"LinearVelocity": msg.linear.x, "AngularVelocity": msg.angular.z}
        response = requests.post(f'{self.base_url}/drive', json=data)
        self.get_logger().info(f'Moving Misty: {data}, Response: {response.status_code}')

    def speak_callback(self, msg):
        data = {"Text": msg.data, "Flush": True}
        response = requests.post(f'{self.base_url}/tts/speak', json=data)
        self.get_logger().info(f'Misty speaking: {msg.data}, Response: {response.status_code}')

    def led_callback(self, msg):
        # Parse the RGB values from the message (format: "R,G,B")
        try:
            rgb_values = [int(val) for val in msg.data.split(',')]
            if len(rgb_values) == 3:
                data = {"red": rgb_values[0], "green": rgb_values[1], "blue": rgb_values[2]}
                response = requests.post(f'{self.base_url}/led', json=data)
                self.get_logger().info(f'Misty LED set to: {rgb_values}, Response: {response.status_code}')
            else:
                self.get_logger().warn('Invalid LED format. Expected "R,G,B"')
        except ValueError:
            self.get_logger().warn('Invalid LED values. Expected integers "R,G,B"')

    def publish_battery_status(self):
        response = requests.get(f'{self.base_url}/battery')
        if response.status_code == 200:
            battery_level = response.json().get("result", {}).get("chargePercent", 0.0)
            msg = Float32()
            msg.data = float(battery_level)
            self.battery_pub.publish(msg)
            self.get_logger().info(f'Battery: {battery_level}%')
        else:
            self.get_logger().warn(f'Failed to retrieve battery status. Status code: {response.status_code}')

    def start_recording_callback(self, msg):
        if msg.data and not self.is_recording:
            self.start_recording()
    
    def stop_recording_callback(self, msg):
        if msg.data and self.is_recording:
            self.stop_recording()
    
    def start_recording(self):
        """Start audio recording on Misty"""
        data = {"FileName": "speech_recording.wav", "MaximumDuration": 30000, "SilenceTimeout": 5000}
        response = requests.post(f'{self.base_url}/audio/recording/start', json=data)
        
        if response.status_code == 200:
            self.is_recording = True
            self.get_logger().info('Started recording audio on Misty')
            
            # Publish recording status
            status_msg = Bool()
            status_msg.data = True
            self.recording_status_pub.publish(status_msg)
            
            # LED cue for recording state (blue)
            led_data = {"red": 0, "green": 0, "blue": 255}
            requests.post(f'{self.base_url}/led', json=led_data)
        else:
            self.get_logger().error(f'Failed to start recording. Status code: {response.status_code}, Response: {response.text}')
    
    def stop_recording(self):
        """Stop audio recording and process the recorded audio"""
        if not self.is_recording:
            return
            
        response = requests.post(f'{self.base_url}/audio/recording/stop')
        if response.status_code == 200:
            self.is_recording = False
            self.get_logger().info('Stopped recording audio on Misty')
            
            # LED cue for processing state (purple)
            led_data = {"red": 128, "green": 0, "blue": 128}
            requests.post(f'{self.base_url}/led', json=led_data)
            
            # Publish recording status
            status_msg = Bool()
            status_msg.data = False
            self.recording_status_pub.publish(status_msg)
            
            # Get the recorded audio file
            self.get_recorded_audio("speech_recording.wav")
        else:
            self.get_logger().error(f'Failed to stop recording. Status code: {response.status_code}, Response: {response.text}')
    
    def get_recorded_audio(self, filename):
        """Get the recorded audio file from Misty"""
        response = requests.get(f'{self.base_url}/audio/recording?FileName={filename}&Base64=true')
        
        if response.status_code == 200:
            audio_data = response.json().get("result", {}).get("base64", "")
            if audio_data:
                # Save audio file temporarily
                audio_path = os.path.join(self.temp_dir, filename)
                with open(audio_path, "wb") as audio_file:
                    audio_file.write(base64.b64decode(audio_data))
                self.get_logger().info(f'Saved audio to {audio_path}')
                
                # Publish audio file path for processing
                msg = String()
                msg.data = audio_path
                self.audio_pub.publish(msg)
                
                # LED back to normal (green)
                led_data = {"red": 0, "green": 255, "blue": 0}
                requests.post(f'{self.base_url}/led', json=led_data)
            else:
                self.get_logger().error('No audio data received')
        else:
            self.get_logger().error(f'Failed to get recorded audio. Status code: {response.status_code}, Response: {response.text}')

def main(args=None):
    rclpy.init(args=args)
    node = MistyROS2Wrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()