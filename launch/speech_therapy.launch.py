from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for the Misty speech therapy nodes."""
    return LaunchDescription([
        Node(
            package='misty_speech_therapy',
            executable='misty_wrapper',
            name='misty_wrapper',
            parameters=[
                {'misty_ip': '192.168.1.125'}  # Update with actual IP
            ],
            output='screen'
        ),
        Node(
            package='misty_speech_therapy',
            executable='speech_processor',
            name='speech_processor',
            parameters=[
                {'whisper_model_size': 'tiny'},  # Use smaller model size for less resources
                {'ollama_endpoint': 'http://localhost:11434/api/generate'},
                {'ollama_model': 'llama3'}
            ],
            output='screen'
        ),
        Node(
            package='misty_speech_therapy',
            executable='conversation_manager',
            name='conversation_manager',
            output='screen'
        ),
    ])