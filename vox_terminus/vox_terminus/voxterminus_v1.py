import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import google.generativeai as genai  # Import google.generativeai for generative AI capabilities
from gtts import gTTS
from io import BytesIO
import pygame
import re  # Import regular expressions

class LLMCommandNode(Node):
    def __init__(self):
        super().__init__('llm_command_node')
        self.subscription = self.create_subscription(String, 'voice_commands', self.handle_command, 10)
        
        # Set up Google Generative AI API
        os.environ['GOOGLE_API_KEY'] = 'AIzaSyCN7WugKz_m_gB85tWgEfTg8EJc84Gx0ho'  # Replace with your actual API key
        genai.configure(api_key=os.environ['GOOGLE_API_KEY'])
        
        # Initialize the model
        self.model = genai.GenerativeModel('gemini-pro')
    
    def handle_command(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f"Processing command: {command}")
        
        interpreted_command = self.send_command_to_bard(command)
        
        if interpreted_command:
            self.execute_command(interpreted_command)

    def send_command_to_bard(self, message):
        try:
            # Generate response using Generative AI
            query = f'''Run the Following General Linux or ROS2 command if it says run it is usually ros2 command response should only contain the command no further explanation also fix the command if it doesnt fill real as speech recognition can fail many times
            
            i am writing some basic ros2 / linux command based upon those and your previous knowledge provide appropriate response
            
            "ros2 topic list": "ros2 topic list",
    "ros2 topic echo": "ros2 topic echo /topic_name",
    "ros2 topic pub": "ros2 topic pub /topic_name std_msgs/String 'data: Hello World'",
    "ros2 topic info": "ros2 topic info /topic_name",
    "ros2 service list": "ros2 service list",
    "ros2 service call": "ros2 service call /service_name std_srvs/srv/Empty",
    "ros2 service type": "ros2 service type /service_name",
    "ros2 service find": "ros2 service find std_srvs/srv/Empty",
    "ros2 node list": "ros2 node list",
    "ros2 node info": "ros2 node info /node_name",
    "ros2 node list parameters": "ros2 param list",
    "ros2 param get": "ros2 param get /node_name param_name",
    "ros2 param set": "ros2 param set /node_name param_name value",
    "ros2 action list": "ros2 action list",
    "ros2 action info": "ros2 action info /action_name",
    "ros2 launch": "ros2 launch package_name launch_file.py",
    "ros2 bag record": "ros2 bag record /topic_name",
    "ros2 bag play": "ros2 bag play bag_file.bag",
    "ros2 run": "ros2 run package_name executable_name",
    "ros2 lifecycle set": "ros2 lifecycle set /node_name transition",
    "ros2 doctor": "ros2 doctor"
    "open rqt":"rqt"
            Here's the for which you have to give response Also Fix the Command If its wrong
            : {message}'''
            response = self.model.generate_content(query)

            if response and response.text:
                self.get_logger().info(f"Google Generative AI response: {response.text}")
                
                # Clean the response to extract the command only
                command =response.text
                
                if command:
                    # Speak the response (optional)
                    # self.speak(command)
                    return command
                else:
                    self.get_logger().error("Failed to extract a valid command from the response.")
                    return None
            else:
                self.get_logger().error("Received an empty response from Google Generative AI.")
                return None
        except Exception as e:
            self.get_logger().error(f"Error generating response using Google Generative AI: {e}")
            return None

    def extract_command(self, text):
        """
        Extracts a valid command from the text using regular expressions.
        This method ensures that the command does not contain extra text.
        """
        # Use a regular expression to find a line that starts with "ros2" or a known command prefix
        match = re.search(r'^\s*(ros2\s+[^\n]+)', text, re.MULTILINE)
        
        if match:
            command = match.group(1).strip()
            return command
        else:
            return None

    def execute_command(self, interpreted_command):
        try:
            # Open a new terminal and run the interpreted command
            terminal_command = f'gnome-terminal -- bash -c "{interpreted_command}; exec bash"'
            
            self.get_logger().info(f'Executing in new terminal: {terminal_command}')
            
            # Open a new terminal and execute the command
            process = subprocess.Popen(terminal_command, shell=True)
            
            # This will allow multiple commands to run simultaneously
        except Exception as e:
            self.get_logger().error(f"Failed to execute command: {e}")

    def speak(self, text):
        try:
            audio_stream = BytesIO()
            tts = gTTS(text=text, lang='en')
            tts.write_to_fp(audio_stream)
            audio_stream.seek(0)

            pygame.mixer.init()
            pygame.mixer.music.load(audio_stream)
            pygame.mixer.music.play()

            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
        except Exception as e:
            self.get_logger().error(f"Failed to play audio: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
