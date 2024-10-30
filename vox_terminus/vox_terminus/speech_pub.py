import rclpy
from rclpy.node import Node
import speech_recognition as sr
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.timer = self.create_timer(2.0, self.listen_for_command)

    def listen_for_command(self):
        with self.microphone as source:
            print("Listening for command...")
            audio = self.recognizer.listen(source)
        try:
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f'Heard command: {command}')
            msg = String()
            msg.data = command
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand the command.")
        except sr.RequestError as e:
            self.get_logger().info(f"Could not request results from Google: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
