import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class SerialServer(Node):
    def __init__(self):
        super().__init__('serial_server')
        
        # Default Value declarations of ros2 params:
        self.declare_parameters(
            namespace='',
            parameters=[
                ('device', '/dev/ttyUSB0'),  # Device we are transmitting to & receiving messages from
                ('led_control_topic', 'led_control_topic'),  # ROS2 topic for LED control
                ('led_on_cmd', 'on'),  # Command to turn the LED ON
                ('led_off_cmd', 'off'),  # Command to turn the LED OFF
            ]
        )

        # Retrieving parameters:
        self.led_control_topic_name = self.get_param_str('led_control_topic')
        self.device_name = self.get_param_str('device')
        self.led_on_cmd = self.get_param_str('led_on_cmd')
        self.led_off_cmd = self.get_param_str('led_off_cmd')

        # Setup the serial connection with the Arduino
        self.ser = serial.Serial(self.device_name, 115200, timeout=.1)
        self.ser.reset_input_buffer()

        # ROS2 subscription to control the LED based on String messages
        self.subscriber = self.create_subscription(
            String,
            self.led_control_topic_name,
            self.serial_listener_callback,
            10
        )
        self.subscriber  # prevent unused variable warning

    def get_param_str(self, name):
        try:
            return self.get_parameter(name).get_parameter_value().string_value
        except:
            pass

    def send_cmd(self, cmd):
        """Send a command to the Arduino to toggle the LED."""
        self.get_logger().info(f"Sending: {cmd}")
        self.ser.write(bytes(cmd, 'utf-8'))

    def recieve_cmd(self):
        """Read and print the response from Arduino (optional debugging)."""
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
            if line:
                self.get_logger().info(f"Received: {line}")
        except:
            pass

    def serial_listener_callback(self, msg):
        """Callback function to handle incoming ROS2 String messages."""
        if msg.data == self.led_on_cmd:
            self.send_cmd(self.led_on_cmd)  # Turn LED ON
            self.recieve_cmd()
        elif msg.data == self.led_off_cmd:
            self.send_cmd(self.led_off_cmd)  # Turn LED OFF
            self.recieve_cmd()
        else:
            self.get_logger().warn(f"Unknown command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    serial_server = SerialServer()
    rclpy.spin(serial_server)

if __name__ == '__main__':
    main()
