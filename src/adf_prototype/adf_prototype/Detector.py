import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero import Device, MotionSensor

# For mocking
from gpiozero.pins.mock import MockFactory
Device.pin_factory = MockFactory()

class Detector(Node):
    """
    For now, just switches between detected and not detected every 3 seconds.
    """
    def __init__(self):
        super().__init__('detector')

        self.publisher_ = self.create_publisher(Bool, 'dog_detected', 10)
        
        # REAL HARDWARE
        self.pir_pin = 5
        self.pir = MotionSensor(self.pir_pin)
        
        self.check_timer = self.create_timer(0.1, self.check_timer_callback)
        
        self.mock_toggle_timer = self.create_timer(3, self.mock_toggle_timer_callback)
        self.mock_toggle = False

    def mock_toggle_timer_callback(self):
        if not self.mock_toggle:
            self.pir.pin.drive_high()
        else:
            self.pir.pin.drive_low()
        self.mock_toggle = not self.mock_toggle
        # TODO: maybe make detection based on keyboard button press
        # See https://github.com/rohbotics/ros2_teleop_keyboard/blob/master/teleop_twist_keyboard.py

    def check_timer_callback(self):
        val = self.pir.motion_detected
        msg = Bool()
        msg.data = val
        self.publisher_.publish(msg)
        # self.get_logger().info("pir: %s" % (val))

def main(args=None):
    rclpy.init(args=args)

    detector_node = Detector()
    detector_node.get_logger().info("Starting detector node...")
    rclpy.spin(detector_node)
    
    detector_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
