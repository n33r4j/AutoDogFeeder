import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero import MotionSensor

class Detector(Node):
    def __init__(self):
        super().__init__('detector')

        self.publisher_ = self.create_publisher(Bool, 'dog_detected', 10)
        self.pir_pin = 5
        self.pir = MotionSensor(self.pir_pin)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        val = self.pir.motion_detected
        self.publisher_.publish(val)

def main(args=None):
    rclpy.init(args=args)

    detector_node = Detector()
    rclpy.spin(detector_node)
    
    detector_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
