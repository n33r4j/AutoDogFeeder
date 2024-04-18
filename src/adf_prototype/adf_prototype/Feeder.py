import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class Feeder(Node):
    def __init__(self):
        super().__init__('feeder')

        self.open_angle = -90
        self.close_angle = 0
        self.curr_door_angle = self.close_angle

        self.door_state = 0 # 0 -> closed, 1 -> open

        self.subscription_ = self.create_subscription(Int8, 'feeder_state', self.feeder_callback, 10)


    def feeder_callback(self, data):
        if self.door_state != data.data:
            
            self.door_state = data.data

    def close_door(self):
        self.curr_door_angle = self.close_angle

    def open_door(self):
        self.curr_door_angle = self.open_angle

    def timer_callback(self, data):
        pass

    def cleanup(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    feeder_node = Feeder()
    feeder_node.get_logger().info("Starting feeder node...")
    rclpy.spin(feeder_node)

    feeder_node.cleanup()
    feeder_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
