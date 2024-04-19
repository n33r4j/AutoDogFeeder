import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

from gpiozero import Device, AngularServo

# For mocking
from gpiozero.pins.mock import MockFactory, MockPWMPin
Device.pin_factory = MockFactory(pin_class=MockPWMPin)


class Feeder(Node):
    def __init__(self):
        super().__init__('feeder')

        self.open_angle = -90
        self.close_angle = 0
        self.curr_door_angle = self.close_angle
        self.needsUpdate = False

        self.curr_door_target_angle = self.close_angle
        self.pin = 17

        self.servo_1 = AngularServo(self.pin, 
                                    min_angle=self.close_angle, 
                                    max_angle=self.open_angle)

        self.door_state = 0 # 0 -> closed, 1 -> open

        self.subscription_ = self.create_subscription(Int8, 'feeder_state', self.feeder_callback, 10)
        
        self.door_update_timer = self.create_timer(0.5, self.door_update_callback)

    def feeder_callback(self, data):
        if self.door_state != data.data:            
            self.door_state = data.data
            if self.door_state == 0:
                self.close_door()
            elif self.door_state == 1:
                self.open_door()

    def close_door(self):
        self.curr_door_target_angle = self.close_angle

    def open_door(self):
        self.curr_door_target_angle = self.open_angle

    def door_update_callback(self):
        if self.curr_door_angle != self.curr_door_target_angle:
            self.servo_1.angle = self.curr_door_angle
            self.curr_door_angle = self.curr_door_target_angle
            message = "Opening" if self.curr_door_target_angle == self.open_angle else "Closing" 
            self.get_logger().info("%s door (pos: %d)" % (message, self.curr_door_target_angle))

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
