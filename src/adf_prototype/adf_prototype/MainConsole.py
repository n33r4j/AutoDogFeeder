import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool
from datetime import datetime



class MainConsole(Node):
    def __init__(self):
        super().__init__('main_console')

        # 2~3 times a day for small dogs (hard coded for now)
        # The timezone on this container is UTC
        self.feeding_times = {"morning": "07:25:00",
                              "afternoon": "07:25:30",
                              "evening": "07:26:00"}
        self.is_feeding_time = False
        self.feeding_timer_period = 1 # seconds.
        self.feeding_timer = self.create_timer(self.feeding_timer_period,
                                               self.feeding_timer_callback)

        self.curr_camera_mode = 0 # See Camera node
        self.camera_mode_pub = self.create_publisher(Int8, 'camera_mode', 10)
        self.camera_mode_timer = self.create_timer(0.5,
                                               self.camera_mode_timer_callback)
        
        self.detector_sub = self.create_subscription(Bool, 'dog_detected', self.detector_callback, 10)


    def detector_callback(self, data):
        self.get_logger().info("mc camera mode: %s" % data.data)
        self.curr_camera_mode = 1 if data.data else 0

    def camera_mode_timer_callback(self):
        msg = Int8()
        msg.data = self.curr_camera_mode
        self.camera_mode_pub.publish(msg)

    def feeding_timer_callback(self):
        curr_time = self.get_clock().now().to_msg()
        time_str = datetime.utcfromtimestamp(curr_time.sec).strftime("%H:%M:%S")
        self.get_logger().info(time_str)
        
        # Better to make this check that curr_time >= feeding time [i]
        if time_str in self.feeding_times.values():
            self.get_logger().info("It's feeding time!")
            self.is_feeding_time = True # Will be reset by Feeder node.
        
    def cleanup(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    main_console_node = MainConsole()
    main_console_node.get_logger().info("Starting main console node...")
    rclpy.spin(main_console_node)
    
    main_console_node.cleanup()
    main_console_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        
                

