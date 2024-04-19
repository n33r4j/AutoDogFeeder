import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Bool
from datetime import datetime, timedelta
from adf_interfaces.msg import LCD16x2



class MainConsole(Node):
    def __init__(self):
        super().__init__('main_console')

        # 2~3 times a day for small dogs (hard coded for now)
        # The timezone on this container is UTC
        self.feeding_times = {"morning": "07:25:00",
                              "afternoon": "07:25:30",
                              "evening": "07:26:00"}
        self.sec_till_next_meal = self.get_clock().now().to_msg().sec
        self.time_till_next_meal = datetime.utcfromtimestamp(self.sec_till_next_meal).strftime("%H:%M:%S")

        self.total_portions = 10
        self.curr_portions_left = 10

        self.is_feeding_time = False
        self.feeding_timer_period = 1 # seconds.
        self.feeding_timer = self.create_timer(self.feeding_timer_period,
                                               self.feeding_timer_callback)

        self.curr_camera_mode = 0 # See Camera node
        self.camera_mode_pub = self.create_publisher(Int8, 'camera_mode', 10)
        self.camera_mode_timer = self.create_timer(1,
                                               self.camera_mode_timer_callback)
        
        self.detector_sub = self.create_subscription(Bool, 'dog_detected', self.detector_callback, 10)
        

        self.curr_feeder_state = 0 # 0 -> open, 1 -> closed
        self.feeder_state_pub = self.create_publisher(Int8, 'feeder_state', 10)
        self.feeder_state_timer = self.create_timer(1,
                                                    self.feeder_state_timer_callback)
        
        self.display_pub = self.create_publisher(LCD16x2, 'display_text', 10)
        self.display_update_timer = self.create_timer(0.1,
                                                     self.display_update_timer_callback)
                                  

    def detector_callback(self, data):
        self.get_logger().info("mc camera mode: %s" % data.data)
        self.curr_camera_mode = 1 if data.data else 0

    def feeder_state_timer_callback(self):
        msg = Int8()
        msg.data = self.curr_feeder_state
        self.feeder_state_pub.publish(msg)

    def camera_mode_timer_callback(self):
        msg = Int8()
        msg.data = self.curr_camera_mode
        self.camera_mode_pub.publish(msg)

    def make_screen(self, screen_type):
        screen = ["",""]
        if screen_type == "START":
            screen = ["AutoDogFeeder v1",\
                      ":)"]

        elif screen_type == "ALERT":
            screen = ["Feeding Time!",\
                      "0^_^0"]

        elif screen_type == "IDLE":
            screen = [f"{self.time_till_next_meal} P:{self.curr_portions_left}/{self.total_portions}",\
                      f"C:{self.curr_camera_mode} D:{self.curr_feeder_state}"] # Camera, Door
        else: #BLANK
            pass

        msg = LCD16x2()
        msg.l1 = "{: ^16}".format(screen[0])
        msg.l2 = "{: ^16}".format(screen[1])
        return msg

    def display_update_timer_callback(self):
        self.display_pub.publish(self.make_screen("IDLE"))

    def feeding_timer_callback(self):
        curr_time = self.get_clock().now().to_msg()
        time_str = datetime.utcfromtimestamp(curr_time.sec).strftime("%H:%M:%S")
        # self.get_logger().info(time_str)
        
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


        
                

