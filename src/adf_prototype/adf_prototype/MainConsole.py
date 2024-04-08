import rclpy
from rclpy.node import Node


class MainConsole(Node):
    def __init__(self):
        super().__init__('main_console')

    def sub_callback(self, data):
        pass

    def timer_callback(self):
        pass           
        
    def cleanup(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    main_console_node = MainConsole()
    rclpy.spin(main_console_node)
    
    main_console_node.cleanup()
    main_console_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        
                

