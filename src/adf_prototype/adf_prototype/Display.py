import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from RPLCD import CharLCD

class Display(Node):
    def __init__(self):
        super().__init__('display')
        
        self.display_text_sub = self.create_subscription(String,
                                                         "display_text",
                                                         self.display_message_callback,
                                                         10)
        self.lcd_cols = 16
        self.lcd_rows = 2
        self.lcd = CharLCD(cols=self.lcd_cols, 
                           rows=self.lcd_rows, 
                           pin_rs=37, 
                           pin_e=35, 
                           pins_data=[40, 38, 36, 32, 33, 31, 29, 23])
        
        self.curr_text = ""

    def display_message_callback(self, data):
        self.curr_text = data.data

    def update_lcd(self):
        self.lcd.clear()
        self.lcd.write_string(self.curr_text)



def main(args=None):
    rclpy.init(args=args)

    display_node = Display()
    rclpy.spin(display_node)
    
    display_node.cleanup()
    display_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()