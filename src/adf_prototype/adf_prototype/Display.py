import rclpy
from rclpy.node import Node
from adf_interfaces.msg import LCD16x2

## Only on PI
# from RPLCD import CharLCD
# from RPi.GPIO import GPIO


C_UP_R = "\033[F" # – move cursor to the beginning of the previous line
C_UP = "\033[A" # – move cursor up one line

MOCK = True

class Display(Node):
    def __init__(self):
        super().__init__('display')
        
        self.display_text_sub = self.create_subscription(LCD16x2,
                                                         "display_text",
                                                         self.display_message_callback,
                                                         10)
        
        self.update_interval = 0.5 # sec
        self.display_update_timer = self.create_timer(self.update_interval, 
                                                        self.update_lcd)

        self.lcd_cols = 16
        self.lcd_rows = 2

        self.isFirstTime = True
        
        # self.lcd = CharLCD( numbering_mode=GPIO.BOARD,
        #                     cols=self.lcd_cols, 
        #                     rows=self.lcd_rows, 
        #                     pin_rs=37, 
        #                     pin_e=35, 
        #                     pins_data=[40, 38, 36, 32, 33, 31, 29, 23])
    
        self.curr_text = ["default", "text"]
        self.clear_line = " "*(self.lcd_cols+2)

    def display_message_callback(self, data):
        self.curr_text[0] = data.l1
        self.curr_text[1] = data.l2

    def update_lcd(self):
        if MOCK:
            # To prevent overwriting previous commands.
            if self.isFirstTime:
                self.isFirstTime = False
            else:
                print(4*(C_UP_R+self.clear_line), end="\r")
            
            print("-"*18)
            print(f"|{self.curr_text[0]: ^16}|\n|{self.curr_text[1]: ^16}|")
            print("-"*18)
        else:
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