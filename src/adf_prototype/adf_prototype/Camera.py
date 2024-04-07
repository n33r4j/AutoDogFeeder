import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge

from datetime import datetime


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
       
    
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.subscription_ = self.create_subscription(Int8, '/camera_mode', self.mode_callback, 10)
        self.mode = 0 # 0 -> idle, 1 -> timing out, 2 -> record
        self.framerate = 20.0

        self.isNewRecording = True
        self.isTimingOut = False
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            exit()

        self.frameW = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frameH = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
        self.br = CvBridge()
    

    def mode_callback(self, data):
        self.mode = data.data

    def start_recording(self):
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v') # or 'XVID'
        timestamp = datetime.now().strftime("%H-%M-%S_%d-%m-%Y")
        self.output = cv2.VideoWriter('video_{timestamp}.mp4', self.fourcc, self.framerate, (self.frameW, self.frameH))
        self.isNewRecording = False

    def write_frame(self, frame):
        self.output.write(frame)

    def stop_recording(self):
        self.output.release()
        self.isNewRecording = True


    def cleanup(self):
        self.cap.release()

    def timer_callback(self):
        """
        gets called every 0.1 seconds.
        """
        
        ret, frame = self.cap.read()
            
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')

            if self.mode == 2:
                if self.isNewRecording:
                    self.start_recording()
                self.write_frame(frame)

            elif self.mode == 1:
                self.write_frame(frame)

            elif self.mode == 0:
                pass
        else:
            self.get_logger().error("Couldn't get frame")


def main(args=None):
    rclpy.init(args=args)

    camera_node = Camera()
    rclpy.spin(camera_node)
    
    camera_node.cleanup()
    camera_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()