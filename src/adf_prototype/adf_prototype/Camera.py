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
        self.subscription_ = self.create_subscription(Int8, 'camera_mode', self.mode_callback)
        self.mode = 0 # 0 -> idle, 1 -> timing out, 2 -> record
        self.framerate = 20.0
        self.frameW = 640
        self.frameH = 480
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera")
            
        self.br = CvBridge()
    

    def mode_callback(self, data):
        self.mode = data.data

    def start_recording(self):

    def write_frame(self, frame):


    def stop_recording(self):


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
        