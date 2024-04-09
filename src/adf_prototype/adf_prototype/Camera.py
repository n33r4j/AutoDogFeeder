import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os

from datetime import datetime


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
       
    
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.subscription_ = self.create_subscription(Int8, '/camera_mode', self.mode_callback, 10)
        self.mode = 0 # 0 -> idle, 1 -> record
        self.framerate = 20.0
        self.timeout = 5 # seconds
        self.recordings_dir = "recordings"

        self.isNewRecording = True
        self.hasStoppedRecording = True
        self.isTimingOut = False
        self.start_time = -1
        
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
        # self.get_logger().info('%d %d' % (self.mode, data.data))
        if not self.isTimingOut and self.mode > data.data:
            self.isTimingOut = True
            self.start_time = cv2.getTickCount()
        
        self.mode = data.data

    def start_recording(self):
        self.get_logger().info('Starting recording...')
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v') # or 'XVID'
        timestamp = datetime.now().strftime("%H-%M-%S_%d-%m-%Y")
        self.output = cv2.VideoWriter(f'{self.recordings_dir}/video_{timestamp}.mp4', self.fourcc, self.framerate, (self.frameW, self.frameH))
        self.isNewRecording = False
        self.hasStoppedRecording = False

    def write_frame(self, frame):
        self.output.write(frame)

    def stop_recording(self):
        self.get_logger().info('Stopping recording...')
        self.output.release()
        self.isNewRecording = True
        self.hasStoppedRecording = True


    def cleanup(self):
        self.cap.release()

    def timer_callback(self):
        """
        gets called every 0.1 seconds.
        """
        
        ret, frame = self.cap.read()
            
        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            # self.get_logger().info('Publishing video frame')

            if self.mode == 1:
                if self.isNewRecording:
                    self.start_recording()
                self.write_frame(frame)
                self.start_time = cv2.getTickCount()

            elif self.mode == 0:
                if self.isTimingOut:
                    curr_time = (cv2.getTickCount() - self.start_time) / cv2.getTickFrequency()
                    if curr_time < self.timeout:
                        self.write_frame(frame)
                    else:
                        self.isTimingOut = False

                else:
                    if not self.hasStoppedRecording:
                        self.stop_recording()
                    else:
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