#!/usr/bin/python3

# https://github.com/Devanthro/csi_camera/blob/main/csi_camera/ros2_dual_camera.py

import cv2
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("stereocamera_node")
        left_image_topic_ = self.declare_parameter(
            "left_image_topic", "/image/left/image_compressed").value
        right_image_topic_ = self.declare_parameter(
            "right_image_topic", "/image/right/image_compressed").value
        self.left_frame_id_ = self.declare_parameter("left_frame_id", "left_camera").value
        self.right_frame_id_ = self.declare_parameter("right_frame_id", "right_camera").value
        self.left_image_publisher_ = self.create_publisher(CompressedImage, left_image_topic_, 1)
        self.right_image_publisher_ = self.create_publisher(CompressedImage, right_image_topic_, 1)
        self.br = CvBridge()

        self.get_logger().info(
            f"Starting publishers...")

        self.start_cameras()

        self.timer = self.create_timer(1.0/20, self.image_callback)
        self.get_logger().info("Stereocamera node ready")

    def close_videocapture(self):
        if self.left_camera is not None:
            self.left_camera.stop()
            self.left_camera.release()
        if self.right_camera is not None:
            self.right_camera.stop()
            self.right_camera.release()

    def image_callback(self):
        try:
            time_msg = self.get_time_msg()
            if self.left_camera.video_capture.isOpened():
                _, left_image = self.left_camera.read()
                left_img_msg = self.get_image_msg(left_image, time_msg)
                self.left_image_publisher_.publish(left_img_msg)

            if self.right_camera.video_capture.isOpened():
                _, right_image = self.right_camera.read()
                right_img_msg = self.get_image_msg(right_image, time_msg, False)
                self.right_image_publisher_.publish(right_img_msg)

        except Exception as e:
            print(e)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def get_image_msg(self, image, time,left=True):
        """
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        """
        img_msg = self.br.cv2_to_compressed_imgmsg(image) #, dst_format="png")
        #print(img_msg)
        #img_msg = self.br.cv2_to_imgmsg(image, encoding="bgr8")
        img_msg.header.stamp = time
        img_msg.header.frame_id = self.left_frame_id_ if left else self.right_frame_id_
        return img_msg

    def start_cameras(self):

        self.left_camera = CSI_Camera()
        self.left_camera.open(
            gstreamer_pipeline(
                sensor_id=0,
                #capture_width=1920,
                #capture_height=1080,
                flip_method=1,
                #display_width=1920,
                #display_height=1080,
            )
        )
        self.left_camera.start()

        self.right_camera = CSI_Camera()
        self.right_camera.open(
            gstreamer_pipeline(
                sensor_id=1,
                #capture_width=1920,
                #capture_height=1080,
                flip_method=3,
                #display_width=1920,
                #display_height=1080,
            )
        )
        self.right_camera.start()


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080
"""


def gstreamer_pipeline(
        sensor_id=0,
        sensor_mode=0,
        capture_width=3040,
        capture_height=4056,
        display_width=3040,#int(3040/2),
        display_height=4056,#int(4056/2),
        framerate=30,
        flip_method=0,
):
    return (
            "nvarguscamerasrc sensor-id=%d sensor-mode=%d exposurecompensation=0 wbmode=1 ! "
            "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                sensor_id,
                sensor_mode,
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )

def main():
    rclpy.init()
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except Exception as e:
        pass
        #print(e)
    finally:
        camera_node.close_videocapture()
        camera_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
