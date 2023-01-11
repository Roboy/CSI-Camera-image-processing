#!/usr/bin/python3

import cv2
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Int32MultiArray, Image
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
        super().__init__("face_detection_node")
        left_face_coordinates_topic_ = self.declare_parameter(
            "left_face_coordinates_topic", "/faces/left/coordinates_array").value
        right_face_coordinates_topic_ = self.declare_parameter(
            "right_face_coordinates_topic", "/faces/right/coordinates_array").value
        self.left_frame_id_ = self.declare_parameter("left_frame_id", "left_camera").value
        self.right_frame_id_ = self.declare_parameter("right_frame_id", "right_camera").value
        self.left_faces_publisher_ = self.create_publisher(Int32MultiArray, left_face_coordinates_topic_, 1)
        self.right_faces_publisher_ = self.create_publisher(Int32MultiArray, right_face_coordinates_topic_, 1)
        self.br = CvBridge()

        self.get_logger().info(
            f"Starting publishers...")

        self.start_cameras()

        self.timer = self.create_timer(1.0/20, self.image_callback)
        self.get_logger().info("Stereocamera node ready")


        self.face_cascade = cv2.CascadeClassifier(
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
        )
        self.eye_cascade = cv2.CascadeClassifier(
            "/usr/share/opencv4/haarcascades/haarcascade_eye.xml"
        )

    def close_videocapture(self):
        if self.left_camera is not None:
            self.left_camera.stop()
            self.left_camera.release()
        if self.right_camera is not None:
            self.right_camera.stop()
            self.right_camera.release()

    def face_detect(self, image):
        return np.array([[1, 2, 3, 4], [1, 2, 3, 4]])

    def image_callback(self):
        try:
            time_msg = self.get_time_msg()
            if self.left_camera.video_capture.isOpened():
                _, left_image = self.left_camera.read()
                left_faces = self.face_detect(left_image)
                left_faces_msg = self.get_face_msg(left_faces)
                self.left_faces_publisher_.publish(left_faces_msg)

            if self.right_camera.video_capture.isOpened():
                _, right_image = self.right_camera.read()
                right_faces = self.face_detect(right_image)
                right_faces_msg = self.get_face_msg(right_faces)
                self.left_faces_publisher_.publish(right_faces_msg)

        except Exception as e:
            print(e)

    def get_face_msg(self, faces):
        """
        Get image message, flatten the 2D array
        """
        face_msg = Int32MultiArray()
        face_msg.data = faces.flatten()
        return face_msg

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
