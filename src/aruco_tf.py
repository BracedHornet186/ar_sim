#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from geometry_msgs.msg import TransformStamped # Handles TransformStamped message
from sensor_msgs.msg import Image 
from tf2_ros import TransformBroadcaster

import cv2 
import numpy as np 
from scipy.spatial.transform import Rotation as R

# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    print("tvecs", tvecs[0])
    print("rvecs", rvecs[0])
    return rvecs[0].reshape((3, 1)), tvecs[0], trash
class ArucoNode(Node):
  """
  Create an ArucoNode class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('aruco_node')

    # Declare parameters
    self.declare_parameter("aruco_dictionary_name", "DICT_4X4_50")
    self.declare_parameter("aruco_marker_side_length", 0.0785)
    self.declare_parameter("camera_calibration_parameters_filename", "/home/abhiyaan-nuc/ar_ws/src/ar_sim/config/zed.yaml")
    self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
    self.declare_parameter("aruco_marker_name", "aruco_marker")
    
    # Read parameters
    aruco_dictionary_name = self.get_parameter("aruco_dictionary_name").get_parameter_value().string_value
    self.aruco_marker_side_length = self.get_parameter("aruco_marker_side_length").get_parameter_value().double_value
    self.camera_calibration_parameters_filename = self.get_parameter(
      "camera_calibration_parameters_filename").get_parameter_value().string_value
    image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
    self.aruco_marker_name = self.get_parameter("aruco_marker_name").get_parameter_value().string_value

    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
      self.get_logger().info("[INFO] ArUCo tag of  is not supported")
        
    # Load the camera parameters from the saved file
    # cv_file = cv2.FileStorage(self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    self.mtx = np.array(([[525.83642578125, 0., 651.248291015625], 
          [0., 525.83642578125, 350.84613037109375],
           [0., 0., 1. ]]), dtype=np.float32)
    self.dst = np.array(([ 0.0, 0.0, 0.0, 0.0, 0.0 ]), dtype=np.float32)
    # kijcv_file.release()
    
    # Load the ArUco dictionary
    self.get_logger().info("[INFO] detecting '{}' markers...".format(aruco_dictionary_name))
    self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
    
    self.this_aruco_parameters = cv2.aruco.DetectorParameters()
    self.Detector = cv2.aruco.ArucoDetector(self.this_aruco_dictionary, self.this_aruco_parameters)
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      image_topic, 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Initialize the transform broadcaster
    self.tfbroadcaster = TransformBroadcaster(self)
      
    # Used to convert between ROS and OpenCV images
    self.bridge = CvBridge()
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.bridge.imgmsg_to_cv2(data)
    
    # Detect ArUco markers in the video frame
    (corners, marker_ids, _) = self.Detector.detectMarkers(current_frame)
    if marker_ids is not None:
      print(marker_ids)
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
    
      # Draw a square around detected markers in the video frame
      # cv2.aruco.drawDetectedMarkers(current_frame, corners, marker_ids)

      # Get the rotation and translation vectors
      rvecs, tvecs, _ = my_estimatePoseSingleMarkers(
        corners,
        self.aruco_marker_side_length,
        self.mtx,
        self.dst)
        
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      for i, marker_id in enumerate(marker_ids):  

        # Create the coordinate transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'zed_left_camera_frame'
        t.child_frame_id = self.aruco_marker_name
      
        # Store the translation (i.e. position) information
        t.transform.translation.x = tvecs[i][0]
        t.transform.translation.y = tvecs[i][0]
        t.transform.translation.z = tvecs[i][0]

        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array((rvecs[0][0], rvecs[1][0], rvecs[2][0])))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()   
        
        # Quaternion format     
        t.transform.rotation.x = quat[0] 
        t.transform.rotation.y = quat[1] 
        t.transform.rotation.z = quat[2] 
        t.transform.rotation.w = quat[3] 

        # Send the transform
        self.tfbroadcaster.sendTransform(t)    
                  
        # Draw the axes on the marker
        # cv2.aruco.drawAxis(current_frame, self.mtx, self.dst, rvecs[i], tvecs[i], 0.05)        
              
    # Display image
    # cv2.imshow("camera", current_frame)
    
    # mocv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  aruco_node = ArucoNode()
  
  # Spin the node so the callback function is called.
  rclpy.spin(aruco_node)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  aruco_node.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()