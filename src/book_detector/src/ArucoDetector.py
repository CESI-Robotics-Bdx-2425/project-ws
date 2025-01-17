#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from book_detector.srv import FindAruco
from scipy.spatial.transform import Rotation as R

# Define aruco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()

book_offsets = {
    0: {'x': 0.040, 'y': 0.105, 'z': 0.149},
    1: {'x': 0.100, 'y': 0.105, 'z': 0.149},
    2: {'x': 0.160, 'y': 0.105, 'z': 0.149},
}

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_image = None

        # Init the rospy node
        rospy.init_node('aruco_detector')
        rospy.sleep(2)
        rospy.loginfo('aruco_detector node initialised')

        # Create listener for transformations
        self.tf_listener = tf.TransformListener()

        # Head movement configuration
        self.rate = rospy.Rate(10)
        self.head_trajectory = JointTrajectory()
        self.head_trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        
        # Variables for head scanning
        self.current_position = 0.0
        self.direction_multiplier = 1
        
        self.init_params()

        # Initialisation du service
        self.service = rospy.Service('book_detector', FindAruco, self.start_search)
        rospy.sleep(2)
        rospy.spin()
        
    def init_params(self):
        try:
            self.MARKER_LENGTH = rospy.get_param('~marker_length', default=0.04)  # Marker size in meters
            
            # Charger toute la structure
            calibration_data = rospy.get_param("~pal_camera_calibration_intrinsics")
            rgb_camera = calibration_data['rgb_xtion']
            self.K = np.array(rgb_camera['camera_matrix']['data']).reshape((3, 3))
            self.D = np.array(rgb_camera['distortion_coefficients']['data'])
            
            print("Camera matrix:", self.K)
            print("Distortion coefficients:", self.D)
        except KeyError as e:
            rospy.logerr(f"Paramètre manquant : {e}")
            raise

    def start_search(self, req):
        # Create subscribers for images
        self.aruco_to_find = req.aruco_id
        self.aruco_position = None
        
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.image_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber("/xtion/depth/image_rect", Image, self.depth_callback, queue_size=10)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(2)

        # Scan with head until Aruco is found
        while not rospy.is_shutdown() and self.aruco_position is None:
            # Update head position for scanning
            self.current_position += 0.02 * self.direction_multiplier

            # Check boundaries and change direction if needed
            if self.current_position > 1.24:
                self.current_position = 1.24
                self.direction_multiplier = -1
            elif self.current_position < -1.24:
                self.current_position = -1.24
                self.direction_multiplier = 1

            # Create and publish head movement
            head_point = JointTrajectoryPoint()
            head_point.positions = [self.current_position, -1]
            head_point.time_from_start = rospy.Duration(0.5)
            self.head_trajectory.points = [head_point]
            self.head_pub.publish(self.head_trajectory)
            
            self.rate.sleep()
        
        self.image_sub.unregister()
        self.depth_sub.unregister()
        self.head_pub.unregister()
        return self.aruco_position


    def depth_callback(self, ros_image):
        try:
            # Convert ROS message to OpenCV Image (Depth Image)
            self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Error while converting depth image : {e}")

    def image_callback(self, ros_image):
        try:
            # Convert ROS message to OpenCV Image
            self.cam_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error while converting RGB image : {e}")
        self.aruco_callback()

    def transform_to_frame(self, tvec, rvec, source_frame='xtion_rgb_optical_frame', target_frame='base_link'):
        try:
            point = PoseStamped()
            point.header.frame_id = source_frame
            point.header.stamp = rospy.Time(0)
            point.pose.position.x, point.pose.position.y, point.pose.position.z = tvec[0][0]
            quat = R.from_rotvec(rvec[0][0]).as_quat()
            point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z, point.pose.orientation.w = quat

            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            return self.tf_listener.transformPose(target_frame, point)
        except Exception as e:
            rospy.logerr(f"TF transform error: {e}")
            return None

    def aruco_callback(self):
        corners, ids, _ = cv2.aruco.detectMarkers(self.cam_image, aruco_dict, parameters=parameters)
        if ids is not None:
            for corner, id in zip(corners, ids):
                if id == self.aruco_to_find:
                    # Estimate position of Aruco
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.MARKER_LENGTH, self.K, self.D)

                    transform_point = self.transform_to_frame(tvec, rvec)

                    # Get Aruco center
                    corner = corner.reshape((4,2))
                    cx = int(corner[:, 0].mean())
                    cy = int(corner[:, 1].mean())

                    # Get depth value                
                    if self.depth_image is not None and 0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]:
                        z = self.depth_image[cy, cx]
                    else:
                        z = None

                    # Print coordinates
                    print(f"Marqueur {id}, {transform_point}")
                    
                    # Ajouter les offsets du livre
                    transform_point.pose.position.x += book_offsets[0]['x']
                    transform_point.pose.position.y += book_offsets[0]['y']
                    transform_point.pose.position.z += book_offsets[0]['z']
                    
                    self.aruco_position = transform_point
                    #print(f"Position (m): X={point_transformed[0]:.3f}, Y={point_transformed[1]:.3f}, Z={point_transformed[2]:.3f}")

if __name__ == "__main__":
    detector = ArucoDetector()