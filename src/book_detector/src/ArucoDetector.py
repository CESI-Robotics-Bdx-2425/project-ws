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

# Camera calibration matrix
D = np.array([0.03428809964086624, -0.11761127959914759, -0.004087049185391351, 0.00022993438231999193, 0.0])
K = np.array(
    [522.0592726819208, 0.0, 327.1122193418259, 0.0, 522.1943972439309, 228.49840496340164, 0.0, 0.0, 1.0]
).reshape((3, 3))

# Set coordinates system
marker_length = 0.04
obj_points = np.array([
    [-marker_length / 2, marker_length / 2, 0],
    [marker_length / 2, marker_length / 2, 0],
    [marker_length / 2, -marker_length / 2, 0],
    [-marker_length / 2, -marker_length / 2, 0]
], dtype=np.float32)

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
        self.head_point = JointTrajectoryPoint()
        self.head_point.positions = [0, -1]
        self.head_point.time_from_start = rospy.Duration(0.5)
        self.head_trajectory.points = [self.head_point]

        # Initialisation du service
        self.service = rospy.Service('book_detector', FindAruco, self.start_search)
        rospy.sleep(2)
        rospy.spin()

    def start_search(self, req):
        # Create subscribers for images
        self.aruco_to_find = req.aruco_id
        self.aruco_position = None
        
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.image_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber("/xtion/depth/image_rect", Image, self.depth_callback, queue_size=10)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(2)
        
        # Set head in position
        self.head_pub.publish(self.head_trajectory)
        self.rate.sleep()
        
        while self.aruco_position == None:
            rospy.sleep(0.1)
        
        self.image_sub.unregister()
        self.depth_sub.unregister()
        self.head_pub.unregister()
        cv2.destroyAllWindows()
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
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, marker_length, K, D)

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

            # Draw the detected Aruco
            cv2.aruco.drawDetectedMarkers(self.cam_image, corners)

        try:
            if self.cam_image is not None:
                cv2.imshow('RGB Camera', self.cam_image)
            if self.depth_image is not None:
                cv2.imshow('Depth Camera', self.depth_image)
            cv2.waitKey(1)
        except Exception as e:
            raise

if __name__ == "__main__":
    detector = ArucoDetector()