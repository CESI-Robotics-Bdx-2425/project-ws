#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from book_detector.srv import FindAruco
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf

class ArucoPoseCalculator:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Init du noeud ROS
        rospy.init_node('aruco_pose_calculator')
        
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
        
        self.ARUCO_CIBLE_ID = None
        self.MARKER_LENGTH = 0.038
        
        # Initialisation du service
        self.service = rospy.Service('book_detector', FindAruco, self.start_search)
        rospy.sleep(2)
        rospy.spin()


    def init_params(self):
        try:
            self.MARKER_LENGTH = rospy.get_param('~marker_length', default=0.038)  # Marker size in meters
            self.scan_limit = rospy.get_param('~scan_limit', default=1)  # Marker size in meters
            
            f = rospy.get_param('~aruco_matrix', default='aruco.npy')
            self.A_T_P = np.loadtxt(f)
            
            rospy.loginfo(f'Init with MARKER_LENGTH {self.MARKER_LENGTH}')
            
            # Charger toute la structure
            self.matrix_file = rospy.get_param('~matrix_file')
            self.coefficients_file = rospy.get_param('~coefficients_file')
            self.K = np.loadtxt(self.matrix_file)
            self.D = np.loadtxt(self.coefficients_file)
            
            print("Camera matrix:", self.K)
            print("Distortion coefficients:", self.D)
        except KeyError as e:
            rospy.logerr(f"Paramètre manquant : {e}")
            raise
        
    def start_search(self, req):
        # Create subscribers for images
        self.ARUCO_CIBLE_ID = req.aruco_id
        self.scan_count = 0
        
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.image_callback, queue_size=10)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(2)

        # Scan with head until Aruco is found
        while not rospy.is_shutdown() and self.O_T_P is None:
            # Update head position for scanning
            self.current_position += 0.02 * self.direction_multiplier

            # Check boundaries and change direction if needed
            if self.current_position > 1.24:
                self.current_position = 1.24
                self.direction_multiplier = -1
            elif self.current_position < -1.24:
                self.current_position = -1.24
                self.direction_multiplier = 1
                self.scan_count += 1

            if self.scan_count == self.scan_limit:
                return PoseStamped()

            # Create and publish head movement
            head_point = JointTrajectoryPoint()
            head_point.positions = [self.current_position, -0.25]
            head_point.time_from_start = rospy.Duration(0.5)
            self.head_trajectory.points = [head_point]
            self.head_pub.publish(self.head_trajectory)
            
            self.rate.sleep()
        
        self.image_sub.unregister()
        self.head_pub.unregister()
        return self.O_T_P      

    def print_status(self, aruco_cible_pose, calculated_pose):
        # Nettoie l'écran
        print("\033[8A\033[J", end="")
        
        print("\nPositions dans base_link:", end="\n")
        
        if aruco_cible_pose:
            pos = aruco_cible_pose.pose.position
            print(f"ArUco cible - Position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]", end="\n")
            
            if calculated_pose is not None:
                # Extraction de la position (translation)
                position = calculated_pose[:3, 3]
                
                # Extraction de l'orientation (rotation) en angles d'Euler (en radians)
                rotation_matrix = calculated_pose[:3, :3]
                quat_angles = R.from_matrix(rotation_matrix).as_quat()
                
                print(f"Poignet à positionner (calculé) - Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]", end="\n")
                print(f"Poignet à positionner (calculé) - Orientation: [{quat_angles[0]:.3f}, {quat_angles[1]:.3f}, {quat_angles[2]:.3f}, {quat_angles[3]:.3f}]", end="\n")
        else:
            print("ArUco cible non détecté", end="\n")
            print("Impossible de calculer la position de l'ArUco à positionner", end="\n")

    def transform_to_frame(self, tvec, rvec, source_frame='xtion_rgb_optical_frame', target_frame='odom'):
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
            return None

    def pose_to_matrix(self, pose):
        T = np.eye(4)
        T[:3, 3] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        quat = [pose.pose.orientation.x, pose.pose.orientation.y, 
                pose.pose.orientation.z, pose.pose.orientation.w]
        T[:3, :3] = R.from_quat(quat).as_matrix()
        return T

    def calculate_target_pose(self, cible_matrix):
        # Calcule la matrice de transformation pour l'ArUco à positionner
        # On change l'ordre de la multiplication pour respecter la transformation d'origine
        target_matrix = np.dot(cible_matrix, self.A_T_P)
        
        # Création d'une instance de PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()  # Horodatage du message
        pose_stamped.header.frame_id = 'odom'  # Cadre de référence

        # Extraction de la position (translation)
        position = target_matrix[:3, 3]
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]

        # Extraction de l'orientation (rotation) sous forme de quaternion
        rotation_matrix = target_matrix[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()  # [qx, qy, qz, qw]
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        return pose_stamped

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            arucoParams = cv2.aruco.DetectorParameters()
            corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

            aruco_cible_pose = None
            self.O_T_P = None

            if ids is not None:
                for corner, id in zip(corners, ids):
                    if id[0] == self.ARUCO_CIBLE_ID:
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.MARKER_LENGTH, cameraMatrix=self.K, distCoeffs=self.D)
                        aruco_cible_pose = self.transform_to_frame(tvec, rvec, source_frame='xtion_rgb_optical_frame', target_frame='odom')
                        if aruco_cible_pose:
                            O_T_A = self.pose_to_matrix(aruco_cible_pose)
                            self.O_T_P = self.calculate_target_pose(O_T_A)

        except Exception as e:
            print(f"Erreur: {e}", end="\n")

if __name__ == '__main__':
    calculator = ArucoPoseCalculator()
    rospy.spin()
    cv2.destroyAllWindows()