#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_commander import PlanningSceneInterface, RobotCommander
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
import table_detector.srv as c_srv
from scipy.spatial.transform import Rotation as R

# Define ArUco dictionary and parameters
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
DETECTOR_PARAMETERS = cv2.aruco.DetectorParameters()

class TableDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_image = None
        self.aruco_detected = False
        self.detected_positions = []        

        # Head movement configuration
        self.head_trajectory = JointTrajectory()
        self.head_trajectory.joint_names = ['head_1_joint', 'head_2_joint']
        self.current_position = 0.0
        self.direction_multiplier = 1
        self.scan_count = 0

        # Initialize ROS node
        rospy.init_node('table_detector')
        rospy.loginfo('TableDetector node initialized')
        rospy.sleep(2)
        
        # Init all params
        self.init_params()

        # TF Listener
        self.tf_listener = tf.TransformListener()

        # MoveIt! scene interface
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()

        rospy.sleep(2)  # Allow TF and MoveIt! to initialize
        self.scene.remove_world_object()
        rospy.sleep(2)

        self.s = rospy.Service('table_detector', c_srv.TableDetector, self.scan_head)

        rospy.spin()

    def init_params(self):
        try:
            self.ARUCO_TARGET_IDS = rospy.get_param('~aruco_target_id', default=[1, 4])  # Target markers to detect
            self.MARKER_LENGTH = rospy.get_param('~marker_length', default=0.04)  # Marker size in meters
            self.scan_limit = rospy.get_param('~scan_limit', default=5) # Limit of scan to perform
            
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

    def depth_callback(self, ros_image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting depth image: {e}")

    def image_callback(self, ros_image):
        try:
            self.cam_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"Error converting RGB image: {e}")
        self.detect_aruco_markers()

    def detect_aruco_markers(self):
        if self.cam_image is None:
            return

        corners, ids, _ = cv2.aruco.detectMarkers(self.cam_image, ARUCO_DICT, parameters=DETECTOR_PARAMETERS)

        if ids is not None:
            for corner, marker_id in zip(corners, ids.flatten()):
                if marker_id in self.ARUCO_TARGET_IDS:
                    rospy.loginfo(f"Detected marker {marker_id}")

                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.MARKER_LENGTH, self.K, self.D)
                    transformed_point = self.transform_to_frame(tvec, rvec)

                    if transformed_point:
                        self.detected_positions.append(transformed_point)
                        self.ARUCO_TARGET_IDS.remove(marker_id)

                    if len(self.ARUCO_TARGET_IDS) == 0:
                        self.add_collision_box(self.detected_positions)
                        self.aruco_detected = True

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

    def add_collision_box(self, positions):
        if len(positions) < 2:
            rospy.logwarn("Not enough ArUco positions to define the table")
            return

        center_x = (positions[0].pose.position.x + positions[1].pose.position.x) / 2
        center_y = (positions[0].pose.position.y + positions[1].pose.position.y) / 2
        center_z = (positions[0].pose.position.z + positions[1].pose.position.z) / 2

        size_x = abs(positions[0].pose.position.x - positions[1].pose.position.x) + 0.7
        size_y = abs(positions[0].pose.position.y - positions[1].pose.position.y) + 0.1
        size_z = 0.9

        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = center_x + 0.25
        box_pose.pose.position.y = center_y
        box_pose.pose.position.z = center_z - size_z / 2
        box_pose.pose.orientation = positions[0].pose.orientation
        
        # Rotate over z
        o = box_pose.pose.orientation
        quat = R.from_quat([o.x, o.y, o.z, o.w]).as_quat()
        z_rotation = R.from_euler('z', 90, degrees=True).as_quat()
        quat = (R.from_quat(quat) * R.from_quat(z_rotation)).as_quat()
        o.x, o.y, o.z, o.w = quat
        box_pose.pose.orientation = o
        box_pose.pose.orientation.y = 0


        # Add marker at Aruco Positions
        i = 0
        for pos in positions:
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.ns = f"aruco_{i}"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pos.pose
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)
            i += 1

        self.scene.add_box("table", box_pose, (size_x, size_y, size_z))
        rospy.loginfo(f"Added collision box for the table at ({center_x}, {center_y}, {center_z})")

    def reset(self):
        self.aruco_detected = False
        self.detected_positions = []
        self.ARUCO_TARGET_IDS = [4, 1]  # Target markers to detect

    def scan_head(self, _):
        rate = rospy.Rate(10)
        self.reset()
        
        # Publishers and subscribers
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        self.result_pub = rospy.Publisher('/table_detector/result', String, queue_size=10)
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_rect_color", Image, self.image_callback, queue_size=10)
        self.depth_sub = rospy.Subscriber("/xtion/depth/image_rect", Image, self.depth_callback, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        while not rospy.is_shutdown() and not self.aruco_detected:
            self.current_position += 0.02 * self.direction_multiplier

            if self.current_position > 1.24:
                self.current_position = 1.24
                self.direction_multiplier = -1
            elif self.current_position < -1.24:
                self.current_position = -1.24
                self.direction_multiplier = 1
                self.scan_count += 1
                
                
            if self.scan_count == self.scan_limit:
                return -1

            head_point = JointTrajectoryPoint()
            head_point.positions = [self.current_position, -0.6]
            head_point.time_from_start = rospy.Duration(0.5)

            self.head_trajectory.points = [head_point]
            self.head_pub.publish(self.head_trajectory)
            rate.sleep()
        
        self.head_pub.unregister()
        self.result_pub.unregister()
        self.image_sub.unregister()
        self.depth_sub.unregister()
        self.marker_pub.unregister()
        
        return 1

if __name__ == "__main__":
    try:
        detector = TableDetector()
    except rospy.ROSInterruptException:
        rospy.loginfo("TableDetector node terminated.")