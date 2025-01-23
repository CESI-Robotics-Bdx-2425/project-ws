#!/usr/bin/env python3
import cv2
import numpy as np
import rospy, rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import tf

class MatrixCalibration:
    def __init__(self):
        rospy.init_node('matrix_calibration')
        self.tf_listener = tf.TransformListener()
        self.bridge = CvBridge()
        
        self.rospack = rospkg.RosPack()
        self.init_params()
        rospy.sleep(3)
        
        rospy.Subscriber('/xtion/rgb/image_rect_color', Image, self.image_callback)
        self.s = rospy.Service('/camera_arm_calibration/save', Empty, self.save)
        rospy.loginfo('Veuillez positionner le bras de Tiago dans la bonne position')

    def save(self, ros_msg):
        try:
            p = f"{self.rospack.get_path('camera_arm_calibration')}/config/aruco.npy"
            np.savetxt(p, self.f)
            rospy.signal_shutdown("save")
        except Exception as e:
            rospy.logerr(f'{e}')
            exit()

    def init_params(self):
        try:
            self.ARUCO_CIBLE_ID = rospy.get_param('~aruco_id', default=2)
            m_file = rospy.get_param('~matrix_file', default=f"{self.rospack.get_path('camera_arm_calibration')}/config/matrix.npy")
            c_file = rospy.get_param('~coefficients_file', default=f"{self.rospack.get_path('camera_arm_calibration')}/config/coefficients_file.npy")
            self.K = np.loadtxt(m_file)
            self.D = np.loadtxt(c_file)
            self.MARKER_LENGTH = rospy.get_param('~marker_length', default=0.038)
        except Exception as e:
            rospy.logerr(f'Erreur lors du chargement des paramètres\n{e}')

    def print_status(self, O_T_A, O_T_P, A_T_P=None):
        # Nettoie l'écran en remontant de 15 lignes (augmenté pour la matrice inverse)
        print("\033[15A\033[J", end="")
        
        print("\nPositions dans base_link:", end="\n")
        
        if O_T_A:
            pos = O_T_A['pose'].pose.position
            print(f"O_T_A - Position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]", end="\n")
        else:
            print("O_T_A non détecté", end="\n")

        if O_T_P:
            pos = O_T_P['pose'].pose.position
            print(f"O_T_P - Position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]", end="\n")
        else:
            print("O_T_P non détecté", end="\n")

        if A_T_P is not None:
            self.f = A_T_P
            print("\nMatrice de transformation directe (4x4):", end="\n")
            # Affichage de la matrice 4x4 ligne par ligne avec un format aligné
            for i in range(4):
                print(f"[{A_T_P[i,0]:8.3f} {A_T_P[i,1]:8.3f} {A_T_P[i,2]:8.3f} {A_T_P[i,3]:8.3f}]", end="\n")

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
    
    def get_arm_position(self):
        t = self.tf_listener.lookupTransform(target_frame='odom', source_frame='arm_left_tool_link', time=rospy.Time(0))
        pose = PoseStamped()
        pose.pose.position.x = t[0][0]
        pose.pose.position.y = t[0][1]
        pose.pose.position.z = t[0][2]
        pose.pose.orientation.x = t[1][0]
        pose.pose.orientation.y = t[1][1]
        pose.pose.orientation.z = t[1][2]
        return pose

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            arucoParams = cv2.aruco.DetectorParameters()
            corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

            O_T_A = None
            A_T_P = None
            O_T_P = None

            if ids is not None:  # Changé de 2 à 1 car on ne cherche plus 2 ArUcos
                for corner, id in zip(corners, ids):
                    if id[0] == self.ARUCO_CIBLE_ID:
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, self.MARKER_LENGTH, cameraMatrix=self.K, distCoeffs=self.D)
                        pose_base_link = self.transform_to_frame(tvec, rvec, source_frame='xtion_rgb_optical_frame', target_frame='odom')
                        if pose_base_link:
                            O_T_A = {'pose': pose_base_link, 'T_matrix': self.pose_to_matrix(pose_base_link)}
                        cv2.drawFrameAxes(img, cameraMatrix=self.K, distCoeffs=self.D, rvec=rvec, tvec=tvec, length=self.MARKER_LENGTH)

                # Déplacé hors de la boucle de détection des ArUcos
                try:
                    pose_base_link = self.get_arm_position()
                    print(pose_base_link, end="\n")
                    if pose_base_link:
                        O_T_P = {'pose': pose_base_link, 'T_matrix': self.pose_to_matrix(pose_base_link)}
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(f"Erreur lors de la récupération de la position du bras: {e}", end="\n")

                if O_T_A and O_T_P:
                    A_T_P = np.dot(np.linalg.inv(O_T_A['T_matrix']), O_T_P['T_matrix'])

            self.print_status(O_T_A, O_T_P, A_T_P)

            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.imshow('Image', img)
            cv2.waitKey(1)

        except Exception as e:
            print(f"Erreur: {e}", end="\n")

if __name__ == '__main__':
    detector = MatrixCalibration()
    rospy.spin()
    cv2.destroyAllWindows()