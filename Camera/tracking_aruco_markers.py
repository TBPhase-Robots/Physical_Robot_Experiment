"""
Code description: Code will identify ArUco tags, and then calculate the position of their centres. It will then use trigonemtry to calculate
their real world position. This position is then published to a ros topic as a geometry pose message. The code can identify new tags and
then make a new ros topics for each tag.
"""

import sys
sys.path.append('..')

import sys
import cv2
from cv2 import cvtColor
from cv2 import aruco

import numpy as np
import time

import os
import pickle

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import String

from Logging.ros_logger import RosLogger

special_markers = [100, 101, 102, 103]

class ArucoTrack(Node):

    # The position of the simulation origin in world space
    origin: Vector3 = Vector3()

    def __init__(self, device):
        # Create ROS node
        super().__init__(node_name="camera_tracker")#

        # Set up logging
        self.logger = RosLogger(self, "camera_tracker")

        # Load the calibration file
        if not os.path.exists('./CameraCalibration.pckl'):
            self.logger.error("Calibration file not found.")
            exit()
        else:
            f = open('./CameraCalibration.pckl', 'rb')
            self.cameraMatrix, self.distCoeffs, _, _ = pickle.load(f)
            f.close()
            if self.cameraMatrix is None or self.distCoeffs is None:
                self.logger.error("Invalid calibration file.")
                exit()

        # Create a dictionary to store robot pose publishers
        self.pub_dict = {}

        # Create publishers for the arena corner poses
        self.arena_corners_pubs = []
        for i in range(100, 104):
            pub = self.create_publisher(Pose, f"arena_corners_{i}", 10)
            self.arena_corners_pubs.append(pub)

        # set up camera
        self.cam = cv2.VideoCapture(device)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, 150)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        self.cam.set(cv2.CAP_PROP_FPS, 60)

        # set up ArUco settings and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.active_robots = []

        # Creates subscriptions
        self.added_robot_subscriber = self.create_subscription(Int32, "/global/robots/added", self.added_robot_callback, 10)
        self.removed_robot_subscriber = self.create_subscription(Int32, "/global/robots/removed", self.removed_robot_callback, 10)
        self.origin_subscriber = self.create_subscription(Vector3, "/global/origin", self.origin_callback, 10)


        while True: # would normally use a while rospy not shutdown, but caused opencv to crash
            # Check for new ROS messages
            rclpy.spin_once(self, timeout_sec=0)

            # Detect and publish robot poses
            self.get_image()
            self.detect_markers()
            self.publish_positions()

            if cv2.waitKey(1) and 0xFF == ord("q"):
                break

        self.cam.release()

        cv2.destroyAllWindows()

    # retrieves the image from the webcam
    def get_image(self):

        ret, self.frame = self.cam.read()

        # makes image greyscale
        self.grey = cvtColor(self.frame,cv2.COLOR_BGR2GRAY)

    # Detects and outputs coordinates for aruco markers
    def detect_markers(self):
        # Detects markers
        corner_list, id_list, rejectedImgPoints = aruco.detectMarkers(self.grey, self.aruco_dict, parameters=self.parameters)

        self.frame_markers = self.frame.copy()
        self.centres_list = []
        self.active_ids = id_list

        # Checks if any markers have been detected
        if len(corner_list) > 0:
            # Finds the markers poses
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corner_list, 24/1000, self.cameraMatrix, self.distCoeffs)

            for i in range(len(corner_list)):
                # Gets robot id
                id = id_list[i][0]

                # Gets robot coordinates
                x = translation_vectors[i][0][0]
                y = translation_vectors[i][0][1]

                # Converts robot orientation from a vector to a matrix
                rotation_matrix = np.identity(3)
                cv2.Rodrigues(rotation_vectors[i], rotation_matrix)

                # Converts the matrix to euler angles (skips theta_2, as it is not needed)
                theta_1 = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                s_1 = np.sin(theta_1)
                c_1 = np.cos(theta_1)
                theta_3 = np.arctan2(s_1 * rotation_matrix[2, 0] - c_1 * rotation_matrix[1, 0], c_1 * rotation_matrix[1, 1] - s_1 * rotation_matrix[2, 1])

                # Gets the robots orientation from the euler angles
                orientation = -theta_3

                # Adds the robots pose to the list of robots
                centre = [id, x - self.origin.x, -(y - self.origin.y), orientation]
                self.centres_list.append(centre)

                # Draws the robots pose to the screen
                cv2.drawFrameAxes(self.frame_markers, self.cameraMatrix, self.distCoeffs, rotation_vectors[i], translation_vectors[i], 0.01)

        # Draws the origin to the screen
        rod_identity = np.zeros((1, 3))
        cv2.Rodrigues(np.identity(3), rod_identity)
        origin_array = np.array([[self.origin.x, self.origin.y, self.origin.z]])
        cv2.drawFrameAxes(self.frame_markers, self.cameraMatrix, self.distCoeffs, rod_identity, origin_array, 0.01)

        # Displays the poses
        cv2.imshow("markers", self.frame_markers)

    def added_robot_callback(self, msg: Int32):
        self.logger.log(f'new robot: {msg.data}')
        self.active_robots.append(msg.data)
        self.create_robot(msg.data)

    def removed_robot_callback(self, msg: Int32):
        self.logger.log(f'removing robot: {msg.data}')
        self.remove_robot(msg.data)
        self.active_robots.remove(msg.data)
 
    # Publishes robot poses
    def publish_positions(self):

        for pos in self.centres_list:
            id = pos[0]

            # Create a pose message with the robots pose data
            positions = Pose()
            positions.position.x = pos[1]
            positions.position.y = pos[2]
            positions.orientation.z = pos[3]

            # Checks that the id is not and arena marker
            if id not in [100, 101, 102, 103]:
                # Publishes the pose for the robot
                try:
                    self.pub_dict[id].publish(positions)
                except KeyError:
                    self.logger.warn(f'No publisher for robot{id}')

            else:
                # Publishes the pose for the arena marker
                self.arena_corners_pubs[id - 100].publish(positions)
            
    def create_robot(self, ID):
        """
        As the Aruco gives an ID number, a dictionary containing all the publishing objects is created, with each robots publisher as the key
        """
        pub_name = f"/robot{ID}/camera_poses"
        self.pub_dict[ID] = self.create_publisher(Pose, pub_name, 10)

    def remove_robot(self, ID):
        """
        As the Aruco gives an ID number, a dictionary containing all the publishing objects is created, with each robots publisher as the key
        """
        self.destroy_publisher(self.pub_dict[ID])
        self.pub_dict.pop(ID)

    def shutdown_callback(self):

        self.logger.log("shutting down")

        self.cam.release()

        cv2.destroyAllWindows()

    def origin_callback(self, msg: Vector3):
        self.origin = msg

def main():
    # Sets up ROS
    rclpy.init()

    # Reads camera device number from args
    device = 0
    if len(sys.argv) > 1:
        device = int(sys.argv[1])

    # Create marker tracker
    aruco_tracker = ArucoTrack(device)
    
    # Pause and keep checking for ROS messages
    rclpy.spin(aruco_tracker)

    rclpy.shutdown()

if __name__ == "__main__":
    main()