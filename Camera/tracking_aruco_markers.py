"""
Code description: Code will identify ArUco tags, and then calculate the position of their centres. It will then use trigonemtry to calculate
their real world position. This position is then published to a ros topic as a geometry pose message. The code can identify new tags and
then make a new ros topics for each tag.
"""

import sys
import cv2
from cv2 import cvtColor
from cv2 import aruco

import numpy as np
import time


import os
import pickle
# import rospy2 as rospy

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32
from std_msgs.msg import String

special_markers = [100, 101, 102, 103]


class ArucoTrack(Node):

    origin: Vector3 = Vector3()

    def __init__(self, device):

        # initiate ros parts
        super().__init__(node_name="camera_tracker")

        if not os.path.exists('./CameraCalibration.pckl'):
            print("Calibration file not found.")
            exit()
        else:
            f = open('./CameraCalibration.pckl', 'rb')
            self.cameraMatrix, self.distCoeffs, _, _ = pickle.load(f)
            f.close()
            if self.cameraMatrix is None or self.distCoeffs is None:
                print("Invalid calibration file.")
                exit()

        # as ArUco tags have IDs, the publisher objects are stored in a dictionary with their ID as the key
        self.pub_dict = {}
        self.middle_pubs = {}
        
        self.middles = []

        # rclpy.Context.on_shutdown(super().context, self.shutdown_callback)

        # imput camera values
        # used later to convert the position in the image to the functions
        # self.vert_res = 3840 #1920
        # self.horiz_res = 2160 #1080
        # self.vert_res = 1920
        # self.horiz_res = 1080

        # self.cam_height = 1.55
        # self.horiz_cam_aperture = 78 *2*np.pi/360
        # self.vert_cam_aperture = (self.vert_res/self.horiz_res) * self.horiz_cam_aperture # vert cam aperture 

        # self.horiz_dist_ground = self.cam_height * np.tan(self.horiz_cam_aperture)
        # self.vert_dist_ground = self.cam_height * np.tan(self.vert_cam_aperture)

        # set up camera for cv2
        self.cam = cv2.VideoCapture(device)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, 150)
        # self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
        self.cam.set(cv2.CAP_PROP_FPS, 60)

        # set up ArUco settings and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

        self.active_robots = []
        self.added_robot_subscriber = self.create_subscription(Int32, "/global/robots/added", self.added_robot_callback, 10)
        self.removed_robot_subscriber = self.create_subscription(Int32, "/global/robots/removed", self.removed_robot_callback, 10)
        self.origin_subscriber = self.create_subscription(Vector3, "/global/origin", self.origin_callback, 10)

        while True: # would normally use a while rospy not shutdown, but caused opencv to crash
            rclpy.spin_once(self, timeout_sec=0)

            self.get_image()
            self.detect_markers()
            self.calc_positions()
            self.publish_positions()
            if cv2.waitKey(1) and  0xFF == ord("q"):
                break

        self.cam.release()

        cv2.destroyAllWindows()

    # def added_robot_callback(self,)

    def get_image(self): # retrieves the image from the webcam

        ret, self.frame = self.cam.read()

        # makes image greyscale
        self.grey = cvtColor(self.frame,cv2.COLOR_BGR2GRAY)

    def detect_markers(self):
        corner_list, id_list, rejectedImgPoints = aruco.detectMarkers(self.grey, self.aruco_dict, parameters=self.parameters)
        # self.frame_markers = aruco.drawDetectedMarkers(self.frame.copy(), corner_list, id_list)
        self.frame_markers = self.frame.copy()

        self.centres_list = []

        self.active_ids = id_list

        if len(corner_list) > 0:
            rotation_vectors, translation_vectors, _objPoints = aruco.estimatePoseSingleMarkers(corner_list, 24/1000, self.cameraMatrix, self.distCoeffs)

            for i in range(len(corner_list)):
                corners = corner_list[i]
                id = id_list[i][0]

                corners = corners[0] # there is double bracket, this is to get rid of one of those brackets

                x_tot = 0
                y_tot = 0

                for corner in corners:
                     x_tot += corner[0]
                     y_tot += corner[1]

                middle = np.array([x_tot/4,y_tot/4])
                self.middles.append(middle)

                x = translation_vectors[i][0][0]
                y = translation_vectors[i][0][1]
                orientation = rotation_vectors[i][0][1]

                # orientation = self.calculate_orientation(middle,corners[0],corners[1])
                # centre = [id,int(x_tot/4),int(y_tot/4),orientation]
                # cv2.circle(self.frame_markers,(centre[1],centre[2]),5,(255,0,0),2) # plots a circle in the middle of the marker 
                rotation_matrix = np.identity(3)
                cv2.Rodrigues(rotation_vectors[i], rotation_matrix)
                # print(rotation_matrix)

                theta_1 = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                s_1 = np.sin(theta_1)
                c_1 = np.cos(theta_1)
                theta_3 = np.arctan2(s_1 * rotation_matrix[2, 0] - c_1 * rotation_matrix[1, 0], c_1 * rotation_matrix[1, 1] - s_1 * rotation_matrix[2, 1])

                orientation = -theta_3

                centre = [id, x - self.origin.x, y - self.origin.y, orientation,middle]
                self.centres_list.append(centre)

                cv2.drawFrameAxes(self.frame_markers, self.cameraMatrix, self.distCoeffs, rotation_vectors[i], translation_vectors[i], 0.01)


        rod_identity = np.zeros((1, 3))
        cv2.Rodrigues(np.identity(3), rod_identity)

        origin_array = np.array([[self.origin.x, self.origin.y, self.origin.z]])

        cv2.drawFrameAxes(self.frame_markers, self.cameraMatrix, self.distCoeffs, rod_identity, origin_array, 0.01)
        cv2.imshow("markers",self.frame_markers)

            
    def calculate_orientation(self,centre,top_left,top_right):
        top_middle = (top_right + top_left) / 2
        vec_to_top = top_middle - centre
        # cv2.line(self.frame_markers,(int(centre[0]),int(centre[1])),(int(top_middle[0]),int(top_middle[1])),(0,0,255),8)
        orientation = np.arctan2(vec_to_top[0],vec_to_top[1])
        return orientation

    def added_robot_callback(self, msg: Int32):
        print(f'new robot: {msg.data}')
        self.active_robots.append(msg.data)
        self.create_robot(msg.data)

    def removed_robot_callback(self, msg: Int32):
        self.active_robots.remove(msg.data)

    def calc_positions(self):

        """
        Aim of function: take a list of circle centres from the circle detection, and then calculate their position in real space using
        the info we know about the position of the camera and its resolution and aperture angle etc.

        The estimation may require a fair amount of tuning

        """

        self.vectors_to_robots = self.centres_list

        # for centre in self.centres_list: ## BUGFIX: WHEN ONLY ONE ROBOT, WILL CYCLE THROUGH SCALAR VALUES
        #     # method of working it out from the centre of the image means the vector is relative to directly below the camera

        #     vert_pixels_from_centre = centre[1] - (self.vert_res/2)
        #     horiz_pixels_from_centre = centre[2] - (self.horiz_res/2)

        #     print(vert_pixels_from_centre, horiz_pixels_from_centre)

        #     vert_pos = vert_pixels_from_centre / (self.vert_res/2) * (self.vert_dist_ground/2)
        #     horiz_pos = horiz_pixels_from_centre / (self.horiz_res/2) * (self.horiz_dist_ground/2)

        #     vector_pos = [centre[0],horiz_pos,vert_pos, centre[3]]

        #     self.vectors_to_robots.append(vector_pos)
 
    def publish_positions(self):

        for pos in self.vectors_to_robots:

            positions = Pose()

            positions.position.x = -pos[2] # could potentially be an earlier maths error, but just this works
            positions.position.y = -pos[1]
            positions.orientation.z = pos[3]

            # print(pos[0])

            try: # see if there is a publishable node for that ID number
                self.pub_dict[pos[0]].publish(positions)

            except KeyError : # if the key doesn't exist than a new publisher with that key is created.
                print(f'No publisher for robot{pos[0]}')

            middle = Pose()

            middle.position.x = pos[4][0]
            middle.position.y = pos[4][1]
            middle.orientation.z = pos[3]

            try: 
                self.middle_pubs[pos[0]].publish(middle)

            except KeyError:
                print(f"No publisher for robot{pos[0]}")
            
            
    def create_robot(self,ID):
        """
        As the Aruco gives an ID number, a dictionary containing all the publishing objects is created, with each robots publisher as the key
        """
        pub_name = f"/robot{ID}/pose"
        mid_name = f"/robot{ID}/middle"
        self.pub_dict[ID] = self.create_publisher(Pose, pub_name, 10)
        self.middle_pubs[ID] = self.create_publisher(Pose,mid_name,10)
        # self.pub_dict[ID] = rospy.Publisher(pub_name,Pose,queue_size=1) 

    def shutdown_callback(self):

        print("shutdown")

        self.cam.release()

        cv2.destroyAllWindows()

    def origin_callback(self, msg: Vector3):
        self.origin = msg

    def calibrate_cam_pos(self):

        """Function to map coordinates to a 2D floor plan of the room by performing a transform from image to perfect image view"""

        perf_img_points = np.array([]) # where the markers would be if the caemra was positioned perfectly.
        cam_points = np.array([]) # points where markers are detect

        self.h, status = cv2.findHomography(perf_img_points,cam_points) # gives homographic transform matrix

    def transform_pos(self):

        self.middles_on_floor = []

        for middle in self.middles :
            self.middles_on_floor.append(cv2.perspectiveTransform(middle,self.h))  

def main():
    rclpy.init()

    device = 0
    if len(sys.argv) > 1:
        device = int(sys.argv[1])

    aruco_tacker = ArucoTrack(device)
    
    rclpy.spin(aruco_tacker)
    rclpy.shutdown()

if __name__ == "__main__":
    main()