#!/usr/bin/env python

# The following code is used to watch a video stream, detect Aruco markers, and use
# a set of markers to determine the posture of the camera in relation to the plane
# of markers.
#
# Assumes that all markers are on the same plane, for example on the same piece of paper
#
# Requires camera calibration (see the rest of the project for example calibration)

import rospy
import std_msgs, std_srvs
import numpy as np
import cv2
import cv2.aruco as aruco
import os
import pickle
from aruco_hand_eye.srv import aruco_info, aruco_infoResponse
import time
import pyrealsense2 as rs
NUMBER = 5
# # Constant parameters used in Aruco methods
# ARUCO_PARAMETERS = aruco.DetectorParameters_create()
# ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
CHARUCOBOARD_ROWCOUNT=7
CHARUCOBOARD_COLCOUNT=5

# Create grid board object we're using in our stream
CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.0359,
        markerLength=0.0244,
        # squareLength=0.04,
        # markerLength=0.02,
        dictionary=ARUCO_DICT)



class CharucoBoardPosture():
    def __init__(self, name, size):
        self.name = name
        self.markersize = size
        self.cnd = 0
        self.frameId = 0
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        self.pipeline.start(config)
        # Check for camera calibration data
        c_x = 643.47548083
        c_y = 363.67742746
        f_x = 906.60886808
        f_y = 909.34831447
        k_1 = 0.16962942
        k_2 = -0.5560001
        p_1 = 0.00116353
        p_2 = -0.00122694
        k_3 = 0.52491878

        c_x = 649.007507324219
        c_y = 356.122222900391
        f_x = 922.76806640625
        f_y = 923.262023925781
    
        self.cameraMatrix = np.array([[f_x, 0, c_x],
                               [0, f_y, c_y],
                               [0, 0, 1]])
        # self.distCoeffs = np.array([k_1, k_2, p_1, p_2, k_3])
        self.distCoeffs = np.array([0.0, 0, 0, 0, 0])

        
        # self.cameraMatrix = np.array([[1.38726465e+03, 0.00000000e+00, 9.67009977e+02], #pinto
        #                               [0.00000000e+00, 1.39067726e+03, 5.44111718e+02],
        #                               [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        # self.distCoeffs = np.array([ 0.1772611,  -0.57056992, -0.0008356,   0.00099024,  0.52153116])

        # dist_coef = np.array([0, 0, 0, 0, 0])
        # self.cameraMatrix = np.array([[906.10541873,   0.0,          643.12531806],
                                    #   [0.0,            904.68643316, 359.79710938],
                                    #   [0.0,            0.0,          1.0         ]])
        # self.distCoeffs = np.array([1.53041876e-01, -4.08438606e-01,  1.53722452e-03, -3.95946669e-04, 2.56666605e-01])
        # if not os.path.exists('/home/iclab/wrs_ws/src/aruco_hand_eye/cfg/calibration.pckl'):
        #     print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
        #     self.cameraMatrix = [[603.00869939,   0.0,          318.46049727],
        #                          [0.0,            601.50770586, 251.87010006],
        #                          [0.0,            0.0,          1.0         ]]
        #     self.distCoeffs = [[7.59282092e-02,  2.21483627e-01,  1.41152268e-03, -4.71388619e-04, -1.18482976e+00]]

        #     #exit()
        # else:
        #     f = open('/home/iclab/wrs_ws/src/aruco_hand_eye/cfg/calibration.pckl', 'rb')
        #     (self.cameraMatrix, self.distCoeffs, _, _) = pickle.load(f)
        #     f.close()
        #     if self.cameraMatrix is None or self.distCoeffs is None:
        #         print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        #         exit()
        #     print(self.cameraMatrix)
        #     print('               ')
        #     print(self.distCoeffs)

        # Create grid board object we're using in our stream
        # board = aruco.GridBoard_create(
        #         markersX=2,
        #         markersY=2,
        #         markerLength=0.09,
        #         markerSeparation=0.01,
        #         dictionary=ARUCO_DICT)

        # Create vectors we'll be using for rotations and translations for postures
        self.rvecs = None 
        self.tvecs = None
        self.rvecs_arr = np.zeros((3, NUMBER))
        self.tvecs_arr = np.zeros((3, NUMBER))
        # cam = cv2.VideoCapture('gridboardiphonetest.mp4')
        # self.cam_left = cv2.VideoCapture(5)
        # self.cam_right = cv2.VideoCapture(10)
        self.cam = None
        self.QueryImg = None
        self.init_server()
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        self.QueryImg = np.asanyarray(color_frame.get_data())

    def init_server(self):
        self.server = rospy.Service('get_ar_marker', aruco_info, self.findCharucoBoard)

    def findCharucoBoard(self, req):
        self.rvecs_arr = np.zeros((3, NUMBER))
        self.tvecs_arr = np.zeros((3, NUMBER))
        res = aruco_infoResponse()

        for order in range (NUMBER):
            # Capturing each frame of our video stream
            # ret, self.QueryImg = self.cam.read()
            # frames = self.pipeline.wait_for_frames()
            # color_frame = frames.get_color_frame()
            # self.QueryImg = np.asanyarray(color_frame.get_data())
            # grayscale image
            gray = cv2.cvtColor(self.QueryImg, cv2.COLOR_BGR2GRAY)

            # Detect Aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)

            # Refine detected markers
            # Eliminates markers not part of our board, adds missing markers to the board
            corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                    image = gray,
                    board = CHARUCO_BOARD,
                    detectedCorners = corners,
                    detectedIds = ids,
                    rejectedCorners = rejectedImgPoints,
                    cameraMatrix = self.cameraMatrix,
                    distCoeffs = self.distCoeffs)   

            # Require 15 markers before drawing axis
            if ids is not None and len(ids) > 10:
                response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                        markerCorners=corners,
                        markerIds=ids,
                        image=gray,
                        board=CHARUCO_BOARD)

                # Require more than 20 squares
                if response is not None and response > 20:
                    # Estimate the posture of the charuco board, which is a construction of 3D space based on the 2D video 
                    pose, rvec, tvec = aruco.estimatePoseCharucoBoard(
                            charucoCorners=charuco_corners, 
                            charucoIds=charuco_ids, 
                            board=CHARUCO_BOARD, 
                            cameraMatrix=self.cameraMatrix, 
                            distCoeffs=self.distCoeffs)
                            
                    # self.rvecs, self.tvecs = aruco.estimatePoseSingleMarkers(corners, self.markersize, self.cameraMatrix, self.distCoeffs)
                    # for _id, rvec, tvec in zip(ids, self.rvecs, self.tvecs):
                    if pose:
                        if order == 0:
                            print("=============================================")
                            print(rvec)
                            print(tvec)
                        for i in range(3):
                            self.rvecs_arr[i][order] = rvec[i][0]
                            self.tvecs_arr[i][order] = tvec[i][0]
                    
                #     self.QueryImg = aruco.drawAxis(self.QueryImg, self.cameraMatrix, self.distCoeffs, rvec, tvec, 0.02)
            cv2.waitKey(10)
            # Display our image
        # print('self.rvecs_arr = ', self.rvecs_arr)
        # print('self.tvecs_arr = ', self.tvecs_arr)
        cv2.destroyAllWindows()
        r_avg = np.zeros(3) 
        t_avg = np.zeros(3)

        ra = self.rvecs_arr[0].nonzero()
        rb = self.rvecs_arr[1].nonzero()
        rc = self.rvecs_arr[2].nonzero()
        tx = self.tvecs_arr[0].nonzero()
        ty = self.tvecs_arr[1].nonzero()
        tz = self.tvecs_arr[2].nonzero()
        ra = self.rvecs_arr[0][ra]
        rb = self.rvecs_arr[1][rb]
        rc = self.rvecs_arr[2][rc]
        tx = self.tvecs_arr[0][tx]
        ty = self.tvecs_arr[1][ty]
        tz = self.tvecs_arr[2][tz]
        ra = np.sort(ra, kind = 'quicksort')
        rb = np.sort(rb, kind = 'quicksort')
        rc = np.sort(rc, kind = 'quicksort')
        tx = np.sort(tx, kind = 'quicksort')
        ty = np.sort(ty, kind = 'quicksort')
        tz = np.sort(tz, kind = 'quicksort')
        r = np.array((ra, rb, rc))
        t = np.array((tx, ty, tz))
        for i in range(3):
            rv, tv = r[i], t[i]
            
            while np.std(rv) > 0.01 and len(rv) >= NUMBER*0.2:
                if abs(rv[0] - np.average(rv)) > abs(rv[-1] - np.average(rv)):
                    rv = np.delete(rv, 0)
                else:
                    rv = np.delete(rv, -1)
            while np.std(tv) > 0.01 and len(tv) >= NUMBER*0.2:
                if abs(tv[0] - np.average(tv)) > abs(tv[-1] - np.average(tv)):
                    tv = np.delete(tv, 0)
                else:
                    tv = np.delete(tv, -1)
            
            r_avg[i] = np.average(rv)
            t_avg[i] = np.average(tv)
        
        # print('[_id, r,t] = ', [_id, r,t])
        # res.ids.append(_id)
        # res.rvecs = np.append(res.rvecs, r_avg)
        # res.tvecs = np.append(res.tvecs, t_avg)
        res.rvecs = r_avg
        res.tvecs = t_avg
        print('res.rvecs is ', res.rvecs)
        print('res.tvecs is ', res.tvecs)
        result = np.array(())
        result = np.append(result, [np.copy(r_avg), np.copy(t_avg)])
        
        result = result.reshape(2,3)

        if self.name == 'test':
            # Outline all of the markers detected in our image
            self.QueryImg = aruco.drawDetectedMarkers(self.QueryImg, corners, borderColor=(0, 0, 255))
            self.QueryImg = aruco.drawAxis(self.QueryImg, self.cameraMatrix, self.distCoeffs, result[0], result[1], 0.02)
            curr_path = os.path.dirname(os.path.abspath(__file__))
            filename = curr_path + "/pic/camera-pic-of-charucoboard-" +  str(int(self.frameId)) + ".jpg"
            cv2.imwrite(filename, self.QueryImg)
            self.frameId += 1
            # cv2.imwrite('./123%d.jpg'%self.cnd, self.QueryImg)
            # self.cnd += 1
            # cv2.namedWindow('Amanda', cv2.WINDOW_AUTOSIZE)
            # self.QueryImg = cv2.imread('./123%d.jpg'%self.cnd)
            # cv2.imshow('Amanda', self.QueryImg)
            # cv2.waitKey(1000)
            # cv2.destroyAllWindows()

            # time.sleep(2)
            print('------')
            # while not cv2.waitKey(1) & 0xFF == ord('q'):
            #     pass
            # cv2.destroyAllWindows()
        return res
            
if __name__ == '__main__':
    rospy.init_node('aruco_tracker')
    is_show = True
    name = 'test'
    size = rospy.get_param('~marker_size')
    if rospy.has_param('is_show'):
        is_show = rospy.get_param('is_show')
    if is_show == False:
        name = 'fuck_run'

    mp = CharucoBoardPosture(name, size)
    if mp.name == 'test':
        while mp.QueryImg is None:
            time.sleep(0.1)
        while not rospy.is_shutdown():
            frames = mp.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            mp.QueryImg = np.asanyarray(color_frame.get_data())
            cv2.imshow('Amanda', mp.QueryImg)
            cv2.waitKey(10)
    rospy.spin()
    cv2.destroyAllWindows()
    del mp
    # while True:
    #     result = mp.findCharucoBoard()
    #     print(result)
    #     print(cv2.Rodrigues(result[0][1])[0])
    #     # print('==========================')
    #     if cv2.waitKey(0) & 0xFF == ord('q'):
    #         break
    
