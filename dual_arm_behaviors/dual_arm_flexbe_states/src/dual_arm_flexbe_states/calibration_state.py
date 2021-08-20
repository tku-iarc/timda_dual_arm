#!/usr/bin/env python
import time
import rospkg, rospy
import numpy as np
import ConfigParser
from flexbe_core import EventState, Logger
from arm_control import Command
from flexbe_core.proxy import ProxyServiceCaller
from hand_eye.srv import hand_eye_calibration, hand_eye_calibrationRequest
from manipulator_h_base_module_msgs.srv import GetKinematicsPose

class ComputeCalibrationState(EventState):
    """
    Publishes a pose from userdata so that it can be displayed in rviz.

    -- robot_name              string          Robots name to move

    <= done									   Calib done.
    <= fail							    	   Calib fail.

    """
	
    def __init__(self, robot_name):
        """Constructor"""
        super(ComputeCalibrationState, self).__init__(outcomes=['done', 'fail'])
        self.robot_name = robot_name
        self.move_mode = 'p2p'
        self.pose_indx = 0
        self.compute_calibration_service = '/camera/compute_calibration'
        self.hand_eye_client = ProxyServiceCaller({self.compute_calibration_service: hand_eye_calibration})
        self.get_feedback_service = self.robot_name +'/get_kinematics_pose'
        self.get_feedback_client = ProxyServiceCaller({self.get_feedback_service: GetKinematicsPose})

    def execute(self, _):
        req = hand_eye_calibrationRequest()
        req.is_done = True

        if not self.hand_eye_client.is_available(self.compute_calibration_service):
            rospy.logerr('compute_calibration_service is not available!')
            return 'fail'
        try:
            res = self.hand_eye_client.call(self.compute_calibration_service, req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        if len(res.end2cam_trans) == 16:
            trans_mat = np.array(res.end2cam_trans).reshape(4,4)
            # camera_mat = np.array(res.camera_mat).reshape(4, 4)
            print('##################################################################')
            print(trans_mat)
            print('##################################################################')
            config = ConfigParser.ConfigParser()
            config.optionxform = str
            rospack = rospkg.RosPack()
            hand_eye_path = rospack.get_path('hand_eye')
            config.read(hand_eye_path + '/config/' + self.robot_name + '_img_trans.ini')
            config.set("External", "Key_1_1", str(trans_mat[0][0]))
            config.set("External", "Key_1_2", str(trans_mat[0][1]))
            config.set("External", "Key_1_3", str(trans_mat[0][2]))
            config.set("External", "Key_1_4", str(trans_mat[0][3]))
            config.set("External", "Key_2_1", str(trans_mat[1][0]))
            config.set("External", "Key_2_2", str(trans_mat[1][1]))
            config.set("External", "Key_2_3", str(trans_mat[1][2]))
            config.set("External", "Key_2_4", str(trans_mat[1][3]))
            config.set("External", "Key_3_1", str(trans_mat[2][0]))
            config.set("External", "Key_3_2", str(trans_mat[2][1]))
            config.set("External", "Key_3_3", str(trans_mat[2][2]))
            config.set("External", "Key_3_4", str(trans_mat[2][3]))
            config.write(open(hand_eye_path + '/config/' + self.robot_name + '_img_trans.ini', 'wb'))
            rospy.loginfo('fuck')
        else:
            rospy.logerr('calibration compute fail')
            return 'fail'
        return 'done'

    def on_enter(self, _):
        time.sleep(0.2)
        