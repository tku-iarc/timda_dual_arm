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

class CaptureCharUcoState(EventState):
    """
    Publishes a pose from userdata so that it can be displayed in rviz.

    -- robot_name              string          Robots name to move

    <= done									   Calib done.
    <= fail							    	   Calib fail.

    """
	
    def __init__(self, robot_name):
        """Constructor"""
        super(CaptureCharUcoState, self).__init__(outcomes=['done', 'fail'])
        self.robot_name = robot_name
        self.move_mode = 'p2p'
        self.pose_indx = 0
        self.hand_eye_service = '/camera/hand_eye_calibration'
        self.hand_eye_client = ProxyServiceCaller({self.hand_eye_service: hand_eye_calibration})
        self.get_feedback_service = self.robot_name +'/get_kinematics_pose'
        self.get_feedback_client = ProxyServiceCaller({self.get_feedback_service: GetKinematicsPose})

    def execute(self, userdata):
        if not self.get_feedback_client.is_available(self.get_feedback_service):
            return 'fail'

        try:
            arm_feedback = self.get_feedback_client.call(self.get_feedback_service, 'arm')
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)
        req = hand_eye_calibrationRequest()
        req.end_trans.translation.x = arm_feedback.group_pose.position.x
        req.end_trans.translation.y = arm_feedback.group_pose.position.y
        req.end_trans.translation.z = arm_feedback.group_pose.position.z
        req.end_trans.rotation.x    = arm_feedback.group_pose.orientation.x
        req.end_trans.rotation.y    = arm_feedback.group_pose.orientation.y
        req.end_trans.rotation.z    = arm_feedback.group_pose.orientation.z
        req.end_trans.rotation.w    = arm_feedback.group_pose.orientation.w
        if not self.hand_eye_client.is_available(self.hand_eye_service):
            return 'fail'
        try:
            res = self.hand_eye_client.call(self.hand_eye_service, req)
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

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

        return 'done'

    def on_enter(self, userdata):
        time.sleep(0.2)
        