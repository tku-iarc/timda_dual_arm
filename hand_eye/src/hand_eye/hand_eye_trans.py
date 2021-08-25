#!/usr/bin/env python
import os
import numpy as np
import rospy
import rospkg
import tf
import ConfigParser
from math import asin, atan2, degrees, pi
from hand_eye.srv import eye2base, eye2baseResponse
from manipulator_h_base_module_msgs.srv import GetKinematicsPose

class HandEyeTrans:
    def __init__(self, robot_name, camera_id):
        self.robot_name = robot_name
        self.camera_id = camera_id
        self._img_pose = np.zeros(6)
        self._base_pose = np.zeros(6)
        self._curr_pose = np.zeros(6)
        self._tool_coor = np.zeros(6)
        self._base_coor = np.zeros(6)
        self._base_tool_trans =  np.mat(np.identity(4))
        self._rbase_base_trans = np.mat(np.identity(4))
        self._rtool_tool_trans = np.mat(np.identity(4))
        self._hand_eye_trans =   np.mat(np.identity(4))
        self._rtool_eye_trans, self._camera_mat = self.__get_camera_param()

        self.__eye2base_server = rospy.Service('eye2base',
                eye2base,
                self.__eye2base_transform
        )
        self.__pix2base_server = rospy.Service('pix2base',
                eye2base,
                self.__pix2base_transform
        )
        self.__eye_trans2base_server = rospy.Service('eye_trans2base',
                eye2base,
                self.__eye_trans2base_transform
        )

    def __get_feedback(self):
        rospy.wait_for_service('get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                'get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def __get_camera_param(self):
        config = ConfigParser.ConfigParser()
        config.optionxform = str
        rospack = rospkg.RosPack()
        curr_path = rospack.get_path('hand_eye')

        config.read(curr_path + '/config/camera_' + str(self.camera_id) + '_internal.ini')
        config.read(curr_path + '/config/' + self.robot_name + '_img_trans.ini')
        
        a00 = float(config.get("External", "Key_1_1"))
        a01 = float(config.get("External", "Key_1_2"))
        a02 = float(config.get("External", "Key_1_3"))
        a03 = float(config.get("External", "Key_1_4"))
        a10 = float(config.get("External", "Key_2_1"))
        a11 = float(config.get("External", "Key_2_2"))
        a12 = float(config.get("External", "Key_2_3"))
        a13 = float(config.get("External", "Key_2_4"))
        a20 = float(config.get("External", "Key_3_1"))
        a21 = float(config.get("External", "Key_3_2"))
        a22 = float(config.get("External", "Key_3_3"))
        a23 = float(config.get("External", "Key_3_4"))

        config.read(curr_path + '/config/camera_' + str(self.camera_id) + '_internal.ini')
        color_width = rospy.get_param('~color_width')
        color_high = rospy.get_param('~color_high')
        internal_name = 'Internal_' + str(color_width) + '_' + str(color_high)

        b00 = float(config.get(internal_name, "Key_1_1"))
        b01 = float(config.get(internal_name, "Key_1_2"))
        b02 = float(config.get(internal_name, "Key_1_3"))
        b10 = float(config.get(internal_name, "Key_2_1"))
        b11 = float(config.get(internal_name, "Key_2_2"))
        b12 = float(config.get(internal_name, "Key_2_3"))
        b20 = float(config.get(internal_name, "Key_3_1"))
        b21 = float(config.get(internal_name, "Key_3_2"))
        b22 = float(config.get(internal_name, "Key_3_3"))
        
        Ex = np.mat([[a00, a01, a02, a03],
                     [a10, a11, a12, a13],
                     [a20, a21, a22, a23],
                     [0,   0,   0,   1]])
        In = np.mat([[b00, b01, b02],
                     [b10, b11, b12],
                     [b20, b21, b22]])
        print(Ex,In)
        return Ex, In

    def __get_robot_trans(self):
        res = self.__get_feedback()
        self._base_tool_trans = np.mat(res.orientation).reshape(4, 4)

    def __rotation2rpy(self, rotation):
        _rpy = []
        _rpy.append(degrees(asin(rotation[1, 2])))
        _rpy.append(degrees(atan2(rotation[0, 2], -rotation[2, 2])))
        _rpy.append(degrees(atan2(-rotation[1, 0], -rotation[1, 1])))
        return _rpy
        
    def __eye2base_transform(self, req):
        self.__get_robot_trans()
        assert len(req.ini_pose) == 3
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
        # eye_obj_trans[:3] = np.multiply(eye_obj_trans[:3], 0.01)
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.trans = np.mat(np.identity(4))
        res.trans[2, :] = result 
        res.pos = np.array(result[:3]).reshape(-1)
        res.euler = self.__rotation2rpy(res.trans)
        return res

    def __eye_trans2base_transform(self, req):
        self.__get_robot_trans()
        assert len(req.ini_pose) == 16
        eye_obj_trans = np.mat(req.ini_pose).reshape(4, 4)
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.trans = np.array(result).reshape(-1)
        res.pos = np.array(result[0:3, 3]).reshape(-1)
        res.euler = self.__rotation2rpy(result)
        return res
        

    def __pix2base_transform(self, req):
        self.__get_robot_trans()
        assert len(req.ini_pose) == 3
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
        # eye_obj_trans[2] = eye_obj_trans[2] * 0.01
        eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
        eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.trans = np.mat(np.identity(4))
        res.trans[2, :] = result
        res.pos = np.array(result[:3]).reshape(-1)
        res.euler = self.__rotation2rpy(res.trans)
        return res

if __name__ == "__main__":
    rospy.init_node('hand_eye_trans')
    robot_name = rospy.get_param('~robot_name')
    camera_id = rospy.get_param('~camera_id')
    worker = HandEyeTrans(robot_name, camera_id)
    rospy.spin()



