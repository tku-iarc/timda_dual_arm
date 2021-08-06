#!/usr/bin/env python
import os
import time
import numpy as np
import rospy
import rospkg
import tf
import ConfigParser
import enum
import argparse
import Queue as queue
import copy
#import Hiwin_RT605_Socket_test_andy as ArmTask
from hand_eye.srv import hand_eye_calibration, hand_eye_calibrationRequest
from manipulator_h_base_module_msgs.srv import GetKinematicsPose
from geometry_msgs.msg import Transform
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status
from enum import IntEnum



c_pose = {'left' :[[[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.19, 0.15],    [0.0, 65, 0.0]],
                    [[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.19, 0.15],    [0.0, 65, 0.0]]],
          'right':[[[0.36, -0.21, 0.14],  [-10.0, 62, 0.0]],
                    [[0.38, -0.22, 0.13],  [-5.0, 63, 0.0]],
                    [[0.40, -0.23, 0.12],    [0.0, 64, 0.0]],
                    [[0.42, -0.20, 0.11],  [5.0, 65, 0.0]],
                    [[0.40, -0.24, 0.10],  [10.0, 66, 0.0]],
                    [[0.38, -0.22, 0.09],    [15.0, 67, 0.0]]],
          'left_indx' : 0, 'right_indx' : 0}

class Arm_status(enum.IntEnum):
    Idle = 1
    Isbusy = 2

class State(enum.IntEnum):
    move = 0
    take_pic = 1
    finish = 2
    init = 3

class CameraCalib:
    def __init__(self, _name, en_sim):
        self.arm_move = False
        self.state = State.move
        self.is_done = {'left': False, 'right': False}
        self.name = _name
        self.en_sim = en_sim
        self.dual_arm = DualArmTask(self.name, self.en_sim)

    def hand_eye_client(self, req):
        rospy.wait_for_service('/camera/hand_eye_calibration')
        try:
            hand_eye = rospy.ServiceProxy('/camera/hand_eye_calibration', hand_eye_calibration)
            res = hand_eye(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_feedback(self, side):
        rospy.wait_for_service(side + '_arm/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                side + '_arm/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def state_control(self, state, side):
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.move
        elif state == State.move:
            state = State.take_pic
        elif state == State.take_pic:
            if self.is_done[side]:
                state = State.finish
            else:
                state = State.move
        elif state == State.finish:
            state = None
        return state

    def strategy(self, state, side):
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0]
            cmd['state'] = state
            cmd['speed'] = 40
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            print('state init')

        elif state == State.move: 
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['state'] = state
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] = c_pose[side+'_indx'] + 1 if c_pose[side+'_indx'] < 5 else 0
            else:
                print('fuckfailfuckfailfuckfail    ', cmd)
            print('state move')
            
        elif state == State.take_pic:
            print('state take_pic start')
            time.sleep(0.2)
            cmd['cmd'], cmd['state'] = None, state
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            arm_feedback = self.get_feedback(side)
            req = hand_eye_calibrationRequest()
            req.end_trans.translation.x = arm_feedback.group_pose.position.x
            req.end_trans.translation.y = arm_feedback.group_pose.position.y
            req.end_trans.translation.z = arm_feedback.group_pose.position.z
            req.end_trans.rotation.x    = arm_feedback.group_pose.orientation.x
            req.end_trans.rotation.y    = arm_feedback.group_pose.orientation.y
            req.end_trans.rotation.z    = arm_feedback.group_pose.orientation.z
            req.end_trans.rotation.w    = arm_feedback.group_pose.orientation.w
            res = self.hand_eye_client(req)
            if res.is_done:
                self.is_done[side] = True
                trans_mat = np.array(res.end2cam_trans).reshape(4,4)
                # camera_mat = np.array(res.camera_mat).reshape(4, 4)
                print('##################################################################')
                print(trans_mat)
                print('##################################################################')
                config = ConfigParser.ConfigParser()
                config.optionxform = str  #reference: http://docs.python.org/library/configparser.html
                # curr_path = os.path.dirname(os.path.abspath(__file__))
                # config.read(['img_trans_pinto.ini', curr_path])
                rospack = rospkg.RosPack()
                curr_path = rospack.get_path('hand_eye')
                config.read(curr_path + '/config/img_trans.ini')
                
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

            print('state take_pic end')

        elif state == State.finish:
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            print('state take_pic finish')

    def process(self, side):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        l_state = None
        r_state = None
        l_status = None
        r_status = None 
        while True:
            if side == 'left':
                l_status = self.dual_arm.left_arm.status
                if l_status == Status.idle or l_status == Status.occupied:
                    l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                    self.strategy(l_state, 'left')
            else:
                r_status = self.dual_arm.right_arm.status
                if r_status == Status.idle or r_status == Status.occupied:
                    r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                    self.strategy(r_state, 'right')
            rate.sleep()
            if l_state is None and r_state is None:
                if l_status == Status.idle and r_status == Status.idle:
                    return

if __name__ == '__main__':
    rospy.init_node('hand_eye_calibration_')
    en_sim = rospy.get_param('~en_sim')
    side = rospy.get_param('~side')
    strategy = CameraCalib('dual_arm', en_sim)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process(side)
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
