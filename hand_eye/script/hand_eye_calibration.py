#!/usr/bin/env python3
import os
import time
import numpy as np
import rospy
import tf
import ConfigParser
import enum
import argparse
import queue
import copy
#import Hiwin_RT605_Socket_test_andy as ArmTask
from control_node import HiwinRobotInterface
from hand_eye.srv import hand_eye_calibration, hand_eye_calibrationRequest
from geometry_msgs.msg import Transform
from math import radians, degrees, pi
from std_msgs.msg import String, Bool
from std_msgs.msg import Bool, Int32
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status


c_pose = {'left' :[[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, -0.25],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, -0.65],    [0.0, 65, 0.0]]],
          'right':[[[0.38, -0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38, -0.2, -0.25],  [0.0, 65, 0.0]],
                    [[0.38, -0.2, -0.65],    [0.0, 65, 0.0]]],
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

    def get_curr_pos(self, side):
        pose = get_feedback(side)
        res = np.array([])
        res = np.append(res, pose.group_pose.position.x)
        res = np.append(res, pose.group_pose.position.y)
        res = np.append(res, pose.group_pose.position.z)
        res = np.append(res, pose.group_pose.orientation.x)
        res = np.append(res, pose.group_pose.orientation.y)
        res = np.append(res, pose.group_pose.orientation.z)
        res = np.append(res, pose.group_pose.orientation.w)
        return res

    def hand_eye_client(self, req):
        rospy.wait_for_service('/camera/hand_eye_calibration')
        try:
            hand_eye = rospy.ServiceProxy('/camera/hand_eye_calibration', hand_eye_calibration)
            res = hand_eye(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def state_control(self, state, side):
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.move
        elif state == State.move:
            state = State.take_pic
        elif state == State.take_pic:
            if self.is_done:
                state = State.finish
            else:
                state = State.move
        elif state == State.finish:
            state = None

    def strategy(self, state, side):
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0]
            cmd['state'] = State.init
            cmd['speed'] = 40
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)

        elif state == State.move: 
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['state'] = state
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] += 1
            else:
                print('fuckfailfuckfailfuckfail')
            
        elif state == State.take_pic:
            time.sleep(0.2)
            pose = get_feedback(side)
            req = hand_eye_calibrationRequest()
            req.end_trans.translation.x = pose.group_pose.position.x
            req.end_trans.translation.y = pose.group_pose.position.y
            req.end_trans.translation.z = pose.group_pose.position.z
            req.end_trans.rotation.x    = pose.group_pose.orientation.x
            req.end_trans.rotation.y    = pose.group_pose.orientation.y
            req.end_trans.rotation.z    = pose.group_pose.orientation.z
            req.end_trans.rotation.w    = pose.group_pose.orientation.w
            res = self.hand_eye_client(req)
            if res.is_done:
                self.is_done[side] = True
                trans_mat = np.array(res.end2cam_trans).reshape(4,4)
                camera_mat = np.array(res.camera_mat).reshape(4, 4)
                print('##################################################################')
                print(trans_mat)
                print('##################################################################')
                config = ConfigParser.ConfigParser()
                config.optionxform = str  #reference: http://docs.python.org/library/configparser.html
                curr_path = os.path.dirname(os.path.abspath(__file__))
                # config.read(['img_trans_pinto.ini', curr_path])
                config.read(curr_path + '/img_trans.ini')
                
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

                config.set("Internal", "Key_1_1", str(camera_mat[0][0]))
                config.set("Internal", "Key_1_2", str(camera_mat[0][1]))
                config.set("Internal", "Key_1_3", str(camera_mat[0][2]))
                config.set("Internal", "Key_2_1", str(camera_mat[1][0]))
                config.set("Internal", "Key_2_2", str(camera_mat[1][1]))
                config.set("Internal", "Key_2_3", str(camera_mat[1][2]))
                config.set("Internal", "Key_3_1", str(camera_mat[2][0]))
                config.set("Internal", "Key_3_2", str(camera_mat[2][1]))
                config.set("Internal", "Key_3_3", str(camera_mat[2][2]))

        elif state == State.finish:
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)

    def process(self, side):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            if side == 'left':
                l_status = self.dual_arm.left_arm.status
                if l_status == Status.idle or l_status == Status.occupied:
                    l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                    self.strategy(l_state, 'left')
                rate.sleep()
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
    rospy.init_node('calibration_')
    side = 'left'
    strtage = CameraCalib('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process(side)
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
