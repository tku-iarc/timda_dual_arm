#!/usr/bin/env python

from enum import IntEnum
# from Queue import Queue

import rospy
#import queue
import Queue as queue
import copy
import numpy as np
from std_msgs.msg import Bool, Int32
from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status, RobotiqGripper
from get_image_info import GetObjInfo
from math import radians, degrees, sin, cos, pi


# c_pose = {'left' :[[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
#                     [[0.38,  0.2, -0.25],  [0.0, 65, 0.0]],
#                     [[0.38,  0.2, -0.65],    [0.0, 65, 0.0]]],
#           'right':[[[0.38, -0.2, 0.15],  [0.0, 65, 0.0]],
#                     [[0.38, -0.2, -0.25],  [0.0, 65, 0.0]],
#                     [[0.38, -0.2, -0.65],    [0.0, 65, 0.0]]],
#           'left_indx' : 0, 'right_indx' : 0}
#-0.20
#0.1363 
#-0.38000
#44.024
#-0.005
#-44.998
#[-0.1100, 0.2363, -0.6000], [44.024, -0.005, -44.998]
#[-0.1100, 0.1363, -0.7000], [44.024, -0.005, -44.998]

#rag gripper 160
#[x,y,z],[-45,45,0]

#[0.2000, 0.2463, -0.7000], [0.000, 0.000, 0.000]
#[0.4000, 0.2463, -0.6500], [45.000, 90.000, 0.000]
#[0.64000, 0.2463, -0.6500], [45.000, 90.000, 0.000]

c_pose = {'left' :[[[0.2000, 0.2463, -0.7000], [0.000, 0.000, 0.000]],
                    [[0.4000, 0.2463, -0.6500], [45.000, 90.000, 0.000]],
                    [[0.64000, 0.2463, -0.6500], [45.000, 90.000, 0.000]],
                    [[0.64000, 0.2463, -0.6500], [45.000, 90.000, 0.000]],
                    [[0.3500, 0.2463, -0.6000], [45.000, 90.000, 0.000]],
                    [[0.200, 0.2463, -0.7000], [0.000, 0.000, 0.000]]],
          'right':[[[-0.30, -0.2463, -0.6500],  [0.000, 0.000, 0.000]],
                    [[-0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[-0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.00, -0.3463, -0.47500],  [0.000, 0.000, 0.000]],
                    [[0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.15, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.30, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.15, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.20, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.20, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
                    [[0.10, -0.4463, -0.6500],  [0.000, 0.000, 0.000]]],
          'left_indx' : 0, 'right_indx' : 0}
# place_pose = [[[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.796],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.796],[0.0, 0.0, 0.0]],
#               [[-0.38,  0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.38,  -0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.43,  -0.02, -0.73],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0.02, -0.73],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.68],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.68],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.68],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.38,  0, -0.7],[0.0, 0.0, 0.0]],
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]],                             
#               [[-0.43,  0, -0.7],[0.0, 0.0, 0.0]]]
# obj_pose = [[[[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]],
#             [[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]]],
#             [[[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]],
#             [[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]]],
#             [[[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]],
#             [[-0.20, 0.0363, -0.47050],[44.024, -0.005, -44.998]]]]

class ObjInfo(dict):
    def __init__(self):
        self['id']      = 0
        self['side_id'] = 'front'         # 'front', 'back', 'side'
        self['name']    = 'alcohol' # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
        self['state']   = 'new'           # 'new', 'old', 'expired'
        self['pos']     = None
        self['euler']   = None
        self['sucang']  = 0

class State(IntEnum):
    init            = 0
    get_ready       = 1
    apporach_people = 2
    move2people     = 3
    release_obj     = 4
    move_back       = 5
    # safety_back     = 8
    finish_init     = 6
   
class WipeTask:
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.state = State.init
        print("1")
        self.dual_arm = DualArmTask(self.name, self.en_sim)
        print("2")
        # self.camara = GetObjInfo()
        self.left_cpose_queue = queue.Queue()
        self.right_cpose_queue = queue.Queue()
        self.place_pose_queue = queue.Queue()
        self.object_queue = queue.Queue()
        self.object_list = []
        self.left_tar_obj = queue.Queue()
        self.right_tar_obj = queue.Queue()
        self.retry_obj_queue_left = queue.Queue()
        self.retry_obj_queue_right = queue.Queue()
        self.target_obj_queue = {'left' : self.left_tar_obj, 'right' : self.right_tar_obj}
        self.target_obj = {'left': None, 'right': None}
        self.retry_obj_queue = {'left': self.retry_obj_queue_left, 'right': self.retry_obj_queue_right}
        self.obj_done = np.zeros((100), dtype=bool)
        self.obj_retry = np.zeros((100), dtype=bool)
        self.next_level = {'left': False, 'right': False}
        print("3")
        #self.init()
    

    def state_control(self, state, side):
 
        print('START', state, side)
        if state is None:
            state = State.init
        elif state == State.init:
            print("8")
            state = State.get_ready

        elif state == State.get_ready:
            state = State.apporach_people

        elif state == State.apporach_people:
            state = State.move2people
            # state = State.grap_obj

        elif state == State.move2people:
            state = State.release_obj

        elif state == State.release_obj:
            if side == 'left':
                is_grip = self.dual_arm.left_arm.gripper.is_grip
            else:
                is_grip = self.dual_arm.right_arm.gripper.is_grip
            state = State.move_back

        elif state == State.move_back:
            if c_pose[side+'_indx'] >= 6:
                state = State.finish_init
            else:
                state = State.move_back

        elif state == State.finish_init:
            state = None
        print('END', state)
        return state
     
    def strategy(self, state, side):
        print("strategy", state)
        print("5")
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            print("6")
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'grip_bottle'
            #cmd['cmd'] = 'occupied'
            cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 1]
            cmd['state'] = State.init
            cmd['speed'] = 40
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            
        elif state == State.get_ready:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.get_ready
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')

        elif state == State.apporach_people:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.apporach_people
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')

        elif state == State.move2people:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            #cmd['suc_cmd'] = 0
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.move2people
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')          

        elif state == State.release_obj:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            if side == 'left':
                cmd['gripper_cmd'] = 'open'
            else:
                cmd['gripper_cmd'] = 'grap_rag'
            cmd['state'] =  State.release_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.move_back:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            if side == 'left':
                cmd['gripper_cmd'] = 'open'
            else:
                cmd['gripper_cmd'] = 'grap_rag'
            cmd['state'] = State.move_back
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')       

        elif state == State.finish_init:
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'reset'
            cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.finish_init
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        print("7")
        return side

    def process(self):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            #print("8")
            l_status = self.dual_arm.left_arm.status
            # print(l_status)
            if l_status == Status.idle or l_status == Status.occupied:
                l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                self.strategy(l_state, 'left')
            rate.sleep()
            #==============================================================================
            # r_status = self.dual_arm.right_arm.status
            # if r_status == Status.idle or r_status == Status.occupied:
            #     r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
            #     self.strategy(r_state, 'right')
            # rate.sleep()
            # if l_state is None and r_state is None:
            #     if l_status == Status.idle and r_status == Status.idle:
            #         return
            if l_state is None :
                if l_status == Status.idle:
                    return
            # # if r_state is None :
            # #     if r_status == Status.idle:
            # #         return
if __name__ == '__main__':
    rospy.init_node('wiped')

    strategy = WipeTask('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    print("4")
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
