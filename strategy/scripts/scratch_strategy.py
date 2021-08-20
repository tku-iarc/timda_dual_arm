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

c_pose = {'left' :[[[-0.20, 0.1363, -0.38000],  [44.024, -0.005, -44.998]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.00, 0.2363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.0363, -0.47500],  [44.024, -0.005, -44.998]],
                    [[0.1807, 0.4337, -0.6569],  [19.196, 0.427, -0.086]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]]],
          'right':[[[-0.16, -0.2863, -0.6500],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-44.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.47500],  [-44.024, 0.005, 4.498]],
                    [[0.60, -0.1820, -0.35500],  [46.024, 20.005, 4.998]],
                    [[0.20, -0.1820, -0.47500],  [46.024, 20.005, 4.998]],
                    [[0.50, 0.1063, -0.35500],  [46.024, 20.005, 4.998]],
                    [[0.20, 0.1063, -0.47500],  [46.024, 20.005, 4.998]],
                    [[0.20, -0.2463, -0.35500], [46.024, 20.005, 4.998]],
                    [[-0.16, -0.1820, -0.6500],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.1820, -0.7600],  [-42.024, 0.005, 4.498]],
                    [[-0.16, -0.2820, -0.6000],  [-42.024, 0.005, 4.498]]],
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
        self['name']    = 'scratcher' # 'plum_riceball', 'salmon_riceball', 'sandwich', 'burger', 'drink', 'lunch_box'
        self['state']   = 'new'           # 'new', 'old', 'expired'
        self['pos']     = None
        self['euler']   = None
        self['sucang']  = 0

class State(IntEnum):
    init            = 0
    apporach_obj    = 1
    move2obj        = 2
    grap_obj        = 3
    move_to_clean   = 4
    arrive_clean    = 5
    scratch         = 6
    finish          = 7
    safety_back     = 8
    finish_init     = 9
   
class ScratchTask:
    def __init__(self, _name, en_sim):
        self.name = _name
        self.en_sim = en_sim
        self.state = State.init
        self.dual_arm = DualArmTask(self.name, self.en_sim)
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
        #self.init()
    
    # def init(self):
    #     for pose in place_pose:
    #         self.place_pose_queue.put(pose)

    # def get_obj_inf(self, side):
    #     fb = self.dual_arm.get_feedback(side)
    #     ids, mats, names, exps, side_ids = self.camara.get_obj_info(side, fb.orientation)
        
    #     if ids is None:
    #         return
    #     for _id, mat, name, exp, side_id in zip(ids, mats, names, exps, side_ids):
    #         obj = ObjInfo()
    #         obj['id'] = _id
    #         obj['name'] = name
    #         obj['expired'] = exp
    #         obj['side_id'] = side_id
    #         obj['pos'] = mat[0:3, 3]
    #         obj['vector'] = mat[0:3, 2]
    #         obj['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
    #         obj['euler']   = [roll, 90, 0]
    #         if obj['vector'][2] > -0.2:
    #             self.object_queue.put(obj)
    #             print('fuck+++++============--------------', obj['pos'])
    #         else:
    #             print('fuck < -0.2 ', obj['vector'])
    #         print('fuckkkkkkkkkkkkkkkkkkkkkkk', obj['id'])

    # def arrange_obj(self, side):
    #     pass

    # def check_pose(self, side):
    #     self.target_obj[side] = self.target_obj_queue[side].get()
    #     fb = self.dual_arm.get_feedback(side)
    #     ids, mats, _, _, _ = self.camara.get_obj_info(side, fb.orientation)
    #     if ids is None:
    #         return
    #     for _id, mat in zip(ids, mats):
    #         if _id == self.target_obj[side]['id']:
    #             self.target_obj[side]['pos'] = mat[0:3, 3]
    #             if mat[2, 2] > -0.1:
    #                 self.target_obj[side]['sucang'], roll = self.dual_arm.suc2vector(mat[0:3, 2], [0, 1.57, 0])
    #                 self.target_obj[side]['euler']   = [roll, 90, 0]
    #     pass

    def state_control(self, state, side):
 
        print('START', state, side)
        if state is None:
            state = State.init
        elif state == State.init:
            state = State.apporach_obj

        elif state == State.apporach_obj:
            state = State.move2obj

        elif state == State.move2obj:
            state = State.grap_obj

        elif state == State.grap_obj:
            if side == 'left':
                is_grip = self.dual_arm.left_arm.gripper.is_grip
            else:
                is_grip = self.dual_arm.right_arm.gripper.is_grip
            if is_grip:
                state = State.move_to_clean
            elif self.next_level[side] == True:
                self.next_level[side] = False
                if c_pose[side+'_indx'] >= 3:
                    state = State.finish
                else:
                    state = State.init
            state = State.move_to_clean

        elif state == State.move_to_clean:
            # print(c_pose[side+'_indx'])
            if c_pose[side+'_indx'] >= 8:
                    state = State.finish
            else:
                if c_pose[side+'_indx'] %2 == 1:
                    state = State.arrive_clean
                else:
                    state = State.move_to_clean

        elif state == State.arrive_clean:
            if side == 'left':
                state = State.move_to_clean
            else:
                state = State.scratch

        elif state == State.scratch:
            print(c_pose[side+'_indx'])
            if c_pose[side+'_indx'] >= 11:
                state = State.finish
            else:
                    state = State.scratch

        elif state == State.finish:
            state = State.safety_back

        elif state == State.safety_back:
            state = State.finish_init

        elif state == State.finish_init:
            state = None
        print('END', state)
        return state
     
    def strategy(self, state, side):
        print("strategy", state)
        cmd = Command()
        cmd_queue = queue.Queue()
        if state == State.init:
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'active'
            cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.init
            cmd['speed'] = 20
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, True, cmd_queue)
            
        elif state == State.apporach_obj:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            # if side == 'left':
            #     cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            # else:
            #     cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.apporach_obj
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')

        elif state == State.move2obj:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            #cmd['suc_cmd'] = 0
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.move2obj
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')          

        elif state == State.grap_obj:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            if side == 'left':
                cmd['gripper_cmd'] = 'grap_scratcher'
            if side == 'right':
                cmd['gripper_cmd'] = 'grap_scratcher'
            cmd['state'] =  State.grap_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.move_to_clean:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            # if side == 'left':
            #     cmd['gripper_cmd'] = 'grap_alcohol'
            # else:
            #     cmd['gripper_cmd'] = 'grap_rag'
            cmd['state'] = State.move_to_clean
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')       


        elif state == State.arrive_clean:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            #print(c_pose[side][c_pose[side+'_indx']][0])
            cmd_queue.put(copy.deepcopy(cmd))
            #cmd['suc_cmd'] = 0
            # cmd['cmd'] = 'occupied'
            # if side == 'left':
            #     cmd['gripper_cmd'] = 'squeeze'
            cmd['state'] = State.arrive_clean
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')   

        elif state == State.scratch:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['state'] = State.scratch
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.finish:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['gripper_cmd'] = 'open'
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')   

        elif state == State.safety_back:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['gripper_cmd'] = 'open'
            cmd['state'] = State.safety_back
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')   

            # cmd['cmd'] = 'jointMove'
            # cmd['gripper_cmd'] = 'open'
            # cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 0]
            # cmd['state'] = State.safety_back
            # cmd_queue.put(copy.deepcopy(cmd))
            # self.dual_arm.send_cmd(side, False, cmd_queue)

        elif state == State.finish_init:
            cmd['cmd'] = 'jointMove'
            cmd['gripper_cmd'] = 'reset'
            cmd['jpos'] = [0, 0, 0, 0, 0, 0, 0, 0]
            cmd['state'] = State.finish_init
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        return side

    def process(self):
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            # l_status = self.dual_arm.left_arm.status
            # if l_status == Status.idle or l_status == Status.occupied:
            #     l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
            #     self.strategy(l_state, 'left')
            # rate.sleep()
            #==============================================================================
            r_status = self.dual_arm.right_arm.status
            if r_status == Status.idle or r_status == Status.occupied:
                r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                self.strategy(r_state, 'right')
            # rate.sleep()
            # if l_state is None and r_state is None:
            #     if l_status == Status.idle and r_status == Status.idle:
            #         return
            # if l_state is None :
            #     if l_status == Status.idle:
            #         return
            if r_state is None :
                if r_status == Status.idle:
                    return
if __name__ == '__main__':
    rospy.init_node('scratch')

    strategy = ScratchTask('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
