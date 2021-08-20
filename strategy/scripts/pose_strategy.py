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

from strategy.srv import plot_position
from strategy.msg import QueSignal,TargetValue

global point_transformed, z_angel
point_transformed = []
z_angel = 0
global im_data
im_data = 0

def callback(data):
    im_data = data

def transformation(x, y, z):
    rotation_mat = np.array([[np.cos(15*np.pi/180), 0., np.sin(15*np.pi/180),-0.068],
                            [0., 1, 0, 0.08],
                            [-np.sin(15*np.pi/180), 0., np.cos(15*np.pi/180), 0.062],
                            [0.,0.,0.,1.]])#rotate 15-degree around x-axis
    # transfar_mat = np.array([[-0.14], [0.], [-0.21]])#[base-center to camera-center, 0, hieght]
    point = [[],[],[]]
    for i in range(3):
        # pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]]])) + transfar_mat
        pre_point = np.dot(rotation_mat, np.array([[x[i]], [y[i]], [z[i]], [1.0]]))
        point[i] = [pre_point[0].tolist(), pre_point[1].tolist(), pre_point[2].tolist()]
    return point

def add_two_ints_client():
    global point_transformed, z_angel
    print("go")
    # rospy.init_node('get_plot_pose')
    rospy.wait_for_service('show_figure')
    try:
        add_two_ints = rospy.ServiceProxy('show_figure', plot_position)
        done = True
        resp1 = add_two_ints(done)
        if resp1.is_done == True:
            x = [[resp1.head_minimum_z[0], resp1.pointing_minimum_z[0], resp1.target[0]]]
            y = [[resp1.head_minimum_z[1], resp1.pointing_minimum_z[1], resp1.target[1]]]
            z = [[resp1.head_minimum_z[2], resp1.pointing_minimum_z[2], resp1.target[2]]]
            for i in range(len(resp1.object)):
                if i%3 == 0: x.append(resp1.object[i])
                if i%3 == 1: y.append(resp1.object[i])
                if i%3 == 2: z.append(resp1.object[i])
            z_angel = resp1.angle
        # fig = plt.figure()
        # ax = plt.axes(projection='3d')
        # ax.plot3D(x[0], y[0], z[0])
        # ax.scatter3D(x[1:],y[1:],z[1:])
        # plt.show()

        point_transformed = transformation(x[0],y[0],z[0])
        # print(point_transformed)

        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
"""
def add_two_ints_client():
    global point_transformed
    print("go")
    # rospy.init_node('get_plot_pose')
    # rospy.wait_for_service('show_figure')
    
    try:
        pub = rospy.Publisher('receive_figure', QueSignal, queue_size=10)
        rate = rospy.Rate(5) # 10hz
        # done = QueSignal()
        while not rospy.is_shutdown():
            done = 0
            pub.publish(done)
            rate.sleep()
            pub.publish(done)
            rate.sleep()
            pub.publish(done)
            rate.sleep()
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def receive_pose_data(data):
    global point_transformed
    try:
        x,y,z = [],[],[]
        if data.is_done == True:
            x = [[data.head_minimum_z[0], data.pointing_minimum_z[0], data.target[0]]]
            y = [[data.head_minimum_z[1], data.pointing_minimum_z[1], data.target[1]]]
            z = [[data.head_minimum_z[2], data.pointing_minimum_z[2], data.target[2]]]
            for i in range(len(data.object)):
                if i%3 == 0: x.append(data.object[i])
                if i%3 == 1: y.append(data.object[i])
                if i%3 == 2: z.append(data.object[i])
            z_angel = data.angle
        #fig = plt.figure()
        #ax = plt.axes(projection='3d')
        #ax.plot3D(x[0], y[0], z[0])
        #ax.scatter3D(x[1:],y[1:],z[1:])
        #plt.show()

        point_transformed = transformation(x[0],y[0],z[0])
        # print(point_transformed)
        rospy.spin()
        return data
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
"""
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
# c_pose = {'left' :[[[-0.1000, 0.2363, -0.600],  [44.024, -0.005, -44.998]],
#                     [[-0.1000, 0.1313, -0.6800],  [44.024, -0.005, -44.998]],
#                     [[-0.1000, 0.1313, -0.6800],  [44.024, -0.005, -44.998]],
#                     [[-0.1000, 0.3363, -0.6000],  [44.024, -0.005, -44.998]],
#                     [[0.20, 0.3363, -0.5000],  [0.000, 0.000, 0.000]],
#                     [[0.20, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
#                     [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
#                     [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
#                     [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
#                     [[0.1807, 0.4337, -0.6569],  [19.196, 0.427, -0.086]],
#                     [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]]],
#           'right':[[[-0.30, -0.2463, -0.6500],  [0.000, 0.000, 0.000]],
#                     [[-0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[-0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.00, -0.3463, -0.47500],  [0.000, 0.000, 0.000]],
#                     [[0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.15, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.30, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.30, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.15, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.20, -0.2463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.20, -0.0463, -0.7500],  [0.000, 0.000, 0.000]],
#                     [[0.10, -0.4463, -0.6500],  [0.000, 0.000, 0.000]]],
#           'left_indx' : 0, 'right_indx' : 0}

obj_pose = [0,0,0,0,0,0]
place_pose = [obj_pose[0], obj_pose[1], obj_pose[2], 0.000, 0.000, 0.000]


c_pose = {'left' :[[[-0.1000, 0.2363, -0.600],  [44.024, -0.005, -44.998]],
                    [[-0.1000, 0.1313, -0.6800],  [44.024, -0.005, -44.998]],
                    [[-0.1000, 0.1313, -0.6800],  [44.024, -0.005, -44.998]],
                    [[-0.1000, 0.3363, -0.6000],  [44.024, -0.005, -44.998]],
                    [[0.20, 0.3363, -0.5000],  [0.000, 0.000, 0.000]],
                    [[0.20, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
                    [[0.30, 0.2363, -0.7000],  [44.024, -0.005, -44.998]],
                    [[0.1807, 0.4337, -0.6569],  [19.196, 0.427, -0.086]],
                    [[-0.20, 0.0363, -0.47500],  [44.024, -0.005, -44.998]]],
          'right':[[[0.10, -0.30, -0.40],  [0.000, 30.000, 0.000]],#start position(hands-up)
                    [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
                    [[obj_pose[0]-0.31*np.cos(obj_pose[3]), obj_pose[1]-0.31*np.sin(obj_pose[3]), obj_pose[2]+0.15],  [obj_pose[3], 45.000, 0.000]],#upper position(above object)
                    [[obj_pose[0], obj_pose[1], obj_pose[2]],  [obj_pose[3], 45.000, 0.000]],#grip position(grip object)
                    [[obj_pose[0], obj_pose[1], obj_pose[2]],  [obj_pose[3], 45.000, 0.000]],#griped
                    [[obj_pose[0], obj_pose[1], obj_pose[2]+10],  [obj_pose[3], 45.000, 0.000]],#lift position(directly above)
                    [[place_pose[0], place_pose[1], place_pose[2]+10],  [0.000, 45.000, 0.000]],#pre-place position
                    [[place_pose[0], place_pose[1], place_pose[2]],  [0.000, 45.000, 0.000]],#place position
                    [[place_pose[0], place_pose[1], place_pose[2]],  [0.000, 45.000, 0.000]],#placeed
                    [[place_pose[0], place_pose[1], place_pose[2]+10],  [0.000, 45.000, 0.000]],#pre-place position
                    [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
                    [[0.10, -0.30, -0.40],  [0.000, 30.000, 0.000]]],#start position(hands-up)
          'left_indx' : 0, 'right_indx' : 0}
# place_pose = [, 0.000, 45.000, 0.000]
# obj_pose = [point_transformed[2][0], point_transformed[2][1], point_transformed[2][2], z_angel, 45.0, 0]

#limit_height = 

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
    safety_up       = 1
    apporach_obj    = 2
    move2obj        = 3
    grab_obj        = 4
    grabed          = 5
    lift_up         = 6
    apporach_place  = 7
    place_obj       = 8
    placed          = 9
    above_obj       = 10
    safety_back     = 11
    finish_init     = 12
   
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
            print("8")
            state = State.safety_up

        elif state == State.safety_up:
            state = State.apporach_obj

        elif state == State.apporach_obj:
            state = State.move2obj

        elif state == State.move2obj:
            state = State.grab_obj

        elif state == State.grab_obj:
            # if side == 'left':
            #     is_grip = self.dual_arm.left_arm.gripper.is_grip
            # else:
            #     is_grip = self.dual_arm.right_arm.gripper.is_grip
            # if is_grip:
            #     state = State.safety_up
            # elif self.next_level[side] == True:
            #     self.next_level[side] = False
            #     if c_pose[side+'_indx'] >= 3:
            #         state = State.finish
            #     else:
            #         state = State.init
            state = State.grabed

        elif state == State.grabed:
            state = State.lift_up

        elif state == State.lift_up:
            # print(c_pose[side+'_indx'])
            state = State.apporach_place

        elif state == State.apporach_place:
            state = State.place_obj

        elif state == State.place_obj:
            # if side == 'left':
            #     is_grip = self.dual_arm.left_arm.gripper.is_grip
            # else:
            #     is_grip = self.dual_arm.right_arm.gripper.is_grip
            # if is_grip:
            #     state = State.place_obj
            # elif self.next_level[side] == True:
            #     self.next_level[side] = False
            #     if c_pose[side+'_indx'] >= 3:
            #         state = State.finish
            #     else:
            #         state = State.init
            state = State.placed

        elif state == State.placed:
            state = State.above_obj

        elif state == State.above_obj:
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
            
        elif state == State.safety_up:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.safety_up
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')

        elif state == State.apporach_obj:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
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

        elif state == State.grab_obj:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['state'] =  State.grab_obj
            #cmd['gripper_cmd'] = 'grap_pose_point'
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.grabed:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['gripper_cmd'] = 'grap_pose_point'
            cmd['state'] =  State.grabed
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail') 

        elif state == State.lift_up:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.lift_up
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')       

# if place point change more than two axis than > p2p 
        elif state == State.apporach_place:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            #print(c_pose[side][c_pose[side+'_indx']][0])
            cmd_queue.put(copy.deepcopy(cmd))
            #cmd['suc_cmd'] = 0
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.apporach_place
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')   

        elif state == State.place_obj:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['state'] = State.place_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.placed:
            #cmd['cmd'] = 'occupied'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd['gripper_cmd'] = 'open'
            cmd['state'] = State.placed
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] +=1
            else:
                print('fuckfailfuckfailfuckfail')  

        elif state == State.above_obj:
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.above_obj
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
            #cmd['gripper_cmd'] = 'open'
            cmd['state'] = State.safety_back
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
            # cmd['suc_cmd'] = 'Off'
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(side, False, cmd_queue)
        print("7")
        return side

    def process(self):
        global point_transformed
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.dual_arm.shutdown)
        while True:
            if point_transformed[2][1][0]>=0:
                print("8")
                l_status = self.dual_arm.left_arm.status
                # print(l_status)
                if l_status == Status.idle or l_status == Status.occupied:
                    l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                    self.strategy(l_state, 'left')
                rate.sleep()
                if l_state is None :
                    if l_status == Status.idle:
                        return

            #==============================================================================
            elif point_transformed[2][1][0]<0:
                print("82")
                r_status = self.dual_arm.right_arm.status
                if r_status == Status.idle or r_status == Status.occupied:
                    r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                    self.strategy(r_state, 'right')
                rate.sleep()
                if r_state is None :
                    if r_status == Status.idle:
                        return
            # if l_state is None and r_state is None:
            #     if l_status == Status.idle and r_status == Status.idle:
            #         return
            # if l_state is None :
            #     if l_status == Status.idle:
            #         return
            # if r_state is None :
            #     if r_status == Status.idle:
            #         return
if __name__ == '__main__':
    rospy.init_node('pose')
    # rospy.Subscriber("show_figure", TargetValue, receive_pose_data)
    add_two_ints_client()
# while(1):
#     if data.is_done == 0:

#         strategy = WipeTask('dual_arm', False)
#         rospy.on_shutdown(strategy.dual_arm.shutdown)
#         print("4")
#         strategy.process()
#         strategy.dual_arm.shutdown()
#         del strategy.dual_arm
#         is_done == False
#     else:
#         print("not receive data yet")
    obj_pose = [point_transformed[2][0][0], point_transformed[2][1][0], point_transformed[2][2][0], z_angel, 45.0, 0]
    place_pose = [obj_pose[0], obj_pose[1], obj_pose[2], 0.000, 45.000, 0.000]
    
    # c_pose = {'left' :[[[0.1000, 0.30, -0.5000],  [0, 0, 0]],
    #                     [[0.3000, 0.30, -0.2500],  [0, 45, 0]],
    #                     [[obj_pose[0]-0.15*np.cos(obj_pose[3]), obj_pose[1]+0.15*np.sin(obj_pose[3]), obj_pose[2]+0.10], [0, 45, obj_pose[3]*180/np.pi]],
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]],  [0, 45, obj_pose[3]*180/np.pi+45]],
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]],  [0, 45, obj_pose[3]*180/np.pi+45]],
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]+0.10],  [0, 45, obj_pose[3]*180/np.pi+45]],
    #                     [[place_pose[0], place_pose[1], place_pose[2]+0.10],  [0, 45, 0]],
    #                     [[place_pose[0], place_pose[1], place_pose[2]],  [0, 45, 0]],
    #                     [[place_pose[0], place_pose[1], place_pose[2]],  [0, 45, 0]],
    #                     [[place_pose[0], place_pose[1], place_pose[2]+0.10],  [0, 45, 0]],
    #                     [[0.3000, 0.30, -0.2500],  [0, 45, 0]],
    #                     [[0.1000, 0.30, -0.3000],  [0, 20, 0]]],
    #             'right':[[[-0.30, -0.2463, -0.6500],  [0.000, 0.000, 0.000]],#start position(hands-up)
    #                     [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
    #                     [[obj_pose[0]-0.15*np.cos(obj_pose[3]), obj_pose[1]-0.15*np.sin(obj_pose[3]), obj_pose[2]+0.10],  [0, 45.000, obj_pose[3]*180/np.pi]],#upper position(above object)
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]],  [0, 45.000, obj_pose[3]*180/np.pi]],#grip position(grip object)
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]],  [0, 45.000, obj_pose[3]*180/np.pi]],#griped
    #                     [[obj_pose[0], obj_pose[1], obj_pose[2]+0.10],  [0, 45.000, obj_pose[3]*180/np.pi]],#lift position(directly above)
    #                     [[place_pose[0], place_pose[1], place_pose[2]+0.10],  [0.000, 45.000, 0.000]],#pre-place position
    #                     [[place_pose[0], place_pose[1], place_pose[2]],  [0.000, 45.000, 0.000]],#place position
    #                     [[place_pose[0], place_pose[1], place_pose[2]],  [0.000, 45.000, 0.000]],#placeed
    #                     [[place_pose[0], place_pose[1], place_pose[2]+0.10],  [0.000, 45.000, 0.000]],#pre-place position
    #                     [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
    #                     [[0.10, -0.30, -0.40],  [0.000, 30.000, 0.000]]],#start position(hands-up)
    #             'left_indx' : 0, 'right_indx' : 0}

    c_pose = {'left' :[[[0.1000, 0.30, -0.5000],  [0, 0, 0]],
                        [[0.3000, 0.30, -0.2500],  [0, 45, 0]],
                        [[obj_pose[0]-0.10, obj_pose[1]+0.10, obj_pose[2]+0.10], [-30, 45, -10]],
                        [[obj_pose[0], obj_pose[1], obj_pose[2]],  [-90, 45, -45]],
                        [[obj_pose[0], obj_pose[1], obj_pose[2]],  [-90, 45, -45]],
                        [[obj_pose[0], obj_pose[1], obj_pose[2]+0.10],  [-90, 45, -45]],
                        [[0.60, 0.1363, -0.2500],  [-100, 55, -25]],
                        [[0.72, 0.1363, -0.2500],  [-120, 70, -15]],
                        [[0.72, 0.1363, -0.2500],  [-120, 70, -15]],
                        [[0.60, 0.1363, -0.2500],  [-100, 55, -25]],
                        [[0.3000, 0.30, -0.2500],  [0, 45, 0]],
                        [[0.1000, 0.30, -0.3000],  [0, 20, 0]]],
                'right':[[[-0.10, -0.30, -0.500],  [0.000, 0.000, 0.000]],#start position(hands-up)
                        [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
                        [[obj_pose[0]-0.10, obj_pose[1]-0.10, obj_pose[2]+0.10],  [40, 45.000, 10]],#upper position(above object)
                        [[obj_pose[0], obj_pose[1], obj_pose[2]],  [90, 45.000, 45]],#grip position(grip object)
                        [[obj_pose[0], obj_pose[1], obj_pose[2]],  [90, 45.000, 45]],#griped
                        [[obj_pose[0], obj_pose[1], obj_pose[2]+0.10],  [90, 45.000, 45]],#lift position(directly above)
                        [[0.60, -0.1363, -0.2500],  [100, 55, 25]],#pre-place position
                        [[0.72, -0.1363, -0.2500],  [120, 70, 15]],#place position
                        [[0.72, -0.1363, -0.2500],  [120, 70, 15]],#placeed
                        [[0.60, -0.1363, -0.2500],  [100, 55, 25]],#pre-place position
                        [[0.30, -0.30, -0.25],  [0.000, 45.000, 0.000]],#pre-upper position(extend hands)
                        [[0.10, -0.30, -0.40],  [0.000, 20.000, 0.000]]],#start position(hands-up)
                'left_indx' : 0, 'right_indx' : 0}
    print(c_pose['left'][2],z_angel)
    strategy = WipeTask('dual_arm', False)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    print("4")
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
