#!/usr/bin/env python

from sys import dont_write_bytecode
import rospy
import Queue as queue
# from Queue import Queue
import copy
import numpy as np
from enum import IntEnum
from math import radians, degrees, sin, cos, pi
import math

from std_msgs.msg import Bool, Int32

from arm_control import DualArmTask
from arm_control import ArmTask, SuctionTask, Command, Status, RobotiqGripper

from obj_info import ObjInfo
from get_image_info import GetObjInfo

'''
Plum Riceball:      ABCD (10~, 20~, 30~, 40~)   (5 sides)
Salmon Riceball:    EFGH (50~, 60~, 70~, 80~)   (5 sides)
Sandwich:           IJK  (90~, 100~, 110~)      (5 sides)
Hamburger:          LMN  (120~, 130~, 140~)     (2 sides)
Drink:              OPQ  (150~, 160~, 170~)     (4 sides)??
Lunchbox:           RST  (180~, 190~, 200~)     (2 side)
'''
#drawer_pose:
tmp_x = 0.35 #0.45
tmp_y = 0.35
# drawer_pose = {'left' :[[[tmp_x,  tmp_y, -0.15],  [0.0, 0.0, 0.0]],     #shelf level 1
#                         [[tmp_x,  tmp_y, -0.60],  [0.0, 0.0, 0.0]],     #shelf level 2
#                         [[tmp_x,  tmp_y, -0.95],    [0.0, 0.0, 0.0]]],  #shelf level 3

#                 'right':[[[tmp_x, -tmp_y, -0.15],  [0.0, 0.0, 0.0]],    #shelf level 1
#                         [[tmp_x, -tmp_y, -0.60],  [0.0, 0.0, 0.0]],     #shelf level 2
#                         [[tmp_x, -tmp_y, -0.95],    [0.0, 0.0, 0.0]]],  #shelf level 3

#                 'left_indx_open' : 0,                                 #_index_open: current shelf level 
#                 'right_indx_open' : 0,
#                 'left_indx_close' : 0,                                #_index_close: current shelf level 
#                 'right_indx_close' : 0}

drawer_pose = {'left' :[[102.42, -20.63, 0.00, 76.72, 14.86, -56.99, -8.23],     #shelf level 1
                        [81.64, -21.45, 0.00, 78.86, -9.89, -57.8, 5.31],     #shelf level 2
                        [94.13, -16.15, -41.78, 94.73, -7.65, -79.92, -8.65]],  #shelf level 3

                'right':[[102.42, -20.63, 0.00, 76.72, 14.86, -56.99, -8.23],    #shelf level 1
                        [81.64, -21.45, 0.00, 78.86, -9.89, -57.8, 5.31],     #shelf level 2
                        [94.13, -16.15, -41.78, 94.73, -7.65, -79.92, -8.65]],  #shelf level 3

                'left_indx_open' : 0,                                 #_index_open: current shelf level 
                'right_indx_open' : 0,
                'left_indx_close' : 0,                                #_index_close: current shelf level 
                'right_indx_close' : 0}


#camera_pose: pos, euler #, phi??
#cam_pose_shelf
cam_pose = {'left' :[[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],     #shelf level 1
                    [[0.38,  0.2, -0.25],  [0.0, 65, 0.0]],     #shelf level 2
                    [[0.38,  0.2, -0.65],  [0.0, 65, 0.0]]],    #shelf level 3

            'right':[[[0.38, -0.2, 0.15],   [0.0, 65, 0.0]],    #shelf level 1
                    [[0.38, -0.2, -0.25],  [0.0, 65, 0.0]],     #shelf level 2
                    [[0.38, -0.2, -0.65],  [0.0, 65, 0.0]]],    #shelf level 3
            'left_indx' : 0,                                    #_index: current shelf level 
            'right_indx' : 0}

#stock box
#cam_pose_box
box_pose = {'left' :[[[-0.28,  0.2, -0.25],  [0.0, -30, 0.0]]],
            'right': [[[-0.28, -0.2, -0.25],  [0.0, -30, 0.0]]]}

tot_shelf_level = 3 #np.size(drawer_pose['left'])/6

#dispose box
dispose_pose = {'plum_riceball':    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],  #TODO
                'salmon_riceball':  [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                'sandwich':         [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                'hamburger':        [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                'lunchbox':         [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]]
                }
                            
#reorient pose
reorient_pose = {'plum_riceball':    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],  #TODO
                 'salmon_riceball':  [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                 'sandwich':         [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                 'hamburger':        [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                 'lunchbox':         [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]]
                }                           

#shelf first obj pose
shelf_pose = {  'plum_riceball':[[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],  #TODO
                                [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]]],
                'plum_cnt' : 0,
                'salmon_riceball':[[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                                  [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]]],
                'salmon_cnt' : 0,
                'sandwich':[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],  
                'hamburger':[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],                   
                'drink':[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],                   
                'lunchbox':[[0.38,  0.2, 0.15],  [0.0, 65, 0.0]]
            }

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

# obj_pose = [[[[0.465, -0.1, -0.18], [0, 90, 0]],
#             [[0.465,  0.1, -0.18], [0, 90, 0]]],
#             [[[0.545, -0.1, -0.43], [0, 90,  0]],
#             [[0.545,  0.1, -0.43], [0, 90,  0]]],
#             [[[0.6, -0.2, -0.883], [0, 90,  0]],
#             [[0.6,  0.2, -0.883], [0, 90, 0]]]]

class State(IntEnum):
    init                = 0
    open_drawer         = 1 
    move_cam2shelf      = 2
    move_cam2box        = 3
    detect_obj          = 4 
    move2obj            = 5
    check_closer_pose   = 6
    pick                = 7
    dispose             = 8
    reorient            = 9
    stock               = 10    
    organize            = 11
    close_drawer        = 12
    finish              = 13

class MerchandiseTask():
    def __init__(self, name_arm_ctrl, en_sim_arm_ctrl):
        # arm control
        self.name_arm_ctrl = name_arm_ctrl
        self.en_sim_arm_ctrl = en_sim_arm_ctrl
        self.dual_arm = DualArmTask(self.name_arm_ctrl, self.en_sim_arm_ctrl)

        # state machine
        self.state = State.init

        # object information
        self.camera = GetObjInfo()
        self.curr_merchandise_list = []
        
        # self.place_pose_queue = queue.Queue()

        # self.object_queue = queue.Queue()   #object detected from both camera
        self.expired_queue = queue.Queue()  #no reorient
        self.old_queue = queue.Queue()      #need reorient
        self.new_queue = queue.Queue()      #no reorient

        #current level 
        self.expired_done = False   #this_level_expired_done
        self.old_done = False       #this_level_old_done
        self.new_done = False       #this_level_new_done

        self.left_tar_obj = queue.Queue()
        self.right_tar_obj = queue.Queue()
        self.target_obj_queue = {'left' : self.left_tar_obj, 'right' : self.right_tar_obj}
        self.target_obj = {'left': None, 'right': None}
    
        # self.left_retry_obj = queue.Queue()
        # self.right_retry_obj = queue.Queue()
        # self.retry_obj_queue = {'left': self.left_retry_obj, 'right': self.right_retry_obj}
        
        # self.obj_done = np.zeros((100), dtype=bool)
        # self.obj_retry = np.zeros((100), dtype=bool)    

    #     self.init()            
    
    # def init(self):
    #     for pose in place_pose:
    #         self.place_pose_queue.put(pose)
        
    def get_obj_info(self, arm_side):
        fb = self.dual_arm.get_feedback(arm_side) # position(x,y,z), orientation(x,y,z,w), euler, orientation, phi
       
        ids, base_H_mrks, names, exps, side_ids = self.camera.new_get_obj_info(arm_side, fb.orientation)
        self.curr_merchandise_list = self.camera.get_merchandise_log()        
        self.camera.print_merchandise_log(self.curr_merchandise_list)
        
        if ids is None:
            return
        for id, base_H_mrk, name, exp, side_id in zip(ids, base_H_mrks, names, exps, side_ids):
            num = int(id/10)-1
            self.curr_merchandise_list[num]['id'] = id
            self.curr_merchandise_list[num]['side_id'] = side_id
            self.curr_merchandise_list[num]['expired'] = exp
            
            self.curr_merchandise_list[num]['pos'] = base_H_mrk[0:3, 3]
            self.curr_merchandise_list[num]['vector'] = base_H_mrk[0:3, 2]  #aruco_z_axis, rotation (coordinate_axis: aruco z axis)
            self.curr_merchandise_list[num]['sucang'], roll = self.dual_arm.suc2vector(base_H_mrk[0:3, 2], [0, 1.57, 0])    #TODO #suck ang(0-90), roll (7 axi)
            self.curr_merchandise_list[num]['euler'] = [roll, 90, 0]      

            # self.camera.print_merchandise_log(self.curr_merchandise_list)

            #Check Arm approach from [TOP] or [DOWN]: aruco_z_axis's [z value]
            if self.curr_merchandise_list[num]['vector'][2] >= -0.2:  #TODO aruco_z_axis's [z value]>=, >???
                #Arm approach from top => GOOD to go!!!                
                # self.object_queue.put(self.curr_merchandise_list[num])
                if self.curr_merchandise_list[num]['expired'] == 'expired':
                    #TODO: check current shelf level
                    self.expired_queue.put(self.curr_merchandise_list[num])
                elif self.curr_merchandise_list[num]['expired'] == 'old':
                    #TODO: check current shelf level
                    self.old_queue.put(self.curr_merchandise_list[num])
                elif self.curr_merchandise_list[num]['expired'] == 'new':
                    #TODO: check current shelf level
                    self.new_queue.put(self.curr_merchandise_list[num])
                else: #ERROR
                    print('eerrrror')
                    pass
                print('Good!!! object put in queue; name: {}, id: {}'.format(self.curr_merchandise_list[num]['name'], self.curr_merchandise_list[num]['id']))                
            else:                
                print('ERROR!!! Arm approach from downside => BAD, aruco_z_axis: {}'.format(self.curr_merchandise_list[num]['vector']))

    def check_closer_pose(self, arm_side):
        self.target_obj[arm_side] = self.target_obj_queue[arm_side].get()

        fb = self.dual_arm.get_feedback(arm_side)        
        ids, base_H_mrks,  _, _, _ = self.camera.new_get_obj_info(arm_side, fb.orientation)

        if ids is None:
            return

        for id, base_H_mrk, in zip(ids, base_H_mrks):
            if id == self.target_obj[arm_side]['id']:
                self.target_obj[arm_side]['pos'] = base_H_mrk[0:3, 3] 
                if base_H_mrk[2, 2] > -0.1: #FIXME: value= ?? #TODO: i dont know z value > -0.1????
                    self.target_obj[arm_side]['sucang'], roll = self.dual_arm.suc2vector(base_H_mrk[0:3, 2], [0, 1.57, 0])
                    self.target_obj[arm_side]['euler']   = [roll, 90, 0]
        pass

    def state_control(self, arm_state, arm_side):
 
        print('\nCURRENT: arm_state= {}, arm_side = {}'.format(arm_state, arm_side))
        if arm_state is None:
            arm_state = State.init
        elif arm_state == State.init:
            arm_state = State.open_drawer #State.move_cam2shelf#
        elif arm_state == State.open_drawer:
            arm_state = State.init #move_cam2shelf
        elif arm_state == State.move_cam2shelf:
            arm_state = State.detect_obj
        elif arm_state == State.move_cam2box:
            arm_state = State.detect_obj
        elif arm_state == State.detect_obj:
            #TODO: check if it works
            if self.expired_queue.empty() & self.old_queue.empty() & self.new_queue.empty():
                if cam_pose[arm_side+'_indx'] <= tot_shelf_level:
                    if (self.expired_done==True & self.old_done==True):
                        arm_state = State.move_cam2shelf #move_cam2 to shelf #take pic
                    else:
                        arm_state = State.move_cam2box #move_cam2 stock box #take pic
                else:
                    arm_state = State.finish
            else: #have obj in queue
                arm_state = State.move2obj
        
        elif arm_state == State.move2obj:
            arm_state = State.pick#State.check_closer_pose

        elif arm_state == State.check_closer_pose:
            arm_state = State.pick
        
        elif arm_state == State.pick:
            if arm_side == 'left':
                is_grip = self.dual_arm.left_arm.suction.is_grip
            else:
                is_grip = self.dual_arm.right_arm.suction.is_grip

            if is_grip:                
                if(self.target_obj[arm_side]['expired']=='expired'):    #dispose
                    arm_state = State.dispose
                elif(self.target_obj[arm_side]['expired']=='old'):      #reorient
                    arm_state = State.reorient
                elif(self.target_obj[arm_side]['expired']=='new'):      #stock
                    arm_state = State.stock
                else:
                    arm_state = State.place

            elif (self.expired_done & self.old_done & self.new_done) == True: #if current level done
                self.expired_done = False
                self.old_done = False
                self.new_done = False               

                if cam_pose[arm_side+'_indx'] >= tot_shelf_level:
                    arm_state = State.close_drawer #State.finish
                else:
                    arm_state = State.move_cam2shelf   #move to next level shelf to take picture
            else:
                # if self.obj_retry[self.target_obj[arm_side]['id']] == False:
                #     self.retry_obj_queue[arm_side].put(self.target_obj[arm_side])
                # arm_state = State.move2obj
                arm_state = State.finish #tmp

        elif arm_state == State.dispose:
            arm_state = State.move2obj
        elif arm_state == State.reorient:
            arm_state = State.move2obj
        elif arm_state == State.stock:
            arm_state = State.move2obj

        elif arm_state == State.place:
            if self.next_level[arm_side] == True:
                self.next_level[arm_side] = False
                if cam_pose[arm_side+'_indx'] >= tot_shelf_level:
                    arm_state = State.close_drawer #State.finish
                else:
                    arm_state = State.move_cam2shelf   #move to next level shelf to take picture
            else:
                arm_state = State.move2obj

        # elif arm_state == State.organize:
        #     arm_state = State.close_drawer

        elif arm_state == State.close_drawer:
            if drawer_pose[arm_side+'_indx_open'] < tot_shelf_level:
                arm_state = State.init #State.open_drawer
            else:
                arm_state = State.finish

        elif arm_state == State.finish:
            arm_state = None

        print('SWITCHED to: arm_state= {}, arm_side = {}'.format(arm_state, arm_side))
        return arm_state

    def strategy(self, arm_state, arm_side):

        print("--->execute strategy: ", arm_state)
        cmd = Command()
        cmd_queue = queue.Queue()

        if arm_state == State.init:
            print(' ++++++++++ init ++++++++++ ', arm_side)
            cmd['cmd'] = 'jointMove'            
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0] #[0, 0, 0, 0, 0, 0, 0, 0]            
            cmd['state'] = State.init
            cmd['speed'] = 100
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, False, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.open_drawer:
            print(' ++++++++++ open_drawer ++++++++++ ', arm_side)
            # print('drawer_pose: \n\tarm_side = {}; \n\tpos, euler, phi = {}, {}, {}'.format( 
            #     arm_side+'_indx_open', 
            #     drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][0],
            #     drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][1],
            #     0))
            # #TODO: can use both hands together????????
            # draw_pose = ObjInfo()
            # draw_pose['pos'] = drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][0]
            # #safe prepare pose
            # cmd['state'] = State.open_drawer
            # cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            # cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0]-0.3, draw_pose['pos'][1], draw_pose['pos'][2]], \
            #     drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][1], 0
            # cmd_queue.put(copy.deepcopy(cmd))
            # #suck
            # cmd['suc_cmd'] = 'Off'
            # cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            # cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0], draw_pose['pos'][1], draw_pose['pos'][2]], \
            #     drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][1], 0            
            # cmd_queue.put(copy.deepcopy(cmd))
            # #pull drawer
            # cmd['suc_cmd'] = 0
            # cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            # cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0]-0.2, draw_pose['pos'][1], draw_pose['pos'][2]], \
            #     drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']][1], 0            
            # # cmd['cmd'] = 'occupied'            
            # cmd_queue.put(copy.deepcopy(cmd))
            # self.dual_arm.send_cmd(arm_side, True, cmd_queue)

            print('drawer_pose: \n\tarm_side = {}; \n\tjoint = {}'.format( 
                arm_side+'_indx_open', 
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']]
                ))            
            cmd['cmd'] = 'jointMove'            
            cmd['jpos'] = drawer_pose[arm_side][drawer_pose[arm_side+'_indx_open']]       
            cmd['state'] = State.open_drawer
            cmd['speed'] = 100
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, False, cmd_queue)

            if arm_side != 'fail':
                drawer_pose[arm_side+'_indx_open'] += 1         #TODO: 1 shelf level only take picture one time!!!!! NO!!!!
                print('{} arm move move to drawer open pose SUCCEED'.format(arm_side))
            else:
                print('{} arm move move to drawer open pose FAILED'.format(arm_side))
            print('next shelf level = {}'.format(drawer_pose[arm_side+'_indx_open']))
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.move_cam2shelf:
            print(' ++++++++++ move_cam2shelf ++++++++++ ', arm_side)            
            print('camera_pose: \n\tarm_side = {}; \n\tpos, euler, phi = {}, {}, {}'.format( 
                arm_side+'_indx', 
                cam_pose[arm_side][cam_pose[arm_side+'_indx']][0],
                cam_pose[arm_side][cam_pose[arm_side+'_indx']][1],
                0))
            
            cmd['state'] = State.move_cam2shelf
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = cam_pose[arm_side][cam_pose[arm_side+'_indx']][0], cam_pose[arm_side][cam_pose[arm_side+'_indx']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
                        
            # cmd['cmd'], cmd['state'] = 'occupied', State.move_cam2shelf
            cmd['cmd'] = 'occupied'
            cmd_queue.put(copy.deepcopy(cmd))
            arm_side = self.dual_arm.send_cmd(arm_side, True, cmd_queue)

            if arm_side != 'fail':
                cam_pose[arm_side+'_indx'] += 1         #TODO: 1 shelf level only take picture one time!!!!! NO!!!!
                print('{} arm move move to take picture pose SUCCEED'.format(arm_side))
            else:
                print('{} arm move move to take picture pose FAILED'.format(arm_side))            
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.move_cam2box:
            print(' ++++++++++ move_cam2box ++++++++++ ', arm_side)            
            print('box_pose: \n\tarm_side = {}; \n\tpos, euler, phi = {}, {}, {}'.format( 
                arm_side+'_indx', 
                box_pose[arm_side][0][0],
                box_pose[arm_side][0][1],
                0))
            
            cmd['state'] = State.move_cam2box
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = box_pose[arm_side][0][0], box_pose[arm_side][0][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
                        
            # cmd['cmd'], cmd['state'] = 'occupied', State.move_cam2box
            cmd['cmd'] = 'occupied'
            cmd_queue.put(copy.deepcopy(cmd))
            arm_side = self.dual_arm.send_cmd(arm_side, True, cmd_queue)

            if arm_side != 'fail':
                cam_pose[arm_side+'_indx'] += 1         #TODO: 1 shelf level only take picture one time!!!!! NO!!!!
                print('{} arm move move to take picture box pose SUCCEED'.format(arm_side))
            else:
                print('{} arm move move to take picture box pose FAILED'.format(arm_side))            
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.detect_obj:
            print(' ++++++++++ detect_obj ++++++++++ ', arm_side)
            self.get_obj_info(arm_side)
            print('total expired_queue', self.expired_queue.qsize())
            print('total old_queue', self.old_queue.qsize())
            print('total new_queue', self.new_queue.qsize())

            cmd['cmd'] = None
            cmd['state'] = State.detect_obj
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.move2obj:
            print(' ++++++++++ move2obj ++++++++++ ', arm_side)            

            # expired(disposal) -> old(reorient) -> new(stock)
            if self.expired_queue.qsize() > 0: 
                obj = self.expired_queue.get()
                #dispose State.dispose
            elif self.old_queue.qsize() > 0:
                obj = self.old_queue.get()
                #reorient State.dispose
            elif self.new_queue.qsize() > 0:
                obj = self.new_queue.get()
                #stock State.stock
            else:
                obj = ObjInfo()
       
            #TODO:check object_queue if fail grasp, do what??? target_obj_queue, obj_done                    
            print(obj['name'], obj['id'], obj['side_id'], obj['expired'])

            #TODO: check if left/right out of reach
            obj_putback = False
            if (arm_side == 'left' and obj['pos'][1] < -0.02) or (arm_side == 'right' and obj['pos'][1] > 0.02): #y<-0.02 (2cm) too
                obj_putback = True
            else:
                cmd['state'] = State.move2obj                 
                cmd['suc_cmd'] = 'Off'            
                cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'            
                pos = copy.deepcopy(obj['pos'])     #(x, y, z): base_H_mrks[0:3, 3]            
                shift = 0.05                        #FIXME: how far should the camera be???
                cmd['pos'], cmd['euler'], cmd['phi'] = [pos[0]+shift, pos[1]+shift, pos[2]+shift] , [0, 90, 0], 0
                cmd_queue.put(copy.deepcopy(cmd))

                # cmd['cmd'], cmd['state'] = 'occupied', State.move2obj
                cmd['cmd'] = 'occupied'
                cmd_queue.put(copy.deepcopy(cmd))
                arm_side = self.dual_arm.send_cmd(arm_side, True, cmd_queue)

            if (obj_putback == True) or (arm_side == 'fail'): #TODO: failed because of what???? cannot reach???
                if obj['expired'] == 'expired':
                    self.expired_queue.put(obj)  #bcus failed so put the obj back to queue
                elif obj['expired'] == 'old':
                    self.old_queue.put(obj)
                elif obj['expired'] == 'new':
                    self.new_queue.put(obj)
                # self.obj_done[obj['id']] = False
                print('move2obj FAILED!!!!!!! arm_side = {}, name = {}, id = {}'.format(arm_side, obj['name'], obj['id']))
            else:
                self.target_obj_queue[arm_side].put(obj)
                print('move2obj OK! arm_side = {}, name = {}, id = {}'.format(arm_side, obj['name'], obj['id']))
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.check_closer_pose:
            print(' ++++++++++ check_closer_pose ++++++++++ ', arm_side)
            self.check_closer_pose(arm_side)

            cmd['cmd'], cmd['state'] = 'occupied', State.check_closer_pose
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.pick:
            print(' ++++++++++ pick ++++++++++ ', arm_side)
            #TODO: check ori code, i don't know how to rewrite

            #======SKIP check_closer_pose FOR VIRTUAL TESTING=======#
            #======COMMENT OUT if check_closer_pose IS USED=========#
            self.target_obj[arm_side] = self.target_obj_queue[arm_side].get()
            #======SKIP check_closer_pose FOR VIRTUAL TESTING=======#

            obj = copy.deepcopy(self.target_obj[arm_side])
            pos = copy.deepcopy(obj['pos'])
            mrk_z_axis = copy.deepcopy(obj['vector']) #already normalized            
         
            #away_scale: approach?/away? mrk along z axis 
            away_scale = 0.150 #(0.050 m = 50mm = 5cm)
            pos_shifted_approach = pos + away_scale*mrk_z_axis
            pos_shifted_leave = pos - away_scale*mrk_z_axis# + [0.0, 0.0, 0.05]
            print('pos', pos)
            print('pos_shifted_approach', pos_shifted_approach)
            print('pos_shifted_leave', pos_shifted_leave)

            #approach obj + suc off
            cmd['state'] = State.pick
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line' #p2p, line
            cmd['pos'], cmd['euler'], cmd['phi'] = pos_shifted_approach , obj['euler'], 0            
            cmd_queue.put(copy.deepcopy(cmd))
            
            #attach + suc on            
            cmd['suc_cmd'] = 'On'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line' #p2p, line
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, obj['euler'], 0       
            cmd_queue.put(copy.deepcopy(cmd))
            
            #attach + suc on + move up back off            
            cmd['suc_cmd'] = 'On'
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line' #p2p, line
            cmd['pos'], cmd['euler'], cmd['phi'] = pos_shifted_leave, obj['euler'], 0
            cmd_queue.put(copy.deepcopy(cmd))          
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)
        
        elif arm_state == State.dispose:
            print(' ++++++++++ dispose ++++++++++ ', arm_side)
            #TODO: check if left/right out of reach
            
            #disposal_box_intermediate_pose (left/right arm)
            cmd['cmd'] = 'jointMove'            
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0]
            cmd['speed'] = 100
            cmd_queue.put(copy.deepcopy(cmd))                      
            
            #put disposed product into disposal box
            #use obj name to get dispose position
            obj_name_dispose = self.target_obj[arm_side]['name']
            cmd['cmd'], cmd['mode'] = 'ikMove', 'line' #p2p, line
            cmd['pos'], cmd['euler'], cmd['phi'] = dispose_pose[obj_name_dispose]               
            cmd['state'] = State.dispose
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)
            print(obj_name_dispose, 'disposed successfully!!!!!!!!!!!!!!!!!!!!!!!!!!')

        elif arm_state == State.reorient:
            print(' ++++++++++ reorient ++++++++++ ', arm_side)
            #TODO: check if left/right out of reach

            obj_name_reorient = self.target_obj[arm_side]['name']
            reorient_method = self.target_obj[arm_side]['side_id']
            
            #write the related transformation of the product to the correct reorientation

            cmd['state'] = State.reorient
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)
            print(obj_name_reorient, 'reoriented successfully!!!!!!!!!!!!!!!!!!!!!!!!!!')

        elif arm_state == State.stock:
            print(' ++++++++++ stock ++++++++++ ', arm_side)    
            #TODO: check if left/right out of reach        
            obj_name_stock = self.target_obj[arm_side]['name']
            
            #current shelf_intermediate_pose (left/right arm)
            cmd['cmd'] = 'jointMove'            
            cmd['jpos'] = [0, 0, -1.2, 0, 1.87, 0, -0.87, 0]
            cmd['speed'] = 100
            cmd_queue.put(copy.deepcopy(cmd))      

            #use obj name to get dispose position
            #TODO: stock the product behind reoriented product!!!!
            new_pose = []
            if(obj_name_stock == 'plum_riceball'):
                #stock 2 times
                new_pose = shelf_pose[obj_name_stock][shelf_pose['plum_cnt']]
                shelf_pose['plum_cnt'] += 1
            elif(obj_name_stock == 'salmon_riceball'):
                #stock 2 times
                new_pose = shelf_pose[obj_name_stock][shelf_pose['salmon_cnt']]
                shelf_pose['salmon_cnt'] += 1
            elif(obj_name_stock == 'lunchbox'):
                #put UNDER the reoriented obj
                new_pose = shelf_pose[obj_name_stock]
            else:
                #just put behind the reoriented obj
                new_pose = shelf_pose[obj_name_stock]

            cmd['cmd'], cmd['mode'] = 'ikMove', 'line' #p2p, line
            cmd['pos'], cmd['euler'], cmd['phi'] = new_pose
            cmd['state'] = State.stock
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)
            print(obj_name_stock, 'stocked successfully!!!!!!!!!!!!!!!!!!!!!!!!!!')

        elif arm_state == State.place:
            print(' ++++++++++ place ++++++++++ ', arm_side)
            cmd['state'] = State.place
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.5, 0, 2.07, 0, -0.57, 0]
            cmd_queue.put(copy.deepcopy(cmd))

            pose = self.place_pose_queue.get()
            pos, euler = pose[0], pose[1]
            if arm_side == 'left':
                pos[1] += 0.12
            else:
                pos[1] -= 0.12
            cmd['cmd'], cmd['mode'] = 'fromtNoaTarget', 'line'
            cmd['pos'], cmd['euler'], cmd['phi'] = pos, euler, 0
            cmd['suc_cmd'], cmd['noa'] = 0, [0, 0, -0.2]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, 0.2]
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'], cmd['mode'], cmd['noa'] = 'noaMove', 'line', [0, 0, -0.2]
            cmd['suc_cmd'] = 'Off'
            cmd_queue.put(copy.deepcopy(cmd))

            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1.8, 0, 2.57, 0, -0.87, 0]
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        elif arm_state == State.organize:
            print(' ++++++++++ organize ++++++++++ ', arm_side)
            #TODO        
            cmd['state'] = State.organize            
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.close_drawer:
            print(' ++++++++++ close_drawer ++++++++++ ', arm_side)
            print('drawer_pose: \n\tarm_side = {}; \n\tpos, euler, phi = {}, {}, {}'.format( 
                arm_side+'_indx_close', 
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][0],
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][1],
                0))
            #TODO: can use both hands together????????
            draw_pose = ObjInfo()
            draw_pose['pos'] = drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][0]
            
            # safe prepare pose
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0]-0.3, draw_pose['pos'][1], draw_pose['pos'][2]], \
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][1], 0
            cmd_queue.put(copy.deepcopy(cmd))
            
            # suck
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0]-0.2, draw_pose['pos'][1], draw_pose['pos'][2]], \
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][1], 0
            cmd['suc_cmd'] = 0
            cmd_queue.put(copy.deepcopy(cmd))

            # push drawer back
            cmd['cmd'], cmd['mode'] = 'ikMove', 'p2p'
            cmd['pos'], cmd['euler'], cmd['phi'] = [draw_pose['pos'][0], draw_pose['pos'][1], draw_pose['pos'][2]], \
                drawer_pose[arm_side][drawer_pose[arm_side+'_indx_close']][1], 0
            cmd['suc_cmd'] = 'Off'  #TODO: release sucker when pose reached? or release sucker while moving??
            # cmd['cmd'] = 'occupied'
            cmd['state'] = State.close_drawer
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, True, cmd_queue)

            if arm_side != 'fail':
                drawer_pose[arm_side+'_indx_close'] += 1         #TODO: 1 shelf level only take picture one time!!!!! NO!!!!
                print('{} arm move move to drawer close pose SUCCEED'.format(arm_side))                
            else:
                print('{} arm move move to drawer close pose FAILED'.format(arm_side))                

            print('next shelf level = {}'.format(drawer_pose[arm_side+'_indx_close']))
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:',self.dual_arm.right_arm.status)

        elif arm_state == State.finish:
            print(' ++++++++++ finish ++++++++++ ', arm_side)
            cmd['suc_cmd'] = 'Off'
            cmd['cmd'] = 'jointMove'
            cmd['jpos'] = [0, 0, -1, 0, 1.57, 0, -0.57, 0]
            cmd['state'] = State.finish
            cmd_queue.put(copy.deepcopy(cmd))
            self.dual_arm.send_cmd(arm_side, False, cmd_queue)
            print('left_arm.status:', self.dual_arm.left_arm.status)
            print('right_arm.status:', self.dual_arm.right_arm.status)

        return arm_side

    def process(self):
            #assign task to LEFT of RIGHT arm
            # (1) check left/right arm status
            # (2) state_control: switch arm state
            # (3) strategy: perform arm actions

            rate = rospy.Rate(10)
            rospy.on_shutdown(self.dual_arm.shutdown)
            while True:
                l_status = self.dual_arm.left_arm.status
                if l_status == Status.idle or l_status == Status.occupied:
                    l_state = self.state_control(self.dual_arm.left_arm.state, 'left')
                    self.strategy(l_state, 'left')
                rate.sleep()
                
                r_status = self.dual_arm.right_arm.status
                if r_status == Status.idle or r_status == Status.occupied:
                    r_state = self.state_control(self.dual_arm.right_arm.state, 'right')
                    self.strategy(r_state, 'right')
                rate.sleep()

                if l_state is None and r_state is None:
                    if l_status == Status.idle and r_status == Status.idle:
                        return

if __name__ == '__main__':
    rospy.init_node('merchandize')

    strategy = MerchandiseTask('dual_arm', True)
    rospy.on_shutdown(strategy.dual_arm.shutdown)
    strategy.process()
    strategy.dual_arm.shutdown()
    del strategy.dual_arm
