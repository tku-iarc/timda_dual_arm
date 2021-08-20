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

#right_bord_pos = 0.75, -0.2, -0.43
right_c_pose = [[[0.5004538080655432, -0.2532141209025338,  -0.06718596221420141, ],[ 0.0323699884883336,  65.01652468420149, -0.03279616184726517, ], -7.011867844079209e-05],
                [[0.5008529474659436, -0.3355746512003659,  -0.0795152955502393,  ],[ 0.03174282075550094, 64.97838972621923, -0.008572001027169684,], -7.070874784871437e-05],
                [[0.5012091875650093, -0.3880340515253939,  -0.07990131419809376, ],[ 0.03823722452664159, 64.95018528966597,  0.02866793345451034, ], -7.163562836185784e-05],
                [[0.5016784266586852, -0.4281358694143783,   0.08159055375953016, ],[ 0.04726460637440334, 69.6466404688525,   0.04312863619678441, ], -3.306651113363206e-06],
                [[0.5022728829717511, -0.261248327548225,   -0.002510953338052603,],[ 0.05331214002166357, 74.65792458021502,  0.06337251894565969, ], -0],
                [[0.5031123648350082, -0.04043193288831777, -0.06547315022754449, ],[-10.20392555186906,   74.58744891759585, -0.000119274099985916,], -0],
                [[0.525530662483441,  -0.08308183343130505, -0.05351415654100983, ],[-22.40352584903326,   63.87173907086792, -0.02369158474965406, ], -0],
                [[0.5266675579478259, -0.1319401998174144,  -0.02380926693679808, ],[-43.61933743112927,   63.80247094910232, -0.1297594779958388,  ], -0],
                [[0.5269723930424395, -0.06574322711920698, -0.05364590073202039, ],[-68.04140824688895,   63.78608835251243, -0.1597996226962361,  ], -1.207418269725733e-06],
                [[0.5270769760067999, -0.06577762263693067,  0.03399384384781029, ],[-68.04841396320886,   63.78216302331231, -0.1753758208530155,  ], -3.078324659199975e-06],
                [[0.5272092900902987, -0.06582212291928258,  0.07255324533775831, ],[-82.4680992940786,    67.51613737133755, -0.1960948975294225,  ], -0],
                [[0.5280539813753632, -0.1540678604564929,  -0.07786178981456808, ],[ 14.3691586823422,    77.68382391186351, -0.2298830595211877,  ], -0],
                [[0.4823415350671684, -0.2399701948205364,  -0.08925383410436771, ],[ 28.04875340603212,   77.65837139858763, -0.2352478929567015,  ], -0],
                [[0.4791931993168074, -0.3149709612270601,  -0.1314791281837705,  ],[ 44.22395960243066,   77.62795426419487, -0.2457129339789655,  ], -0],
                [[0.4138575366293342, -0.297527484295576,   -0.2126739825191691,  ],[ 60.58006338433427,   66.3474808736441,  -11.95869213814548,   ], -0],
                [[0.4139939455668695, -0.2975475105725396,  -0.2852077107466298,  ],[ 75.32258426252123,   66.34753499853126, -11.96455947827606,   ], -0],
                [[0.4121825233908999, -0.319210126629863,   -0.2530958086474356,  ],[ 74.79920625273058,   66.64329176528997, -6.085190075340546,   ], -15.71553991053553],
                [[0.4123827640796597, -0.3193150560478027,  -0.1468756819758649,  ],[ 74.790731418693,     66.64335354449192, -6.107679999675296,   ], -15.72096225956446],
                [[0.4147377995611876, -0.4293097186830637,  -0.1907875458258219,  ],[ 75.18244454404424,   96.58401763743851,  11.45644267135307,   ],  16.56934230738585],
                [[0.4150644218349576, -0.4295692602099899,  -0.2836906345115083,  ],[ 75.21163971739115,   108.7901650180467,  11.38845247440495,   ],  16.64490865236882],
                [[0.4151721273681787, -0.4298914174447654,  -0.4091692158745216,  ],[ 75.22962615126963,   120.8127021400017,  11.32261873301032,   ],  16.72212785351192],
                [[0.4238030391923046, -0.3112565144187608,  -0.4169273091393302,  ],[ 46.36080028874433,   120.8568503533463, -2.690389443070493,   ],  17.0316218545143],
                [[0.4242931851394318, -0.1617764832546068,  -0.3877009376970814,  ],[ 46.37267289744338,   117.4385590058583, -18.64582960792653,   ],  17.08024512224669],
                [[0.4197479786921792, -0.1711877155854302,  -0.3969178320297976,  ],[-5.035224992939342,   117.4393858139155, -18.65125261731989,   ],  17.08080629014662]]


c_pose = {'left' :[[[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.19, 0.15],    [0.0, 65, 0.0]],
                    [[0.38,  0.21, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.2, 0.15],  [0.0, 65, 0.0]],
                    [[0.38,  0.19, 0.15],    [0.0, 65, 0.0]]],
          'right': right_c_pose,
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
            cmd['pos'], cmd['euler'], cmd['phi'] = c_pose[side][c_pose[side+'_indx']][0], c_pose[side][c_pose[side+'_indx']][1], c_pose[side][c_pose[side+'_indx']][2]
            cmd['state'] = state
            cmd_queue.put(copy.deepcopy(cmd))
            side = self.dual_arm.send_cmd(side, False, cmd_queue)
            if side != 'fail':
                c_pose[side+'_indx'] = c_pose[side+'_indx'] + 1
                if c_pose[side+'_indx'] == len(c_pose[side]):
                    self.is_done[side] = True
                    c_pose[side+'_indx'] = 0
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
            req.is_done = self.is_done[side]
            req.end_trans.translation.x = arm_feedback.group_pose.position.x
            req.end_trans.translation.y = arm_feedback.group_pose.position.y
            req.end_trans.translation.z = arm_feedback.group_pose.position.z
            req.end_trans.rotation.x    = arm_feedback.group_pose.orientation.x
            req.end_trans.rotation.y    = arm_feedback.group_pose.orientation.y
            req.end_trans.rotation.z    = arm_feedback.group_pose.orientation.z
            req.end_trans.rotation.w    = arm_feedback.group_pose.orientation.w
            res = self.hand_eye_client(req)
            if self.is_done[side]:
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
                config.read(curr_path + '/config/' + side + '_arm_img_trans.ini')
                
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

                config.write(curr_path + '/config/' + side + '_arm_img_trans.ini')

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
