#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""Use to generate arm task and run."""

import rospy
import tf
import Queue as queue
import copy
# import object_distribtion

from math import radians, degrees, sin, cos, pi, acos, asin
from numpy import multiply
from enum import IntEnum
# from Queue import Queue

import numpy as np

from std_msgs.msg import String, Float64, Bool
from robotis_controller_msgs.msg import StatusMsg
# from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
from manipulator_h_base_module_msgs.msg import P2PPose, JointPose, KinematicsPose
from manipulator_h_base_module_msgs.srv import GetKinematicsPose, GetKinematicsPoseResponse
from manipulator_h_base_module_msgs.srv import GetJointPose, GetJointPoseResponse
from manipulator_h_base_module_msgs.srv import CheckRangeLimit, CheckRangeLimitRequest
from vacuum_cmd_msg.srv import VacuumCmd
from suction import SuctionTask
from robotiq_2f_gripper import RobotiqGripper

_POS = (0, 0, 0)
_ORI = (0, 0, 0)
_PHI = 45

class Status(IntEnum):
    idle            = 0
    busy            = 1
    emergency_stop  = 2
    ik_fail         = 3
    grasping        = 4
    occupied        = 5
    gripping        = 6


class Command(dict):

    def __init__(self):
        self['cmd']    = "ikMove" #'ikMove', 'jointMove', 'noaMove', 'fromtNoaTarget', 'relativeMove', 'relativePos', 'relativeEuler', 'grasping', 'occupied'
        self['mode']   = "p2p"    #'p2p', 'line'
        self['pos']    = None
        self['euler']  = None
        self['phi']    = None
        self['noa']      = None
        self['suc_cmd']= None
        self['gripper_cmd']= None
        self['jpos']   = None
        self['speed']  = None
        self['state']  = None
        self['next_state'] = None

class ArmTask:
    """Running arm task class."""

    def __init__(self, _name , en_sim):
        """Inital object."""
        self.name = _name + '_arm'
        self.suc_name = _name
        self.grip_name = _name
        self.en_sim = en_sim
        self.suction_angle = 0
        self.init()

    def init(self):
        self.__set_pubSub()
        #rospy.on_shutdown(self.stop_task)
        self.__set_mode_pub.publish('set')
        self.__is_busy = False
        self.__ik_fail = False
        self.__is_stop = False
        self.__is_wait = False
        self.__speed = 500
        self.status = Status.idle
        self.__cmd_queue = queue.Queue()
        self.__cmd_queue_2nd = queue.Queue()
        self.occupied = False
        self.state = None
        self.next_state = None
        if self.en_sim==True:
            self.suction = SuctionTask(self.suc_name + '_gazebo')
            self.gripper = RobotiqGripper(self.grip_name)
        else:
            self.suction = SuctionTask(self.suc_name)
            self.gripper = RobotiqGripper(self.grip_name)
        print("03")

    def __set_pubSub(self):
        print ("[Arm] name space : " + str(self.name)) 
        self.__set_mode_pub = rospy.Publisher(
            str(self.name) + '/set_mode_msg',
            String,
            # latch=True,
            queue_size=1
        )
        self.__wait_pub = rospy.Publisher(
            str(self.name) + '/wait',
            Bool,
            # latch=True,
            queue_size=1
        )
        self.__clear_pub = rospy.Publisher(
            str(self.name) + '/clear_cmd',
            Bool,
            # latch=True,
            queue_size=1
        )
        self.__joint_pub = rospy.Publisher(
            str(self.name) + '/joint_pose_msg',
            JointPose,
            # latch=True,
            queue_size=1
        )
        self.__p2p_pub = rospy.Publisher(
            str(self.name) + '/p2p_pose_msg',
            P2PPose,
            # latch=True,
            queue_size=1
        )
        self.__line_pub = rospy.Publisher(
            str(self.name) + '/kinematics_pose_msg',
            KinematicsPose,
            # latch=True,
            queue_size=1
        )
        self.__status_sub = rospy.Subscriber(
            str(self.name) + '/status',
            StatusMsg,
            self.__status_callback,
            queue_size=1
        )
        self.__stop_sub = rospy.Subscriber(
            'robot/is_stop',
            Bool,
            self.__stop_callback,
            queue_size=5
        )
        # Waiting for topic enable
        rospy.sleep(0.3)

    def __status_callback(self, msg):
        if 'IK Failed' in msg.status_msg:
            rospy.logwarn('ik fail')
            self.__ik_fail = True
            self.status = Status.ik_fail

        elif 'End Trajectory' in msg.status_msg:
            self.__is_busy = False
            rospy.sleep(0.3)
            # if self.occupied is True and self.__cmd_queue.empty():
            #     self.status = Status.occupied
            # elif self.__cmd_queue.empty() and self.__cmd_queue_2nd.empty():
            #     self.status = Status.idle
            print('Arm task receive End Trajectory')

    def __stop_callback(self, msg):
        if msg.data:
            self.__is_stop = True
            self.status = Status.emergency_stop
        
    def back_home(self):
        self.jointMove(0,(0, 0, 0, 0, 0, 0, 0))

    @property
    def cmd_queue(self):
        return self.__cmd_queue
    
    @cmd_queue.setter
    def cmd_queue(self, cmd_q):
        self.__cmd_queue = cmd_q

    @property
    def cmd_queue_empty(self):
        return self.__cmd_queue.empty()

    @property
    def cmd_queue_2nd(self):
        return self.__cmd_queue_2nd
    
    @cmd_queue_2nd.setter
    def cmd_queue_2nd(self, cmd_q):
        self.__cmd_queue_2nd = cmd_q

    @property
    def cmd_queue_2nd_empty(self):
        return self.__cmd_queue_2nd.empty()

    @property
    def status(self):
        return self.status

    @property
    def is_busy(self):
        return self.__is_busy

    @property
    def wait(self):
        return self.__is_wait

    @wait.setter
    def wait(self, state):
        if type(state) is bool:
            self.__is_wait = state
        else:
            err_msg = 'Type Error'
            print(err_msg)
            raise Exception(err_msg)

    @property
    def is_ikfail(self):
        return self.__ik_fail

    @property
    def is_stop(self):
        return self.__is_stop

    def set_speed(self,i_speed):
        self.__speed = i_speed

    def cmd_queue_put(self, cmd_q):
        while not cmd_q.empty():
            self.__cmd_queue.put(cmd_q.get())

    def cmd_queue_2nd_put(self, cmd_q):
        while not cmd_q.empty():
            self.__cmd_queue_2nd.put(cmd_q.get())

    def cmd_2to1(self):
        while not self.__cmd_queue_2nd.empty():
            cmd = self.__cmd_queue_2nd.get()
            self.__cmd_queue.put(cmd)
            if cmd['cmd'] == 'occupied':
                return

    def jointMove(self, slide_pos = 0,cmd=[0, 0, 0, 0, 0, 0, 0]):
        """Publish msg of joint cmd (rad) to manager node."""
        name  = list()
        value = list()
        speed = self.__speed
        
        for i, val in enumerate(cmd):
            name.append('joint{}'.format(i+1))
            value.append(val)

        self.__joint_pub.publish(JointPose(name, value, slide_pos, speed))
        self.__is_busy = True
        self.status = Status.busy

    def singleJointMove(self, index=-1, pos=0):
        fb = self.get_joint()
        slide_pos = fb.slide_pos
        joint_pos = list(fb.joint_value)
        if index == 0:
            slide_pos = slide_pos + pos
            self.jointMove(slide_pos ,joint_pos)
        elif 0 < index <= 7:
            joint_pos[index-1] = joint_pos[index-1] + pos
            self.jointMove(slide_pos ,joint_pos)

    def euler2rotation(self, euler):
        roll, pitch, yaw = euler

        origin    = np.matrix([[1, 0, 0],
                               [0, -1, 0],
                               [0, 0, -1]])

        rotationX = np.matrix([[1.0,      0.0,       0.0],
                               [0.0, cos(yaw), -sin(yaw)],
                               [0.0, sin(yaw),  cos(yaw)]])

        rotationY = np.matrix([[cos(pitch),  0.0, sin(pitch)],
                               [0.0,         1.0,        0.0],
                               [-sin(pitch), 0.0, cos(pitch)]])

        rotationZ = np.matrix([[cos(roll), -sin(roll), 0.0],
                               [sin(roll),  cos(roll), 0.0],
                               [0.0,           0.0,    1.0]])
        return origin * rotationY * rotationX * rotationZ

    def euler2quaternion(self, euler):
        roll, pitch, yaw = euler
        quaternion = tf.transformations.quaternion_from_euler(-pitch+pi, -yaw, roll-pi, 'ryxz')
        return (quaternion)

    def ikMove(self, mode='line', pos=_POS, euler=_ORI, phi=_PHI):
        """Publish msg of ik cmd (deg) to manager node."""
        roll, pitch, yaw = euler
        roll  = roll * pi/ 180
        pitch = pitch* pi/ 180
        yaw   = yaw  * pi/ 180

        self.__is_busy = True
        self.status = Status.busy

        msg = KinematicsPose()
        msg.name = 'arm'
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        quaternion = self.euler2quaternion((roll, pitch, yaw))
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        msg.speed = self.__speed
        msg.phi = radians(phi)
       
        #rospy.loginfo('Sent:{}'.format(cmd))

        if mode == 'line':
            self.__line_pub.publish(msg)
        elif mode == 'p2p':
            self.__p2p_pub.publish(msg)


    def quaternion2euler(self, ori):
        quaternion = (
            ori.x,
            ori.y,
            ori.z,
            ori.w
        )
        pitch, yaw, roll = tf.transformations.euler_from_quaternion(quaternion, 'ryxz')
        # euler = roll+pi ,-pitch+pi, -yaw
        euler = -roll, -pitch, yaw
        return euler

    def rotation2vector(self, rot):
        vec_n = [rot[0, 0], rot[1, 0], rot[2, 0]]
        vec_s = [rot[0, 1], rot[1, 1], rot[2, 1]]
        vec_a = [rot[0, 2], rot[1, 2], rot[2, 2]]
        return vec_n, vec_s, vec_a 

    def get_fb(self):
        rospy.wait_for_service(self.name + '/get_kinematics_pose')
        try:
            get_endpos = rospy.ServiceProxy(
                self.name + '/get_kinematics_pose',
                GetKinematicsPose
            )
            res = get_endpos('arm')
            return res
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)
    
    def get_joint(self):
        rospy.wait_for_service(self.name + '/get_joint_pose')
        try:
            joint = rospy.ServiceProxy(
                self.name + '/get_joint_pose',
                GetJointPose
            )
            name  = list()
            for i in range(1, 8):
                name.append('joint{}'.format(i))
                
            res = joint(name)
            return res
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def check_range_limit(self, pos=_POS, euler=_ORI, phi=_PHI):
        roll, pitch, yaw = copy.deepcopy(euler)
        roll  = roll * pi/ 180
        pitch = pitch* pi/ 180
        yaw   = yaw  * pi/ 180

        req = CheckRangeLimitRequest()
        req.pose.position.x = pos[0]
        req.pose.position.y = pos[1]
        req.pose.position.z = pos[2]

        quaternion = self.euler2quaternion((roll, pitch, yaw))
        req.pose.orientation.x = quaternion[0]
        req.pose.orientation.y = quaternion[1]
        req.pose.orientation.z = quaternion[2]
        req.pose.orientation.w = quaternion[3]
        req.phi = radians(phi)

        rospy.wait_for_service(self.name + '/check_range_limit')
        try:
            range_limit = rospy.ServiceProxy(
                self.name + '/check_range_limit',
                CheckRangeLimit
            )
            res = range_limit(req)
            return res.limit_value, res.is_limit
        except rospy.ServiceException as e:
            print ("Service call failed: %s" % e)

    def noa_move_suction(self, mode='p2p', suction_angle=None, n=0, o=0, a=0):
        if suction_angle is None:
            suction_angle = self.suction_angle

        suction_angle = suction_angle * pi/180
        suction_rot = np.matrix([[cos(suction_angle),  0.0, sin(suction_angle)],
                               [0.0,                 1.0,                0.0],
                               [-sin(suction_angle), 0.0, cos(suction_angle)]])
        fb = self.get_fb()
        pos = fb.group_pose.position
        phi = fb.phi
        euler = fb.euler
        # euler = self.quaternion2euler(ori)
    
        rot = self.euler2rotation(euler) * suction_rot
        vec_n, vec_o, vec_a = self.rotation2vector(rot) #for suction
        move = [0, 0, 0]
        # a -= 0.065

        if n > 1e-10:
            move += multiply(vec_n, n)
        if o != 0:
            move += multiply(vec_o, o)
        if a != 0:
            move += multiply(vec_a, a)
        self.ikMove(
            mode,
            (pos.x + move[0], pos.y + move[1], pos.z + move[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            degrees(phi)
        )

    def noa_relative_pos(self, mode='p2p', pos=_POS, euler=_ORI, phi=_PHI, suction_angle=None, n=0, o=0, a=0):
        #由a點移動至基於b點延noa向量移動後的c點
        if suction_angle is None:
            suction_angle = self.suction_angle
        suction_angle = suction_angle * pi/180
        suction_rot = np.matrix([[cos(suction_angle),  0.0, sin(suction_angle)],
                               [0.0,                 1.0,                0.0],
                               [-sin(suction_angle), 0.0, cos(suction_angle)]])
        euler[0], euler[1], euler[2] = radians(euler[0]), radians(euler[1]), radians(euler[2])
        rot = self.euler2rotation(euler) * suction_rot
        vec_n, vec_o, vec_a = self.rotation2vector(rot) #for suction
        move = [0, 0, 0]
        a -= 0.065

        if n > 1e-10:
            move += multiply(vec_n, n)
        if o != 0:
            move += multiply(vec_o, o)
        if a != 0:
            move += multiply(vec_a, a)

        self.ikMove(
            mode,
            (pos[0] + move[0], pos[1] + move[1], pos[2] + move[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            phi
        )

    def relative_move_pose(self, mode='p2p', pos = _POS):
        fb = self.get_fb()
        curr_pos = fb.group_pose.position
        phi = fb.phi
        euler = fb.euler

        pos[0] += curr_pos.x
        pos[1] += curr_pos.y
        pos[2] += curr_pos.z

        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (degrees(euler[0]), degrees(euler[1]), degrees(euler[2])),
            degrees(phi)
        )

    def relative_move(self, mode='p2p', pos = _POS, euler=_ORI, phi=_PHI):
        fb = self.get_fb()
        curr_pos = fb.group_pose.position
        pos[0] += curr_pos.x
        pos[1] += curr_pos.y
        pos[2] += curr_pos.z

        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (euler[0], euler[1], euler[2]),
            phi
        )

    def move_euler(self, mode='p2p', euler=_ORI):
        fb = self.get_fb()
        pos = fb.group_pose.position
        phi = fb.phi
        self.ikMove(
            mode,
            (pos.x, pos.y, pos.z),
            euler,
            degrees(phi)
        )

    def move_to_vector_point(self, mode='p2p', pos=_POS, vector=[1,0,0], phi=0): # This funthion will move arm and return suction angle 
    # Only for left arm Euler (0 0 30)
        goal_vec = [-1*i for i in vector]
        a = 0.866
        b = 0.5
        x, y, z = goal_vec[0], goal_vec[1], goal_vec[2]
        roll_angle = 0.0
        suc_angle = -acos((b*y - a*z) / (a*a + b*b))
        roll_angle_c = acos(x / sin(suc_angle))
        roll_angle_s = asin(-((a*y + b*z)/(a*a + b*b)) / sin(suc_angle))
        if (roll_angle_c*roll_angle_s) >= 0:
            roll_angle = roll_angle_c
        else:
            roll_angle = -roll_angle_c
        euler = [0., 0., 0.]
        pos[0] += vector[0]*0.065
        pos[1] += vector[1]*0.065
        pos[2] += vector[2]*0.065
        euler[0] = roll_angle
        euler[1] = 0
        euler[2] = 30
        self.ikMove(
            mode,
            (pos[0], pos[1], pos[2]),
            (euler[0], euler[1], euler[2]),
            phi
        )
        return suc_angle

    def wait_busy(self):
        """This is blocking method."""
        while self.is_busy:
            rospy.sleep(0.1)

    def freeze(self,  enable):
        self.__wait_pub.publish(enable)

    def clear_cmd(self):
        self.__clear_pub.publish(True)

    def process(self):
        if self.status == Status.emergency_stop or self.status == Status.ik_fail:
            return
        # if self.status == Status.grasping:
        #     print('graspinininingggggggggg')
        #     if self.suction.is_grip:
        #         self.clear_cmd()
        #         print('is_gripppppppppppp_clear_cmd')
        #         rospy.sleep(0.1)
        #     return

        if not self.__cmd_queue.empty() and not self.is_busy:
            
            cmd = self.__cmd_queue.get()
            self.status = Status.busy        
            if cmd['state'] is not None:
                self.state = cmd['state']
            if cmd['next_state'] is not None:
                self.next_state = cmd['next_state']
            if cmd['speed'] is not None:
                self.set_speed(cmd['speed'])

            if cmd['cmd'] == 'occupied':
                self.occupied = True
            else:
                self.occupied = False
            # if self.status == Status.occupied:
                #    self.status = Status.busy
            if cmd['suc_cmd'] is not None and type(cmd['suc_cmd']) is not str:
                    self.suction_angle = cmd['suc_cmd']

            if cmd['cmd'] == 'ikMove':
                self.ikMove(cmd['mode'], cmd['pos'], cmd['euler'], cmd['phi'])

            elif cmd['cmd'] == 'jointMove':
                self.jointMove(cmd['jpos'][0], cmd['jpos'][1:8])

            elif cmd['cmd'] == 'noaMove':
                self.noa_move_suction(cmd['mode'], n=cmd['noa'][0], o=cmd['noa'][1], a=cmd['noa'][2])

            elif cmd['cmd'] == 'fromtNoaTarget':
                self.noa_relative_pos(cmd['mode'], cmd['pos'], cmd['euler'], cmd['phi'], n=cmd['noa'][0], o=cmd['noa'][1], a=cmd['noa'][2])

            elif cmd['cmd'] == 'relativeMove':
                self.relative_move(cmd['mode'], cmd['pos'], cmd['euler'], cmd['phi'])

            elif cmd['cmd'] == 'relativePos':
                self.relative_move_pose(cmd['mode'], cmd['pos'])

            elif cmd['cmd'] == 'relativeEuler':
                self.move_euler(cmd['mode'], cmd['euler'])

            elif cmd['cmd'] == 'grasping':
                self.noa_move_suction(cmd['mode'], n=cmd['noa'][0], o=cmd['noa'][1], a=cmd['noa'][2])
                self.status = Status.grasping

            elif cmd['cmd'] == 'gripping':
                self.status = Status.gripping

            if cmd['suc_cmd'] is not None:
                if type(cmd['suc_cmd']) is not str:
                    self.suction.gripper_suction_deg(cmd['suc_cmd'])
                    self.suction_angle = cmd['suc_cmd']
                elif 'On' in cmd['suc_cmd']:
                    self.suction.gripper_vacuum_on()
                elif 'calibration' in cmd['suc_cmd']:
                    self.suction.gripper_calibration()
                elif 'Off' in cmd['suc_cmd']:
                    self.suction.gripper_vacuum_off()
            
            if cmd['gripper_cmd'] is not None:
                if 'active' in cmd['gripper_cmd']:
                    self.gripper.gripper_setting()
                elif 'reset' in cmd['gripper_cmd']:
                    self.gripper.gripper_reset()
                elif 'open' in cmd['gripper_cmd']:
                    self.gripper.gripper_open()
                elif 'close' in cmd['gripper_cmd']:
                    self.gripper.gripper_close()
                elif 'grap_alcohol' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    #print('-------------gggg------------')
                    self.gripper.gripper_pos(57)
                elif 'grap_bottle' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    #print('-------------gggg------------')
                    self.gripper.gripper_pos(117)
                elif 'squeeze' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    self.gripper.gripper_pos(100)
                elif 'grap_rag' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    self.gripper.gripper_pos(180)
                elif 'grap_scratcher' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    self.gripper.gripper_pos(200)
                elif 'grap_pose_point' in cmd['gripper_cmd']:
                    #self.gripper.gripper_setting(255, 150)
                    self.gripper.gripper_pos(210)
        # print("is_busy ", self.is_busy, "occupied ", self.occupied, "cmd_queue.empty() ", self.__cmd_queue.empty(), "cmd_queue_2nd.empty() ",self.__cmd_queue_2nd.empty())
        if not self.is_busy and not self.occupied:
            if self.__cmd_queue.empty() and self.__cmd_queue_2nd.empty():
                self.status = Status.idle
        elif not self.is_busy and self.occupied:
            if self.__cmd_queue.empty():
                self.status = Status.occupied
        elif not self.status == Status.grasping:
            self.status = Status.busy
# if __name__ == '__main__':
#     rospy.init_node('test_arm_task')
#     print("Test arm task script")
    
#     a = ArmTask('right_arm',en_sim = True)
#     #rospy.sleep(0.3)

#     a.set_speed(100)
#     a.jointMove(0, (0, -1, 0, 1, 0, 0, 0))
#     a.set_speed(20)
#     a.wait_busy()
    
#     a.ikMove('p2p', (0, -0.3, -0.9), (0, 0, 0), 30) 
#     a.set_speed(100)
#     a.wait_busy()
    
#     a.noa_move_suction('p2p', -45, n=0, s=0, a=-0.1)
#     a.wait_busy()
        
#     a.singleJointMove(0,-0.2)
#     a.wait_busy()
        
#     a.jointMove(0, (0, -1, 0, 1, 0, 0, 0))
#     a.wait_busy()
        
#     a.singleJointMove(2,0.5)
#     a.wait_busy()
        
#     a.relative_move_pose('p2p', (0, 0.1, 0) )
#     a.wait_busy()
    
#     a.back_home()
#     a.wait_busy()
