#!/usr/bin/env python


from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
import rospy
import tf
from flexbe_core import EventState
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached

from std_msgs.msg import Bool
from math import pi, radians
from enum import IntEnum
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
# from manipulator_h_base_module_msgs.msg import IK_Cmd, JointPose
# from manipulator_h_base_module_msgs.msg import P2PPose, KinematicsPose



_POS = (0, 0, 0)
_ORI = (0, 0, 0)
_PHI = 0

class Status(IntEnum):
    ready           = 0
    moving          = 1
    stop            = 2
    finish          = 3

class Robotiq2FGripperState(EventState):
    '''
    Control robotiq_2f85_gripper.

    -- robot_name               string          Robots name to move
    -- en_sim					bool			Use real robot or Gazebo
    -- gripper_cmd			    string			gripper command to execute

    <= done 									gripper move done.
    <= failed 									gripper move failed.
    '''


    def __init__(self, robot_name, en_sim, gripper_cmd):
        '''
        Constructor
        '''
        super(Robotiq2FGripperState, self).__init__(outcomes=['done'])

        self.robot_name = robot_name
        self.en_sim = en_sim
        self.gripper_cmd = gripper_cmd
        self.speed = 255
        self.force = 200
        self.curr_pos = 0
        self.status = Status.ready
        self.__set_pubSub()

    def __set_pubSub(self):
        print ("[Arm] name space : " + str(self.robot_name))

        self.gripper_topic = str(self.robot_name) + '/Robotiq2FGripperRobotOutput'
        self.__gripper_pub = ProxyPublisher({
            self.gripper_topic:
            outputMsg})

        self.gripper_status_topic = str(self.robot_name) + '/Robotiq2FGripperRobotInput'
        self.__gripper_sub = ProxySubscriberCached({
            self.gripper_status_topic:
            inputMsg})


    def __status_callback(self, msg):
        if self.gripper_cmd == 'active':
            self.status = Status.ready
            rospy.loginfo('Gripper Ready!')
            if msg.gSTA == 3 and msg.gACT == 1:
                self.status = Status.finish
        elif self.gripper_cmd == 'open':
            if msg.gPO <= 13:
                self.status = Status.finish
                rospy.loginfo('Gripper open!')
        elif self.gripper_cmd == 'close':
            if msg.gPO >= 230:
                self.status = Status.finish
                rospy.loginfo('Gripper close!')
        elif self.gripper_cmd == 'reset':
            if msg.gSTA == 0 and msg.gACT == 0:
                self.status = Status.finish
                rospy.loginfo('Finish reset!')
        else:
            if msg.gPR == self.gripper_cmd:
                self.status = Status.finish
                rospy.loginfo('Finish Grap!')

        # if msg.gSTA == 3 and msg.gACT == 1:
        #     self.status = Status.ready
        #     rospy.loginfo('Gripper Ready!')
        # elif msg.gGTO == 1 and msg.gOBJ == 0:
        #     self.status = Status.moving
        #     rospy.logwarn('Gripper Moving!')
        # elif msg.gPR == self.gripper_cmd:
        #     self.status = Status.stop
        #     rospy.logwarn('Gripper Stop!')
        #     self.status = Status.finish
        #     rospy.loginfo('Finish Grab!')
        # # elif msg.gACT == 1 and msg.gGTO == 1 and msg.gSTA == 1 and msg.gOBJ == 3 and msg.gPR == 0  and msg.gPO == 3:
        # #     self.status = Status.finish
        # #     rospy.loginfo('Finish Grab!')
        # else:
        #     rospy.logwarn('Unknow Status')

    def grip_status_callback(self, msg):
        self.is_grip = msg.gOBJ
        self.curr_pos = msg.gPO 
    def gripper_reset(self):
        cmd = outputMsg()
        cmd.rACT = 0
        cmd.rGTO = 1
        cmd.rATR = 0
        cmd.rPR  = 0
        cmd.rSP  = 255
        cmd.rFR  = 150
        self.__gripper_pub.publish(self.gripper_topic, cmd)
        rospy.sleep(0.5)    
    def gripper_open(self):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0 
        cmd.rPR  = 0
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.__gripper_pub.publish(self.gripper_topic, cmd)
        rospy.sleep(0.5)    
    def gripper_close(self):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0 
        cmd.rPR  = 255
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.__gripper_pub.publish(self.gripper_topic, cmd)
        rospy.sleep(0.3)    
    def gripper_setting(self, speed = -1, force = -1):
        self.speed = self.speed if speed == -1 else speed
        self.force = self.force if force == -1 else force
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0 
        cmd.rPR  = self.curr_pos
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        print(self.gripper_topic)
        self.__gripper_pub.publish(self.gripper_topic, cmd)


    def gripper_pos(self, pos):
        #print('===================get in pos====================')
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rATR = 0 
        cmd.rPR  = int(pos)
        cmd.rSP  = int(self.speed)
        cmd.rFR  = int(self.force)
        self.__gripper_pub.publish(self.gripper_topic, cmd)


    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.__gripper_sub.has_msg(self.gripper_status_topic):
            msg = self.__gripper_sub.get_last_msg(self.gripper_status_topic)
            self.__status_callback(msg)

        if self.status == Status.finish:
            return 'done'


    def on_enter(self, userdata):
        print(userdata)
        print(self.gripper_cmd)
        self.status = Status.ready
        # self.__gripper_sub.remove_last_msg(self.gripper_status_topic)
        if self.gripper_cmd == 'active':
            self.gripper_setting(255, 150)
        elif self.gripper_cmd == 'open':
            self.gripper_open()
        elif self.gripper_cmd == 'close':
            self.gripper_close()
        elif self.gripper_cmd == 'reset':
            self.gripper_reset()
        else:
            self.gripper_pos(int(self.gripper_cmd))