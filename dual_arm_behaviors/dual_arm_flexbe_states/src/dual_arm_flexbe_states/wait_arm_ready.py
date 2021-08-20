from flexbe_core.proxy import ProxySubscriberCached
from strategy.msg import ArmStatus
import rospy
from enum import IntEnum
from flexbe_core import EventState

class Status(IntEnum):
    idle             = 0
    finish           = 1


class WaitArmReady(EventState):
    '''
    Wait for timda_mobile to Goal.
    -- robot_name               string          Robots name to move
    -- en_sim					bool			Use real robot or Gazebo

    <= done 									Robot move done.
    <= failed 									Robot move failed.
    '''
    
    def __init__(self, en_sim, robot_name):
        '''
        Constructor
        '''
        super(WaitArmReady, self).__init__(outcomes=['done'])

        self.robot_name = robot_name
        self.en_sim = en_sim
        self.status = Status.idle
        self.__set_pubSub()

    def __set_pubSub(self):
        if self.robot_name == 'right_arm':
            robot_name = 'left_arm'
        else:
            robot_name = 'right_arm'
        self.arm_ready_topic = str(robot_name) +'/arm_ready'
        self.__status_sub = ProxySubscriberCached({
            self.arm_ready_topic:
            ArmStatus})

    def __status_callback(self, msg):
        if 'Move To Goal' in msg.status:
            self.status = Status.finish
        else:
            rospy.logwarn('Unknow Status')



    
    def execute(self, userdata):
        '''
        Execute this state
        '''
        if self.__status_sub.has_msg(self.timda_mobile_status_topic):
            msg = self.__status_sub.get_last_msg(self.timda_mobile_status_topic)
            self.__status_callback(msg)

        if self.status == Status.finish:
            return 'done'

    def on_enter(self, userdata):
        self.status = Status.idle
        self.__status_sub.remove_last_msg(self.timda_mobile_status_topic)

        

