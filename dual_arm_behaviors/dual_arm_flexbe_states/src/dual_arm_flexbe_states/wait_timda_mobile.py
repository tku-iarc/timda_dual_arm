from flexbe_core.proxy import ProxySubscriberCached
from strategy.msg import TimdaMobileStatus
import rospy
from enum import IntEnum
from flexbe_core import EventState

class Status(IntEnum):
	idle             = 0
	busy             = 1
        failed_to_arrive = 2
	finish           = 3


class WaitTimdaMobile(EventState):
	'''
	Wait for timda_mobile to Goal.

	-- en_sim					bool			Use real robot or Gazebo

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''
	
	def __init__(self, en_sim):
		'''
		Constructor
		'''
		super(WaitTimdaMobile, self).__init__(outcomes=['done', 'failed'])

		#self.robot_name = robot_name
		self.en_sim = en_sim
		self.status = Status.idle
		self.__set_pubSub()

	def __set_pubSub(self):
		self.timda_mobile_status_topic = '/timda_mobile_status'
		self.__status_sub = ProxySubscriberCached({
		    self.timda_mobile_status_topic:
		    TimdaMobileStatus})

	def __status_callback(self, msg):
		if 'Move To Goal' in msg.status:
			self.status = Status.busy
			rospy.loginfo('Timda Mobile Move!')
		elif 'Failed' in msg.status:
			self.status = Status.failed_to_arrive
			rospy.logwarn('Can Not Arrive Goal')
		elif 'Arrive' in msg.status:
			self.status = Status.finish
			rospy.loginfo('Arrive Goal!')
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
		elif self.status == Status.failed_to_arrive:
			return 'failed'

	def on_enter(self, userdata):
		self.status = Status.busy
		self.__status_sub.remove_last_msg(self.timda_mobile_status_topic)

		

