from flexbe_core.proxy import ProxyServiceCaller
from strategy.srv import timda_mobile_state, timda_mobile_state_Response
import rospy

TIMDA_SERVER = "Timda_mobile"


class WaitTimdaMobile(EventState):
	'''
	Wait for timda_mobile to Goal.

	-- robot_name               string          Robots name to move
	-- en_sim					bool			Use real robot or Gazebo

	<= done 									Robot move done.
	<= failed 									Robot move failed.
	'''
	
	def __init__(self, robot_name):
		"""Constructor"""
		super(WaitTimdaMobile, self).__init__(outcomes=['done', 'failed'])
		self.robot_name = robot_name
		self._timda_mobile_service = TIMDA_SERVER
		self.timda_mobile_client = ProxyServiceCaller({self._timda_mobile_service: timda_mobile_state})
		self.timda_mobile_Goal = None
		self.timda_mobile_status = None
	# def __set_server__(self):
	# 	self._timda_mobile_service = 'timda_mobile_state'
	# 	self.timda_mobile_server = ProxyServiceCaller({
	# 	    self.mobile_service:
	# 	    timda_mobile_state})

	def execute(self):
		'''
		Execute this state
		'''

		if self.timda_mobile_status == Status.finish:
			return 'done'
		elif self.status == Status.ik_fail or self.status == Status.emergency_stop:
			return 'failed'
	
	def on_enter(self, userdata):
		#self.__set_server__()
		self.timda_mobile_status = self.timda_mobile_client.call(self._timda_mobile_service, action_goal)	