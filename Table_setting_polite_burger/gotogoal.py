#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

def getPosition(msg):
		print msg.pose.pose

class GoToPose():
	def __init__(self):

		self.goal_sent = False

		# What to do if shut down (e.g. Ctrl-C or failure)
		rospy.on_shutdown(self.shutdown)
	
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

	def goto(self, pos, quat):

		# Send a goal
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
		Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

			self.goal_sent = False
		return result
	

	def shutdown(self):
		if self.goal_sent:
			self.move_base.cancel_goal()
			rospy.loginfo("Stop")
			rospy.sleep(1)

def position_to_go(data):
	"""Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))

        vel_msg = Twist()
	rospy.loginfo(data)
	if data.data == "start" :
		rospy.sleep(0)
		position_to = {'x': -1.61, 'y' : -2.55}
		position_back = {'x': 0.42, 'y' : 0.02}
		quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		rospy.loginfo("Go to (%s, %s) pose", position_to['x'], position_to['y'])
		success_first = navigator.goto(position_to, quaternion)
		rospy.sleep(20)
		rospy.loginfo("Go to (%s, %s) pose", position_back['x'], position_back['y'])
		success_second = navigator.goto(position_back, quaternion)
		# Customize the following values so they are appropriate for your location
		if (success_second) and (success_first):
			rospy.loginfo("Hooray, reached the desired pose")
		else:
			rospy.loginfo("The base failed to reach the desired pose")

			# Sleep to give the last log messages time to be sent
			rospy.sleep(1)

if __name__ == '__main__':

		rospy.init_node('nav_test', anonymous=False)
		navigator = GoToPose()
		rospy.Subscriber('/listener', String , position_to_go)

		rospy.spin()

#		rospy.loginfo("Ctrl-C caught. Quitting")
