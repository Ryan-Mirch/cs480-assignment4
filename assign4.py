#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

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

        # Create goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Send goal
        self.move_base.send_goal(goal)

	# Wait for 2 minutes to reach goal
	success = self.move_base.wait_for_result(rospy.Duration(120)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # Reached goal
            result = True
	    rospy.loginfo("goal reached")	
        else:
            self.move_base.cancel_goal()
	    rospy.loginfo("goal canceled")

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
	
	# the 4 points and their respective quaternions
        point1 = {'x': -2.82, 'y' : 4.77}
        quaternion1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.27, 'r4' : 0.96}

	point2 = {'x': -.508, 'y' : 8.947}
        quaternion2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.1212, 'r4' : 0.9926}

	point3 = {'x': -4.905, 'y' : 7.9167}
        quaternion3 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.986, 'r4' : 0.1647}

	point4 = {'x': -2.073, 'y' : 4.2743}
        quaternion4 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.5311, 'r4' : 0.84725}


	# giving the commands to travel to the 4 points
        rospy.loginfo("Go to point 1 at position (%s, %s)", point1['x'], point1['y'])
        success = navigator.goto(point1, quaternion1)	
		
	if not success:
		rospy.loginfo("Goal was not able to be reached, please restart me.")

        rospy.loginfo("Go to point 2 at position (%s, %s) ", point2['x'], point2['y'])
        success = navigator.goto(point2, quaternion2)
	
	if not success:
		rospy.loginfo("Goal was not able to be reached, please restart me.")

        rospy.loginfo("Go to point 3 at position (%s, %s) ", point3['x'], point3['y'])
        success = navigator.goto(point3, quaternion3)
	
	if not success:
		rospy.loginfo("Goal was not able to be reached, please restart me.")

        rospy.loginfo("Go to point 4 at position (%s, %s) ", point4['x'], point4['y'])
        success = navigator.goto(point4, quaternion4)
	
	if not success:
		rospy.loginfo("Goal was not able to be reached, please restart me.")
	else:
		rospy.loginfo("All 4 points reached!")
	
        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
	rospy.loginfo("Ctrl-C caught. Quitting")
