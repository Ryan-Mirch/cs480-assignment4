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

        # Customize the following values so they are appropriate for your location
        position1 = {'x': -2.82, 'y' : 4.77}
        quaternion1 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.27, 'r4' : 0.96}

	position2 = {'x': -.508, 'y' : 8.947}
        quaternion2 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.1212, 'r4' : 0.9926}

	position3 = {'x': -4.905, 'y' : 7.9167}
        quaternion3 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.986, 'r4' : 0.1647}

	position4 = {'x': -2.073, 'y' : 4.2743}
        quaternion4 = {'r1' : 0.000, 'r2' : 0.000, 'r3' : -0.5311, 'r4' : 0.84725}



        rospy.loginfo("Go to (%s, %s) pose", position1['x'], position1['y'])
        success = navigator.goto(position1, quaternion1)	

        rospy.loginfo("Go to (%s, %s) pose", position2['x'], position2['y'])
        success = navigator.goto(position2, quaternion2)

        rospy.loginfo("Go to (%s, %s) pose", position3['x'], position3['y'])
        success = navigator.goto(position3, quaternion3)

        rospy.loginfo("Go to (%s, %s) pose", position4['x'], position4['y'])
        success = navigator.goto(position4, quaternion4)

        

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
	rospy.loginfo("Ctrl-C caught. Quitting")
