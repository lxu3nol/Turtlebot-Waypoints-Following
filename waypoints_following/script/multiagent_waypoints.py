#!/usr/bin/env python

## References
## http://www.hotblackrobotics.com/en/blog/2018/01/29/action-client-py/
## Ryan's code - goleap2.py
## Thw dynamic waypoints following is succeeded with this python script!!

import rospy
import actionlib                                            ## Brings in the SimpleActionClient
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal ## Brings in the .action file and messages used by the move base action
from waypoints_following.msg import Points                  ## Import new message type "Points.msg"
from markers_2 import mark                                  ## Import class "mark" from markers_1.py
from nav_msgs.msg import Odometry

class move():
    def __init__(self):
        self.m = 0
        self.count = 0
        self.check = False
        self.pointseq = rospy.get_param('p_seq')
        self.points_seq = self.points_generator(self.pointseq)
        self.n = len(self.points_seq)
        self.robot_no = rospy.get_param('robot_id')
        self.odom_sub = True

        ## Create the string "turtlebotX/move_base"
    	self.move_base_str = "/turtlebot"
    	self.move_base_str += str(self.robot_no)
    	self.move_base_str += "/move_base"

        ## Create the string "turtlebotX/odom"
    	self.odom_str = "/turtlebot"
    	self.odom_str += str(self.robot_no)
    	self.odom_str += "/odom"

        ## Create the string "robotX_tf/map"
    	self.frame_id_str = "robot"
    	self.frame_id_str += str(self.robot_no)
    	self.frame_id_str += "_tf/map"

        rospy.init_node('movebase_client_py')

        ## Tell user how to stop TurtleBot
    	# rospy.loginfo("To stop TurtleBot CTRL + C")
        ## What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("goals_generator", Points, self.callback)
        rospy.Subscriber(self.odom_str, Odometry, self.callback1)

        while not rospy.is_shutdown():
            while self.odom_sub:
                rospy.loginfo("Turtlebot%s is waiting for new mission...", self.robot_no)
                time.sleep(2)

            while(self.m < self.n):
                result = self.movebase_client()
                if result and self.check == True:
                    self.m = 0
                    self.check = False
                else:
                    self.m += 1
            rospy.loginfo("Turtlebot%s mission completed.", self.robot_no)
            time.sleep(2)
        rospy.spin()

    ## Callback function from subscribing to goals_generator topic
    ## Capture new waypoints from goals generator topic
    def callback(self, data):
        self.new_points_seq = self.points_generator(data.points)
        marking = mark(self.robot_no)
        ## When new goals are assigned
        if self.new_points_seq != self.points_seq:
            if self.count > 0:
                rospy.loginfo("Stop Turtlebot%s", self.robot_no)
                ## Stop Turtlebot from moving
                self.client.cancel_goal()
                ## Clear all markers in rviz
                marking.clear_text_in_rviz(self.points_seq)
                marking.wait_for_time()
                self.check = True
            self.points_seq = self.new_points_seq
            self.n = len(self.points_seq)
            ## Show all markers in rviz
            marking.show_text_in_rviz(self.points_seq)
            marking.wait_for_time()
            self.count += 1
        print self.points_seq

    ## Callback function from subscribing to turtlebotX/odom topic
    ## Prevent Turtlebot from shutdown/repeating the previous waypoints while waiting for new waypoints
    def callback1(self, msg):
        x_curr = round(msg.pose.pose.position.x,2)
        y_curr = round(msg.pose.pose.position.y,2)
        x_next = self.points_seq[self.n-1][0]
        y_next = self.points_seq[self.n-1][1]
        # rospy.loginfo("Turtlebot%s current point (%s, %s) next point (%s, %s)", self.robot_no, x_curr, y_curr, x_next, y_next)
        ## Turtlebot stays in place when it is not assigned with new waypoints
        if abs(x_next - x_curr) < 0.50 and abs(y_next - y_curr) < 0.50:
            self.odom_sub = True
        else:
            self.odom_sub = False

    ## Translate a list of points into coordinates
    def points_generator(self, p):
        i = 0
        P = []
        while i<len(p):
            P.append(p[i:i+3])
            i+=3
        return P

    ## Send goals to move the Turtlebot
    def movebase_client(self):
        ## Create an action client called "move_base" with action definition file "MoveBaseAction"
        ## SimpleActionClient is a class in actionlib
        self.client = actionlib.SimpleActionClient(self.move_base_str,MoveBaseAction)

        ## Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        ## Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id_str
        goal.target_pose.header.stamp = rospy.Time.now()
        ## Move towards the desired goal
        goal.target_pose.pose.position.x = self.points_seq[self.m][0]
        goal.target_pose.pose.position.y = self.points_seq[self.m][1]
        ## No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0

        ## Sends the goal to the action server.
        self.client.send_goal(goal)
        ## Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        ## If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.loginfo("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            ## Result of executing the action
            return self.client.get_result()

    def shutdown(self):
        ## stop Turtlebot from moving
        rospy.loginfo("Mission is aborted.")
        self.client.cancel_goal()
	    ## sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
