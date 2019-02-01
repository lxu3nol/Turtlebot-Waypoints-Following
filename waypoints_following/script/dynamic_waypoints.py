#!/usr/bin/env python

## References
## http://www.hotblackrobotics.com/en/blog/2018/01/29/action-client-py/
## Ryan's code - goleap2.py
## Solved the problem of shutting down while waiting for new waypoints (1/30/19)

import rospy
import actionlib                                            ## Brings in the SimpleActionClient
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal ## Brings in the .action file and messages used by the move base action
from waypoints_following.msg import Points                  ## Import new message type "Points.msg"
from markers_1 import mark                                  ## Import class "mark" from markers_1.py
from nav_msgs.msg import Odometry

class move():
    def __init__(self):
        self.m = 0
        self.count = 0
        self.check = False
        self.pointseq = rospy.get_param('p_seq')
        self.points_seq = self.points_generator(self.pointseq)
        self.n = len(self.points_seq)
        self.odom_sub = True

        rospy.init_node('movebase_client_py', disable_signals=True) ## You can use KeyboardInterrupt by adding disable_signals=True

        ## Tell user how to stop TurtleBot
    	# rospy.loginfo("To stop TurtleBot CTRL + C")
        ## What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("goals_generator", Points, self.callback)
        rospy.Subscriber("/odom", Odometry, self.callback1)

        while not rospy.is_shutdown():
            while self.odom_sub:
                rospy.loginfo("Waiting for new mission...")
                time.sleep(2)

            while(self.m < self.n):
                result = self.movebase_client()
                if result and self.check == True:
                    self.m = 0
                    self.check = False
                else:
                    self.m += 1
            rospy.loginfo("Goal execution done!")
            time.sleep(2)
        rospy.spin()

    ## Callback function from subscribing to goals_generator topic
    ## Capture new waypoints from goals generator topic
    def callback(self, data):
        self.new_points_seq = self.points_generator(data.points)
        marking = mark()
        ## When new goals are assigned
        if self.new_points_seq != self.points_seq:
            if self.count > 0:
                # rospy.loginfo("Stop TurtleBot")
                ## Stop Turtlebot from moving
                self.client.cancel_goal()
                ## Clear all markers in rviz
                marking.clear_text_in_rviz(self.points_seq)
                marking.wait_for_time()
                self.check = True
            self.points_seq = self.new_points_seq
            self.n = len(self.points_seq)
            self.m = 0
            ## Show all markers in rviz
            marking.show_text_in_rviz(self.points_seq)
            marking.wait_for_time()
            self.count += 1
        print self.points_seq

    ## Callback function from subscribing to /odom topic
    ## Prevent Turtlebot from shutdown/repeating the previous waypoints while waiting for new waypoints
    def callback1(self, msg):
        x_curr = round(msg.pose.pose.position.x,2)
        y_curr = round(msg.pose.pose.position.y,2)
        x_last = self.points_seq[self.n-1][0]
        y_last = self.points_seq[self.n-1][1]
        # rospy.loginfo("Turtlebot current point (%s, %s) last point (%s, %s)", x_curr, y_curr, x_last, y_last)
        ## Turtlebot stays in place when it is not assigned with new waypoints
        if abs(x_last - x_curr) < 0.50 and abs(y_last - y_curr) < 0.50:
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
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        ## Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        ## Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
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
        rospy.loginfo("Mission is aborted!")
        self.client.cancel_goal()
	    ## sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
