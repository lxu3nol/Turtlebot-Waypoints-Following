#!/usr/bin/env python

## References
## http://www.hotblackrobotics.com/en/blog/2018/01/29/action-client-py/

import rospy

## Brings in the SimpleActionClient
import actionlib
## Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

points_seq = rospy.get_param('/waypoints/p_seq')
n = len(points_seq)
m = 0

def movebase_client():
    global points_seq
    global m

    ## Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    ## Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    ## Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    ## Move towards the desired goal
    goal.target_pose.pose.position.x = points_seq[m][0]
    goal.target_pose.pose.position.y = points_seq[m][1]
    ## No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    ## Sends the goal to the action server.
    client.send_goal(goal)
    ## Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    ## If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.loginfo("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        ## Result of executing the action
        return client.get_result()

## If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       ## Initializes a rospy node to let the SimpleActionClient publish and subscribe
       while(m < n):
           rospy.init_node('movebase_client_py')
           result = movebase_client()
           if result:
               m += 1
       rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
