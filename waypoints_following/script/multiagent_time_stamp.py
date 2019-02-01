#!/usr/bin/env python
# license removed for brevity

## References
## http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv
## http://wiki.ros.org/msg
## https://answers.ros.org/question/271620/importerror-no-module-named-xxxxmsg/

import rospy
import time
from waypoints_following.msg import Points

time_stamps = [100, 120]
rand_points = dict([(time_stamps[0],[-1.0, 0.0, 0.0, 3.0, 0.0, 0.0, 1.0, -1.0, 0.0]),
                    (time_stamps[1],[3.0, -2.0, 0.0, -2.0, -2.0, 0.0, -2.0, 2.0, 0.0, 3.0, 2.0, 0.0])])
now = 0
x = 0
a = rospy.get_param('p_seq')

def timer():
    global now
    time.sleep(1)
    now += 1
    return now

def generator():
    global x
    global a
    pub = rospy.Publisher('goals_generator', Points, queue_size=10)
    rospy.init_node('goals', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    message = Points()
    message.points = a

    while not rospy.is_shutdown():
        T = timer()
        # rospy.loginfo(T)
        if T == time_stamps[x]:
            a = rand_points[time_stamps[x]]
            message.points = a
            x += 1
            if x > len(time_stamps)-1:
                x = 0
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        generator()
    except rospy.ROSInterruptException:
        pass
