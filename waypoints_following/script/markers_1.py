#!/usr/bin/env python

## References
## http://wiki.ros.org/rviz/DisplayTypes/Marker
## https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers
## https://answers.ros.org/question/11135/plotting-a-markerarray-of-spheres-with-rviz/
## https://answers.ros.org/question/73051/using-makers-array-in-rviz/

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from waypoints_following.msg import Points

class mark():
    def __init__(self):
        self.marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        rospy.sleep(0.5)

    def wait_for_time(self):
        """Wait for simulated time to begin.
        """
        while rospy.Time().now().to_sec() == 0:
            pass

    ## Show markers in rviz
    def show_text_in_rviz(self, points_seq):
        m = 0
        n = len(points_seq)
        markerArray = MarkerArray()
        ## Small points that show the sequence of goals
        while(m < n):
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.id = m
            marker.action = Marker.ADD
            ## the coordinates of markers have to be deducted by the coordinates of initial posisiton of TurtleBot (Only if we change initial pose of TurtleBot)
            # marker.pose = Pose(Point(self.points_seq[self.m][0]-1.0, self.points_seq[self.m][1]-1.0, 0.0), Quaternion(0, 0, 0, 1))
            marker.pose = Pose(Point(points_seq[m][0], points_seq[m][1], 0.0), Quaternion(0, 0, 0, 1))
            marker.scale = Vector3(0.06, 0.06, 0.06)
            marker.header = Header(frame_id='map') ## frame_id='base_link' creates markers that move around with TurtleBot
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
            markerArray.markers.append(marker)
            m += 1
        m = 0
        ## Numbers that show the sequence of goals
        while(m < n):
            marker1 = Marker()
            marker1.type = Marker.TEXT_VIEW_FACING
            marker1.id = m
            marker1.action = Marker.ADD
            # marker1.pose = Pose(Point(self.points_seq[self.m][0]-1.0, self.points_seq[self.m][1]-1.0, 0.2), Quaternion(0, 0, 0, 1))
            marker1.pose = Pose(Point(points_seq[m][0], points_seq[m][1], 0.2), Quaternion(0, 0, 0, 1))
            marker1.scale = Vector3(0.5, 0.5, 0.5)
            marker1.header = Header(frame_id='map')
            marker1.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
            marker1.text = '%d'%m
            markerArray.markers.append(marker1)
            m += 1
        m = 0

        ## Renumber the marker IDs
        id = 0
        for x in markerArray.markers:
            x.id = id
            id += 1

        ## Publish the MarkerArray
        self.marker_publisher.publish(markerArray)

    ## Clear all markers in rviz
    def clear_text_in_rviz(self, points_seq):
        m = 0
        n = len(points_seq)
        markerArray = MarkerArray()
        while(m < n):
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.id = m
            marker.action = Marker.DELETE
            # marker.pose = Pose(Point(self.points_seq[self.m][0]-1.0, self.points_seq[self.m][1]-1.0, 0.0), Quaternion(0, 0, 0, 1))
            marker.pose = Pose(Point(points_seq[m][0], points_seq[m][1], 0.0), Quaternion(0, 0, 0, 1))
            marker.scale = Vector3(0.06, 0.06, 0.06)
            marker.header = Header(frame_id='map')
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
            markerArray.markers.append(marker)
            m += 1
        m = 0
        while(m < n):
            marker1 = Marker()
            marker1.type = Marker.TEXT_VIEW_FACING
            marker1.id = m
            marker1.action = Marker.DELETE
            # marker1.pose = Pose(Point(self.points_seq[self.m][0]-1.0, self.points_seq[self.m][1]-1.0, 0.2), Quaternion(0, 0, 0, 1))
            marker1.pose = Pose(Point(points_seq[m][0], points_seq[m][1], 0.2), Quaternion(0, 0, 0, 1))
            marker1.scale = Vector3(0.5, 0.5, 0.5)
            marker1.header = Header(frame_id='map')
            marker1.color = ColorRGBA(1.0, 0.0, 0.0, 0.8)
            marker1.text = '%d'%m
            markerArray.markers.append(marker1)
            m += 1
        m = 0

        id = 0
        for x in markerArray.markers:
            x.id = id
            id += 1

        self.marker_publisher.publish(markerArray)
