# Collaboration Between Humans and Robots
Goal of the project: Program the Turtlebot so that it helps to move movable objects in resilient operations. 

How it works: The Turtlebot will first receive coordinates from Crazyflies (nano quadcopters) which fly around the operation area. After receiving the coordinates, the Turtlebot will move to the assigned coordinates to move the objects.

There are three python scripts which control the Turtlebot in different ways:
1) waypoints.py which controls one Turtlebot.
2) dynamic_waypoints.py which controls one Turtlebot and receives sequence of waypoints at certain time period.
3) multiagent_waypoints.py which controls two Turtlebots and receives sequence of waypoints at certain time period.
