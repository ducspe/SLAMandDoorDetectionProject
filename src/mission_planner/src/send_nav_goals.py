#!/usr/bin/env python

import actionlib
import math
import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# (x, y, yaw)
#goals = [(4.0, 0.0, math.pi), (0.0, 0.0, 0), (-2.0, -4.0, math.pi),(4.0, 0.0, math.pi)]
goals = [(4.5, 0.0, math.pi), (5.5, 3.0, 0),(4.5, 0.0, math.pi), (0.0, 0.0, 0)] *3
#goals = [(0.0, 0.0,0.0), (0.0, 0.0, 0)]

class SendNavGoals():
    def __init__(self):
        rospy.init_node('send_nav_goals')
        self.goal_idx = 0

        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        rospy.loginfo("Connected to move_base server")
        rospy.loginfo("Starting goals achievements...")

        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose " + str(self.goal_idx + 1) + " is now being processed by the action server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal " + str(self.goal_idx) + ": " + str(feedback))
        rospy.loginfo("Feedback for goal pose " + str(self.goal_idx + 1) + " received")

    def done_cb(self, status, result):
        self.goal_idx += 1
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html

        if status == 2:
            rospy.loginfo("Goal pose " + str(self.goal_idx) + " received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose " + str(self.goal_idx) + " reached")
            # Wait for 5 seconds at the goal
            rospy.loginfo("Waiting for 2 seconds...")
            rospy.sleep(2.0)

            try:
                self.send_goal(goals.pop(0))
            except IndexError:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo("Goal pose " + str(self.goal_idx) + " was aborted by the action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_idx) + " aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose " + str(self.goal_idx) + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + str(self.goal_idx) + " rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose " + str(self.goal_idx) + " received a cancel request before it started executing, successfully cancelled!")

    def send_goal(self, goal_pose):
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()

        move_base_goal.target_pose.pose.position.x = goal_pose[0]
        move_base_goal.target_pose.pose.position.y = goal_pose[1]

        quaternion = quaternion_from_euler(0.0, 0.0, goal_pose[2])
        move_base_goal.target_pose.pose.orientation.x = quaternion[0]
        move_base_goal.target_pose.pose.orientation.y = quaternion[1]
        move_base_goal.target_pose.pose.orientation.z = quaternion[2]
        move_base_goal.target_pose.pose.orientation.w = quaternion[3]

        self.client.send_goal(move_base_goal, self.done_cb, self.active_cb)

    def movebase_client(self):
        self.send_goal(goals.pop(0))

        rospy.spin()

if __name__ == '__main__':
    try:
        SendNavGoals()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")







# # Use below as blueprint
# # theta: from -pi to pi (2pi in total) depending on the world x-axis

# x = None
# y = None
# theta = None

# def newOdom(msg):
#     global x
#     global y
#     global theta

#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y

#     rot_q = msg.pose.pose.orientation
#     (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

# def createPoint(waypoint):
#     point = Point()
#     point.x = waypoint[0]
#     point.y = waypoint[1]

#     return point

# # a, b: (x, y)
# def angleBetweenTwoVectors(a, b):
#     # cos alpha = (a * b) / (mag(a) * mag(b))
#     dot_prod = a[0] * b[0] + a[1] * b[1]
#     mag_prod = sqrt(a[0] * a[0] + a[1] * a[1]) * sqrt(b[0] * b[0] + b[1] * b[1])
#     return acos(dot_prod / mag_prod)

# rospy.init_node('send_nav_goals')

# sub = rospy.Subscriber('/p3dx/odom', Odometry, newOdom)

# pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)

# speed = Twist()

# r = rospy.Rate(4)

# # Input the waypoints you want to traverse
# waypoints = [(0, 1), (3, 0), (0, 0), (3, 1)]

# # Converts each waypoint to a Point()
# waypoints = list(map(createPoint, waypoints))

# current_waypoint_idx = 0

# # Only continue if all necessary values from the robot are retrieved
# while x == None or y == None or theta == None:
#     pass

# while not rospy.is_shutdown():
#     goal = waypoints[current_waypoint_idx]
#     print('Current waypoint: ' + str(current_waypoint_idx))

#     inc_x = goal.x - x
#     inc_y = goal.y - y

#     angle_to_goal = atan2(inc_y, inc_x)
#     dist_to_goal = sqrt((goal.x - x) * (goal.x - x) + (goal.y - y) * (goal.y - y))

#     ####

#     print('Angle between robot and x-axis: ' + str(angleBetweenTwoVectors([inc_x, inc_y], [1.0, 0.0])))  # the same as atan2(...)

#     ####

#     print('Angle to goal: ' + str(angle_to_goal))

#     # angle_diff = abs(angle_to_goal - theta)
#     angle_diff = theta - angle_to_goal
#     print(angle_diff)
#     angle_threshold = 0.5

#     if angle_diff > angle_threshold:
#         speed.linear.x = 0.0
#         speed.angular.z = -0.3
#     elif angle_diff < -angle_threshold:
#         speed.linear.x = 0.0
#         speed.angular.z = 0.3
#     else:
#         speed.linear.x = 0.5
#         speed.angular.z = 0.0

#     speed.linear.x = 0.0
#     speed.angular.z = 0.3

#     if dist_to_goal < 0.3:
#         current_waypoint_idx = (current_waypoint_idx + 1) % 4

#     pub.publish(speed)
#     r.sleep()
