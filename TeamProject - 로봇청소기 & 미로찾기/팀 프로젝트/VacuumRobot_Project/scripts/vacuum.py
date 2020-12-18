#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2


class VacuumRun:
    abs_poseX = 0
    abs_poseY = 0
    poselist = []
    routelist = []
    statelist = []
    reverselist = []
    outlinelist = []
    fname = '/home/lsj/practice/catkin_ws/src/deu_maze/maps/room.png'

    def __init__(self):
        self.outline_search = True
        self.outline_start = True

        self.driving_forward = True
        self.driving_rotation = False
        self.start = True
        self.vc_continue = False
        self.setpose = True

        self.vacuum_on = Twist()
        self.straight = Twist()
        self.turn = Twist()

        self.rate = rospy.Rate(10)
        self.endpoint = 0
        self.drivepoint = 0
        self.curpose = 0

        self.range_ahead = 1
        self.range_left = 1
        self.range_right = 1

        self.poseX = 0
        self.poseY = 0
        self.orientationZ = 0
        self.orientationW = 0
        self.rotation = 0
        self.state = 0
        self.code = 0

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.image = cv2.imread(self.fname, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        self.range_ahead = msg.ranges[len(msg.ranges) / 2]
        self.range_left = msg.ranges[int(len(msg.ranges) / 1.3)]
        self.range_right = msg.ranges[len(msg.ranges) / 4]

    def odom_callback(self, msg):
        self.poseX = msg.pose.pose.position.x
        self.poseY = msg.pose.pose.position.y
        self.orientationZ = msg.pose.pose.orientation.z
        self.orientationW = msg.pose.pose.orientation.w
        self.abs_poseX = self.poseX + 0.5
        self.abs_poseY = self.poseY + 9.5

    def save_pose(self):
        if self.outline_search is True:
            self.outlinelist.append([round(self.abs_poseX, 1), round(self.abs_poseY, 1)])
            self.endpoint = self.endpoint + 1
        else:
            self.routelist.append([round(self.abs_poseX, 1), round(self.abs_poseY, 1)])

        if self.vc_continue is True:
            self.poselist.append([round(self.abs_poseX, 1), round(self.abs_poseY, 1)])
            print(round(self.abs_poseX, 1), round(self.abs_poseY, 1))

            self.curpose = [round(self.abs_poseX, 1), round(self.abs_poseY, 1)]
            for i in range(len(self.outlinelist)):
                if self.outlinelist[i] == self.curpose:
                    self.drivepoint = self.drivepoint + 1
                    print("EndPoint:", self.endpoint)
                    print("DrivePoint:", self.drivepoint)

    def save_state(self):
        if self.vc_continue is True:
            self.statelist.append(self.state)

    def go(self):
        for i in range(5):
            self.straight.linear.x = 1
            self.cmd_vel_pub.publish(self.straight)
            self.rate.sleep()
        self.save_pose()
        self.save_state()

    def botturn(self, type):
        if type == 0:
            if self.state < 3:
                self.state = self.state + 1
            else:
                self.state = 0
            for i in range(10):
                self.turn.angular.z = math.radians(90)
                self.cmd_vel_pub.publish(self.turn)
                self.rate.sleep()

        elif type == 1:
            if self.state > 0:
                self.state = self.state - 1
            else:
                self.state = 3
            for i in range(10):
                self.turn.angular.z = -math.radians(90)
                self.cmd_vel_pub.publish(self.turn)
                self.rate.sleep()

        elif type == 2:
            if self.state == 0:
                self.state = self.state + 2
            elif self.state == 1:
                self.state = self.state + 2
            elif self.state == 2:
                self.state = self.state - 2
            elif self.state == 3:
                self.state = self.state - 2
            for i in range(20):
                self.turn.angular.z = math.radians(180)
                self.cmd_vel_pub.publish(self.turn)
                self.rate.sleep()

    def outline(self):
        if self.outline_search is True:  # First Time Turn
            if self.outline_start is True:
                while True:
                    if round(self.orientationZ, 2) != -0.71 and round(self.orientationW, 2) != 0.71:
                        self.turn.angular.z = -math.radians(90)
                        self.cmd_vel_pub.publish(self.turn)
                        self.rate.sleep()
                    else:
                        break
                self.outline_start = False
                self.state = 2
                self.save_pose()
            else:
                if self.range_right < 0.7:
                    if self.range_ahead > 0.7:
                        self.go()
                        for i in range(1, len(self.outlinelist)):
                            if self.outlinelist[0] == self.outlinelist[i]:
                                self.botturn(2)
                                self.vc_continue = True
                                self.outline_search = False
                                break
                    elif self.range_ahead < 0.7:
                        self.botturn(0)
                        return 0
                else:
                    self.botturn(1)
                    self.go()
                    return 0

    def goal_pose(self):  # pose
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = 1
        goal_pose.target_pose.pose.position.y = 9
        goal_pose.target_pose.pose.position.z = 0
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0.755
        goal_pose.target_pose.pose.orientation.w = 0.655
        return goal_pose

    def endrun(self):
        for i in range(5):
            self.straight.linear.x = 0
            self.cmd_vel_pub.publish(self.straight)
            self.rate.sleep()
        self.vc_continue = False

        if self.setpose is True:
            if self.range_left > self.range_right:
                self.botturn(0)
                if self.range_ahead > 3.0:
                    self.go()
                    if self.range_left > self.range_right:
                        self.botturn(0)
                        if self.range_ahead > 3.0:
                            self.go()
                            self.setpose = False
                            return 0
                    elif self.range_left < self.range_right:
                        self.botturn(1)
                        if self.range_ahead > 3.0:
                            self.go()
                            self.setpose = False
                            return 0
            elif self.range_left < self.range_right:
                self.botturn(1)
                if self.range_ahead > 3.0:
                    self.go()
                    if self.range_left > self.range_right:
                        self.botturn(0)
                        if self.range_ahead > 3.0:
                            self.go()
                            self.setpose = False
                            return 0
                    elif self.range_left < self.range_right:
                        self.botturn(1)
                        if self.range_ahead > 3.0:
                            self.go()
                            self.setpose = False
                            return 0
        else:
            print("Go home....")
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()
            client.send_goal(self.goal_pose())
            client.wait_for_result()
            print(round(self.abs_poseX, 1), round(self.abs_poseY, 1), round(self.orientationZ, 2),
                  round(self.orientationW, 2))
            print("turn....")
            while True:
                if round(self.orientationZ, 2) != 0.71 and round(self.orientationW, 2) != 0.71:
                    self.turn.angular.z = -math.radians(1)
                    self.cmd_vel_pub.publish(self.turn)
                    self.rate.sleep()
                else:
                    break
            self.botturn(0)
            while self.range_ahead > 0.5 :
                for i in range(4):
                    self.straight.linear.x = 0.7
                    self.cmd_vel_pub.publish(self.straight)
                    self.rate.sleep()
            if 0.5 <= self.abs_poseX <= 1.5 and 8 <= self.abs_poseY <= 9.5:
                for i in range(10):
                    self.straight.linear.x = 0
                    self.cmd_vel_pub.publish(self.straight)
                    self.rate.sleep()
                self.botturn(2)
                while True:
                    if round(self.orientationZ, 2) != 0 and round(self.orientationW, 2) != 1.0:
                        self.turn.angular.z = -math.radians(1)
                        self.cmd_vel_pub.publish(self.turn)
                        self.rate.sleep()
                    else:
                        break
                print("Turtlebot : cleaning end!")
                print(round(self.abs_poseX, 1), round(self.abs_poseY, 1), round(self.orientationZ, 2),
                      round(self.orientationW, 2))
                print("Turtlebot : The area of the area I cleaned is")
                print(len(self.poselist))
                for i in range(len(self.routelist) - 1):
                    self.image = cv2.line(self.image,
                                          (int(self.routelist[i][0] * 108), 1080 - int(self.routelist[i][1] * 108)),
                                          (int(self.routelist[i + 1][0] * 108), 1080 - int(self.routelist[i + 1][1] * 108)),
                                          (0, 0, 255), 10)
                cv2.imwrite('clean_route.png', self.image)
                cv2.imshow('map', self.image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                print("Turtlebot : The route I moved has been saved as 'clean_route.png'")
                exit(0)

    def trace_back(self):
        print("Turtlebot : I go back to places I couldn't explore...")
        self.reverselist = self.routelist[:]
        self.reverselist.reverse()
        self.botturn(2)
        print("this is state", self.state)

        if self.endpoint <= self.drivepoint:
            self.endrun()
        else:
            for i in range(len(self.reverselist) - 1):
                if self.state == 0:
                    if ([round(self.abs_poseX, 1),
                         round(self.abs_poseY, 1) + 0.5] not in self.poselist and self.range_ahead > 0.7) or (
                            [round(self.abs_poseX, 1) - 0.5,
                             round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7) or (
                            [round(self.abs_poseX, 1) + 0.5,
                             round(self.abs_poseY, 1)] not in self.poselist and self.range_right > 0.7):
                        self.vc_continue = True
                        del self.reverselist[:]
                        break
                    else:
                        if self.reverselist[i][1] < self.reverselist[i + 1][1]:
                            self.go()
                        elif self.reverselist[i][1] > self.reverselist[i + 1][1]:
                            self.botturn(2)
                            self.go()
                        else:
                            if self.reverselist[i][0] > self.reverselist[i + 1][0]:
                                self.botturn(0)
                                self.go()
                            elif self.reverselist[i][0] < self.reverselist[i + 1][0]:
                                self.botturn(1)
                                self.go()

                elif self.state == 1:
                    if ([round(self.abs_poseX, 1),
                         round(self.abs_poseY, 1) + 0.5] not in self.poselist and self.range_right > 0.7) or (
                            [round(self.abs_poseX, 1) - 0.5,
                             round(self.abs_poseY, 1)] not in self.poselist and self.range_ahead > 0.7) or (
                            [round(self.abs_poseX, 1),
                             round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_left > 0.7):
                        self.vc_continue = True
                        del self.reverselist[:]
                        break
                    else:
                        if self.reverselist[i][0] > self.reverselist[i + 1][0]:
                            self.go()
                        elif self.reverselist[i][0] < self.reverselist[i + 1][0]:
                            self.botturn(2)
                            self.go()
                        else:
                            if self.reverselist[i][1] > self.reverselist[i + 1][1]:
                                self.botturn(0)
                                self.go()
                            elif self.reverselist[i][1] < self.reverselist[i + 1][1]:
                                self.botturn(1)
                                self.go()


                elif self.state == 2:
                    if ([round(self.abs_poseX, 1) - 0.5,
                         round(self.abs_poseY, 1)] not in self.poselist and self.range_right > 0.7) or (
                            [round(self.abs_poseX, 1),
                             round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_ahead > 0.7) or (
                            [round(self.abs_poseX, 1) + 0.5,
                             round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7):
                        self.vc_continue = True
                        del self.reverselist[:]
                        break
                    else:
                        if self.reverselist[i][1] > self.reverselist[i + 1][1]:
                            self.go()
                        elif self.reverselist[i][1] < self.reverselist[i + 1][1]:
                            self.botturn(2)
                            self.go()
                        else:
                            if self.reverselist[i][0] > self.reverselist[i + 1][0]:
                                self.botturn(1)
                                self.go()
                            elif self.reverselist[i][0] < self.reverselist[i + 1][0]:
                                self.botturn(0)
                                self.go()

                elif self.state == 3:
                    if ([round(self.abs_poseX, 1), round(self.abs_poseY, 1) + 0.5] not in self.poselist and self.range_left > 0.7) or\
                            ([round(self.abs_poseX, 1) + 0.5, round(self.abs_poseY, 1)] not in self.poselist and self.range_ahead > 0.7) or \
                            ([round(self.abs_poseX, 1), round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_right > 0.7):
                        self.vc_continue = True
                        del self.reverselist[:]

                        break
                    else:
                        if self.reverselist[i][0] < self.reverselist[i + 1][0]:
                            self.go()
                        elif self.reverselist[i][0] > self.reverselist[i + 1][0]:
                            self.botturn(2)
                            self.go()
                        else:
                            if self.reverselist[i][1] > self.reverselist[i + 1][1]:
                                self.botturn(1)
                                self.go()
                            elif self.reverselist[i][1] < self.reverselist[i + 1][1]:
                                self.botturn(0)
                                self.go()

    def north(self):
        if self.vc_continue is True:
            if [round(self.abs_poseX, 1), round(self.abs_poseY, 1) + 0.5] not in self.poselist:
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
                else:  # wall
                    if [round(self.abs_poseX, 1) - 0.5,
                        round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7:
                        if self.range_ahead < 0.7:
                            self.botturn(0)
                            if self.range_ahead > 0.7:
                                self.go()
                                return 0
                    elif [round(self.abs_poseX, 1) + 0.5,
                          round(self.abs_poseY, 1)] not in self.poselist and self.range_right > 0.7:
                        if self.range_ahead < 0.7:
                            self.botturn(1)
                            if self.range_ahead > 0.7:
                                self.go()
                    else:
                        self.vc_continue = False
            elif [round(self.abs_poseX, 1) - 0.5,
                  round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7:
                if self.range_ahead < 0.7:
                    self.botturn(0)
                    if self.range_ahead > 0.7:
                        self.go()
                        return 0
                else:
                    if [round(self.abs_poseX, 1), round(self.abs_poseY, 1) + 0.5] in self.poselist:
                        self.botturn(0)
                        if self.range_ahead > 0.7:
                            self.go()
            elif [round(self.abs_poseX, 1) + 0.5,
                  round(self.abs_poseY, 1)] not in self.poselist and self.range_right > 0.7:
                if self.range_ahead < 0.7:
                    self.botturn(1)
                    if self.range_ahead > 0.7:
                        self.go()
                else:
                    if [round(self.abs_poseX, 1), round(self.abs_poseY, 1) + 0.5] in self.poselist:
                        self.botturn(1)
                        if self.range_ahead > 0.7:
                            self.go()
            else:
                self.vc_continue = False
        else:
            self.trace_back()

    def west(self):
        if self.vc_continue is True:
            if [round(self.abs_poseX, 1),round(self.abs_poseY, 1) + 0.5] not in self.poselist and self.range_right > 0.7:
                self.botturn(1)
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
                if [round(self.abs_poseX, 1) - 0.5, round(self.abs_poseY, 1)] in self.poselist:
                    self.botturn(1)
                    if self.range_ahead > 0.7:
                        self.go()
                        return 0
            elif [round(self.abs_poseX, 1) - 0.5, round(self.abs_poseY, 1)] not in self.poselist and self.range_ahead > 0.7:
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
                else:
                    if [round(self.abs_poseX, 1),
                        round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_left > 0.7:
                        self.botturn(0)
                        if self.range_ahead > 0.7:
                            self.go()
                            return 0
                        else:
                            self.vc_continue = False
            elif [round(self.abs_poseX, 1), round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_left > 0.7:
                self.botturn(0)
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
            else:
                self.vc_continue = False
        else:
            self.trace_back()

    def south(self):
        if self.vc_continue is True:
            if [round(self.abs_poseX, 1) - 0.5,
                round(self.abs_poseY, 1)] not in self.poselist and self.range_right > 0.7:
                self.botturn(1)
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
            elif [round(self.abs_poseX, 1), round(self.abs_poseY, 1) - 0.5] not in self.poselist:
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
                else:
                    if [round(self.abs_poseX, 1) + 0.5,
                        round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7:
                        if self.range_ahead < 0.7:
                            self.botturn(0)
                            if self.range_ahead > 0.7:
                                self.go()
                                return 0
                        else:
                            if [round(self.abs_poseX, 1), round(self.abs_poseY, 1) - 0.5] in self.poselist:
                                self.botturn(0)
                                if self.range_ahead > 0.7:
                                    self.go()
                                    return 0
                    else:
                        self.vc_continue = False
            elif [round(self.abs_poseX, 1) + 0.5,
                  round(self.abs_poseY, 1)] not in self.poselist and self.range_left > 0.7:
                if self.range_ahead < 0.7:
                    self.botturn(0)
                    if self.range_ahead > 0.7:
                        self.go()
                        return 0
                else:
                    if [round(self.abs_poseX, 1), round(self.abs_poseY, 1) - 0.5] in self.poselist:
                        self.botturn(0)
                        if self.range_ahead > 0.7:
                            self.go()
                            return 0
                    else:
                        self.vc_continue = False
            else:
                self.vc_continue = False
        else:
            self.trace_back()

    def east(self):
        if self.vc_continue is True:
            if [round(self.abs_poseX, 1),
                round(self.abs_poseY, 1) + 0.5] not in self.poselist and self.range_left > 0.7:
                self.botturn(0)
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
            elif [round(self.abs_poseX, 1),
                  round(self.abs_poseY, 1) - 0.5] not in self.poselist and self.range_right > 0.7:
                self.botturn(1)
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
            elif [round(self.abs_poseX, 1) + 0.5, round(self.abs_poseY, 1)] not in self.poselist:
                if self.range_ahead > 0.7:
                    self.go()
                    return 0
                else:
                    self.vc_continue = False
            else:
                self.vc_continue = False
        else:
            self.trace_back()

    def vacuum_clean(self):
        if self.outline_search is True:
            self.outline()
        else:
            if self.start is True:  # First Time Turn
                for i in range(10):
                    self.vacuum_on.angular.z = -math.radians(90)
                    self.cmd_vel_pub.publish(self.vacuum_on)
                    self.rate.sleep()
                self.start = False
                self.save_pose()
                self.state = 2
                self.save_state()
            else:
                if self.driving_forward is True:
                    print("state", self.state)
                    if self.state == 0:
                        self.north()
                    elif self.state == 1:
                        self.west()
                    elif self.state == 2:
                        self.south()
                    elif self.state == 3:
                        self.east()

                    if self.range_ahead < 1:
                        self.driving_forward = False
                else:
                    if self.state == 0:
                        self.north()
                    elif self.state == 1:
                        self.west()
                    elif self.state == 2:
                        self.south()
                    elif self.state == 3:
                        self.east()

                    if self.range_ahead > 1:
                        self.driving_forward = True


class Begin:
    def __init__(self):
        self.vr = VacuumRun()

    def get_target(self):
        while not rospy.is_shutdown():
            self.vr.vacuum_clean()


if __name__ == "__main__":
    rospy.init_node('vacuum_module')
    vacuum = Begin()
    vacuum.get_target()
