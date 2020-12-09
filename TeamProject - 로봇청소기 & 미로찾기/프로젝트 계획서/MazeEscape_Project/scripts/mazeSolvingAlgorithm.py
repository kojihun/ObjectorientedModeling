#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
import math, time
from deu_maze.msg import LidarMeasure
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Data:
	def __init__(self):
		self.data = list()

	def empty(self):
		if not self.data:
			return True
		else:
			return False

	def dataLength(self):
		return len(self.data)

	def push(self, tmpData):
		self.data.append(tmpData)

	def top(self):
		return self.data[len(self.data)-1]

class Start: # 미로 탈출 시작
	def __init__(self):
		self.mazesolve = MazeSolve()
		self.retracemaze = RetraceMaze()

	def solve_start(self):
		print("**** DEU_MAZE 미로 탈출 시스템 ****")
		print("미로 탈출을 시작합니다:) ")
		while not rospy.is_shutdown():
			if (-8.55 < self.mazesolve.map_x < -7.55) and (5.05 < self.mazesolve.map_y < 5.55) and self.mazesolve.arrive_endPoint is True:
				print("미로를 탈출하셨습니다:) 프로그램을 종료합니다.")
				break

			if ((7.75 < self.mazesolve.map_x < 8.15) and (-5.35 < self.mazesolve.map_y < -4.90)) or self.mazesolve.arrive_endPoint is True:
				self.mazesolve.arrive_endPoint = True
				#print(self.mazesolve.arrive_endPoint)
				self.retracemaze.retrace()
			else:
				self.mazesolve.escape()
				self.mazesolve.saveData()


class MazeSolve:
	stackData = Data()
	crossroadData = Data()

	def __init__(self):

		self.driving_forward = True # 주행 상태 확인 
		self.check_rotate = False # 회전 여부 확인
		self.enter_maze = True # 터틀봇 미로 첫 진입 여부 확인
		self.arrive_endPoint = False # 도착 여부 확인
		self.way_count = 1 # 통로 갯수
		self.speed = 1.0 # 로봇 속도
		self.rotate = Twist()
		self.rate = rospy.Rate(10)

		# 터틀봇 좌표(odom_data)
		self.turtlebot_x = 0
		self.turtlebot_y = 0

		# 맵 봐표(odom_data)
		self.map_x = 0
		self.map_y = 0

		# 터틀봇 방향
		self.absolute_direction = ['right', 'down', 'left', 'up']
		self.turtlebot_direction = 1 # 터틀봇 초기 방향 ('down')
		self.rotate_direction = 0 # 0은 반시계, 1은 시계 방향

		self.lidar_sub = rospy.Subscriber('lidarMeasurement', LidarMeasure, self.lidar_callback) # Lidar topic 구독
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1) # cmd_vel 발행
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback) # odom 구독 


	def lidar_callback(self, floatmsg):
		self.range_ahead = floatmsg.range_ahead
		self.range_left = floatmsg.range_left
		self.range_right = floatmsg.range_right
		self.range_rear = floatmsg.range_rear

		lidarDataList = [self.range_right, self.range_rear, self.range_left, self.range_ahead]
		self.changeWay(lidarDataList)

	def odom_callback(self, msg):
		self.turtlebot_x = msg.pose.pose.position.x
		self.turtlebot_y = msg.pose.pose.position.y
		self.map_x = self.turtlebot_x - 7.92
		self.map_y = self.turtlebot_y + 5.30

	def saveData(self):
		if self.check_rotate:
			tmpData = [self.absolute_direction[self.turtlebot_direction], round(self.turtlebot_x, 3), round(self.turtlebot_y, 3)]
			# [ 절대방향, 터틀봇 x 위치, 터틀봇 y 위치 ]

			if not self.stackData.empty():
				top = self.stackData.top()
				if (tmpData[1] == top[1]) and (tmpData[2] == top[2]): # 동일한 로봇의 위치가 들어오면
					self.stackData.data.pop()
				else:
					self.stackData.push(tmpData)
			else:
				self.stackData.push(tmpData)

			if self.way_count > 2:
				tmpData.append(self.way_count)
				self.crossroadData.push(tmpData)

			"""
			데이터를 확인하고 싶으면, 주석을 삭제 하시면 됩니다:)
			print("저장된 현재 위치 : {}" .format(tmpData))
			print("저장 된 스택 : {}" .format(self.stackData.data))
			print("갈림길 : {}" .format(self.crossroadData.data))
			print("")
			"""

	def changeWay(self, lidarDataList):
		# 길 탐지
		self.wayList = [False, False, False, False]  # [up_way, left_way, down_way, right_way] 초기화

		count = 0
		tmpIndex = self.turtlebot_direction
		for i in range(4):
			if tmpIndex > 3:
				tmpIndex = 0
			#print(tmpIndex)
			if lidarDataList[tmpIndex] > 1.0:
				self.wayList[i] = True
				count +=1
			tmpIndex +=1

		self.way_count = count


	def escape(self):
		if self.enter_maze is True:
			for i in range(13):
				start_maze = Twist()
				start_maze.angular.z = -math.radians(90)
				self.cmd_vel_pub.publish(start_maze)
				self.rate.sleep()
			self.enter_maze = False
			self.start_time = time.time()

		else:
			twistData = Twist()
			self.distance = abs(self.start_time - time.time()) * self.speed # 분기점 거리 계산

			if (-8.25 < self.map_x < -7.34) and ( self.map_y > 4.38): # 미로 진입 하가기전
				twistData.linear.x = self.speed
				self.cmd_vel_pub.publish(twistData)
				self.rate.sleep()
				self.driving_forward = True

			else:
				if self.check_rotate is True:
					if self.rotate_direction is 0:  # 반시계 방향으로 절대 방향 전환
						if (self.turtlebot_direction > 0):
							self.turtlebot_direction -= 1
						else:
							self.turtlebot_direction = 3

					elif self.rotate_direction is 1:  # 시계 방향으로 절대 방향 전환
						if (self.turtlebot_direction < 3):
							self.turtlebot_direction += 1
						else:
							self.turtlebot_direction = 0

					for i in range(10):
						self.cmd_vel_pub.publish(self.rotate)
						self.rate.sleep()
					self.check_rotate = False
					self.start_time = time.time()

				else:
					if self.driving_forward is True:
						if self.distance >= 1.7:
							if self.way_count > 2:
								self.driving_forward = False
							else:
								self.start_time = time.time() # 분기점 시작 시간 재설정
								if self.range_ahead < 0.75:
									self.driving_forward = False

					else:
						if self.range_ahead >= 0.75:
							self.driving_forward = True

					if self.driving_forward is True:
						twistData.linear.x = self.speed

						if self.range_right >= 0.75:
							self.rotate.angular.z = math.radians(90)
							self.rotate_direction = 0 # 반시계 방향 회전

						elif self.range_left >= 0.75:
							self.rotate.angular.z = -math.radians(90)
							self.rotate_direction = 1 # 시계 방향 회전
					else:
						self.check_rotate = True

					self.cmd_vel_pub.publish(twistData)
					self.rate.sleep()

class RetraceMaze:
	def __init__(self):
		self.m_solve =  MazeSolve()

		self.retrace_maze = True # 미로 재진입 여부 (도착지)

		self.re_driving_forward = True  # 주행 상태 확인
		self.re_check_roate = False # 회전 여부 확인

		self.rate = rospy.Rate(10)
		self.pathData = self.m_solve.stackData.data # 저장된 최단 경로

		self.turtlebot_current_direction = 3  # 터틀봇 초기 방향 ('up')

		self.lidar_sub = rospy.Subscriber('lidarMeasurement', LidarMeasure, self.m_solve.lidar_callback)  # Lidar topic 구독
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)  # cmd_vel 발행
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.m_solve.odom_callback)  # odom 구독

	def retrace(self):
		if self.retrace_maze is True:
			for i in range(21):
				start_maze = Twist()
				start_maze.angular.z = -math.radians(90) # 7.35 = math.radians(90)
				self.cmd_vel_pub.publish(start_maze)
				self.rate.sleep()
			self.retrace_maze = False
		else:
			twistData = Twist()
			"""
			데이터를 확인하고 싶으면, 주석을 삭제 하시면 됩니다:)
			print("저장 된 스택 : {}".format(self.pathData))
			print("")
			"""
			if len(self.pathData) is not 0:
				if (self.pathData[len(self.pathData)-1][1] - 0.25 <= self.m_solve.turtlebot_x <= self.pathData[len(self.pathData)-1][1] + 0.25) and (self.pathData[len(self.pathData)-1][2] - 0.25 <= self.m_solve.turtlebot_y <= self.pathData[len(self.pathData)-1][2] + 0.25):
					self.re_driving_forward = False  # 주행 정지                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            riving_forward = False # 주행 정지
					self.re_check_roate = True # 회전 수행

					self.turtlebot_direction = self.m_solve.absolute_direction.index(self.pathData[len(self.pathData)-1][0]) # 방향 설정
					self.turtlebot_direction = (self.turtlebot_direction + 2) % 4 # 현재 방향 반전

					if self.re_check_roate is True:
						if self.re_driving_forward is False:
							turtle_current_left = self.turtlebot_current_direction - 1 if (self.turtlebot_current_direction > 0) else 3
							turtle_current_right = self.turtlebot_current_direction + 1 if (self.turtlebot_current_direction < 3) else 0

							rotateData = Twist()
							if self.turtlebot_direction == turtle_current_left: # 현재 방향의 반시계 방향과 동일한 값일 때
								rotateData.angular.z = math.radians(90)
								self.turtlebot_current_direction = turtle_current_left

							elif self.turtlebot_direction == turtle_current_right: # 현재 방향의 시계 방향과 동일한 값일 때
								rotateData.angular.z = math.radians(-90)
								self.turtlebot_current_direction = turtle_current_right

							for i in range(10):
								self.cmd_vel_pub.publish(rotateData)
								self.rate.sleep()

							self.pathData.pop()
							self.re_check_roate = False
							self.re_driving_forward = True

				else:
					if self.re_check_roate is False:
						if self.re_driving_forward is True:
							twistData.linear.x = self.m_solve.speed

				self.cmd_vel_pub.publish(twistData)
				self.rate.sleep()

			else:
				twistData.linear.x = self.m_solve.speed
				self.cmd_vel_pub.publish(twistData)
				self.rate.sleep()

if __name__ == "__main__":
	rospy.init_node('maze_algorithm')
	maze = Start()
	maze.solve_start()


	
