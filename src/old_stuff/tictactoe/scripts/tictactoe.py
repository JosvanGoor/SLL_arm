#!/usr/bin/env python
import roslib
import rospy
import cv2
import numpy as np
import cjson
import sys
import random
from collections import Counter

#Video
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class tictactoe():
	def __init__(self):
		print "Created class"
		self.bridge = CvBridge()
		#rospy.Subscriber("camera2/depth/image_raw", Image, self.newImageCB)
		self.__board = np.zeros((3,3))
		self.__board[2,0] = 0
		self.__board[1,1] = 0
		self.__board[0,2] = 0		

	#Aanropen na elke Callback van image_topic
	def update(self):
		if self.yourTurn():
			print "Sudo is aan de beurt"
			x,y  = self.getPosition()
			self.__board[x,y] = 2
			if self.checkWon():
				print "Sudo wint"
				return True
		else:
			print "Tegenstander is aan de beurt"
			#Straks verwijderen.
			x,y  = self.getPosition()
			self.__board[x,y] = 1
			if self.checkWon():
				print "Tegenstander wint. Candy!"
				return False

		if self.countNumber(0) == 0 and not self.checkWon():
			print "Speelbord vol"
			return False
		
	def boardEmpty(self):
		return True if (self.__board.sum()) == 0 else False

	def checkWon(self):
		for i in range(0,3):
			#Check Row
			if (self.__board[i,0] == self.__board[i,1]) and (self.__board[i,1] == self.__board[i,2]) and not self.__board[i,0] == 0:
				return True
			#Check Column
			if (self.__board[0,i] == self.__board[1,i]) and (self.__board[1,i] == self.__board[2,i]) and not self.__board[0,i] == 0:
				return True
		if (self.__board[0,0] == self.__board[1,1]) and (self.__board[1,1] == self.__board[2,2]) and not self.__board[0,0] == 0:
			return True
		if (self.__board[0,2] == self.__board[1,1]) and (self.__board[1,1] == self.__board[2,0]) and not self.__board[0,2] == 0:
			return True

		return False

	def countNumber(self, nr):
		c = c = Counter([i for j in self.__board for i in j])
		return c[nr]

	def yourTurn(self):
		return True if (self.countNumber(1) > self.countNumber(2)) else False

	def cellFree(self, x,y):
		if self.__board[x,y] == 0:
			return True
		return False

	def getPosition(self):
		x = random.randint(0,2)
		y = random.randint(0,2)
		while not self.cellFree(x,y):
			x = random.randint(0,2)
			y = random.randint(0,2)
		
		return x,y

if __name__ == '__main__':
	rospy.init_node('TicTacToe')
	TicTacToe = tictactoe()

	try:
		TicTacToe.update() #Test code
		rospy.spin()
		
	except KeyboardInterrupt:
		print "Shutting down"
	
	

