#!/usr/bin/env python

import sys, rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from demo_world.msg import Tracker

class elfin_vision:
	
	def __init__(self, red, green, blue):
		self.tracker = Tracker()
		self.track_flag = False
		self.default_pose_flag = True
		self.cx = 960
		self.cy = 540
		self.bridge = cv_bridge.CvBridge()
		self.image_sub = rospy.Subscriber('/realsense_plugin/color/image_raw', Image, self.image_callback)
		self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)
		self.mask = False
		self.hsv = False
		self.red = red
		self.green = green
		self.blue = blue
		


	def image_callback(self, msg):
		#print("Getting a msg")
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		self.track_flag = False
		self.hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
		# !!!BGR!!!
		self.setMask(self.blue,self.green,self.red)

		(_,cnts, _) = cv2.findContours(self.mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		h,w,d = image.shape

		# Finder 
		M = cv2.moments(self.mask)

		if M['m00']>0:
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])

		# end finder
			for i, c in enumerate(cnts):
				area = cv2.contourArea(c)
				if area > 7500:
					self.track_flag = True
					self.cx = cx
					self.cy = cy
					self.error_x = self.cx - w/2
					self.error_y = self.cy - (h/2)
					self.tracker.x = cx
					self.tracker.y = cy
					self.tracker.flag1 = self.track_flag
					self.tracker.error_x = self.error_x
					self.tracker.error_y = self.error_y
					self.tracker.red = self.red
					self.tracker.green = self.green
					self.tracker.blue = self.blue

					cv2.circle(image,(cx,cy), 10, (0,0,0), -1)
					cv2.putText(image, "({}, {})".format(int(cx),int(cy)), (int(cx-20),int(cy+25)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
					cv2.drawContours(image, cnts, -1, (255,255,255),1)
					break
		else:
			self.track_flag = False
			self.tracker.flag1 = False
		
		self.cxy_pub.publish(self.tracker)

		cv2.circle(image,(960,540), 10, (0,0,0), -1)
		cv2.namedWindow("Camera view", 1)
		cv2.imshow("Camera view", image)
		cv2.waitKey(1)

	
	def setMask(self, blue, green, red):
		"""Set the mask to be used."""
		self.red = red
		self.green = green
		self.blue = blue

		color = np.uint8([[[blue,green,red]]])
		hsv_color = cv2.cvtColor(color,cv2.COLOR_BGR2HSV)
		h = hsv_color[0][0][0]
		if h > 9:
			lower = np.array([h-10, 100, 100])
		else:
			lower = np.array([0,100,100])
		
		if h<245:
			upper = np.array([h+10,255,255])
		else:
			upper = np.array(255,255,255)
		#print(lower)
		self.mask = cv2.inRange(self.hsv, lower, upper)


# if __name__ == "__main__":
# 	rospy.init_node('elfin_vision', anonymous=False)
# 	ev = elfin_vision(255,0,0)
# 	rospy.spin()