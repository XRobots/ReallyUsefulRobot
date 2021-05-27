#detects the object
#gets the distance
#moves the eye until its in the middle
#receives and prints the angle in the main loop
#calculates the trig for the vert and horiz position of the detection
#receives mode from touchscreen remote
#allows mode switching
#publishes 2 different sets of vector 3 messages to move the arm, depending on what mode we're in


import jetson.inference
import jetson.utils
import pyrealsense2 as rs
import rospy
import threading
import time
import math

from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

rospy.init_node('vision')
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
pubCoords = rospy.Publisher('armCoords', Vector3, queue_size=10)
msg = Twist()
msgMode = UInt16()
msgCoords = Vector3()

net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = jetson.utils.gstCamera(640,480, "/dev/video2")
display = jetson.utils.glDisplay()

global mode
mode = 1							# default mode

def func1():							# thread to receive ROS message for eye angle			
	def eye_callback(msg):
		global angle
		angle = (msg.data)
		#print("angle = ", angle)
	def remote_callback(msgMode):				# also receives mode from touchscren remote			
		global mode
		mode = (msgMode.data)
		#print("mode = ", mode)
	while True:
		rospy.Subscriber('eye', UInt16 , eye_callback)
		rospy.Subscriber('touchscreen', UInt16 , remote_callback)
		rospy.spin()

thread1 = threading.Thread(target=func1)			# start thread etc
thread1.daemon = True
thread1.start()	

try:
	# Create a context object. This object owns the handles to all connected realsense devices
	pipeline = rs.pipeline()

	# Configure streams
	config = rs.config()
	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

	# Start streaming
	pipeline.start(config)


	while display.IsOpen():				

		frames = pipeline.wait_for_frames()					# open pipeline for DEPTH data
		depth = frames.get_depth_frame()					# get DEPTH frames

		img, width, height = camera.CaptureRGBA()
		detections = net.Detect(img, width, height)
		display.RenderOnce(img, width, height)
		display.SetTitle("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
		
		#print("detected {:d} objects in image".format(len(detections)))	# print detections
		for detection in detections:
			#print(len(detections))
			n = len(detections)			
			for i in range(0, n):
				#print (i)
				#print ("object detected")
				class_idx = detections[i].ClassID

				if mode == 1:						# looking mode - adjust eye for center

					pubCoords = rospy.Publisher('armCoords',Vector3)
					msgCoords.x = 350
					msgCoords.y = 180
					msgCoords.z = 0
					pubCoords.publish(msgCoords)				# publish default arm position - out of the way

					if class_idx == 47:					# look only for cups
						print (net.GetClassDesc(class_idx))		# print name of object detected, 47 is cup
						detectX = (detections[i].Center[0])
						detectY = (detections[i].Center[1])
						print ("X =", detectX)
						print ("Y =", detectY)	
						detectXa = int((detectX/640*400)+100)		# scale and shift for RGB/depth camera alignment
						detectYa = int((detectY/480*240)+100)		# scale and shift for RGB/depth camera alignment
						#print ("Xa =", detectXa)
						#print ("Ya =", detectYa)
						dist = depth.get_distance(detectXa,detectYa)	# get depth for object
						print("Z =", dist) 				# print distance to object
						if detectY < 230:
							print("move up")
							vel = -0.05
						elif detectY > 250:
							print("move down")
							vel = 0.05
						else:
							print("middle")
							vel = 0
						print("angle Degrees = ", angle)		# print angle from ROS/eye thread aove
						angleRads = angle * (math.pi/180)
						print("angle Radians = ", angleRads)		# print angle in Radians
						vertical = math.cos(angleRads) * dist-0.02	# do trig, minus sensor distance to cente of rotation
						horizontal = math.sin(angleRads) * dist-0.02
						print("vertical =", vertical)
						print("horizontal =", horizontal)
						print("mode =", mode)				# print mode from touchscreen
						print()						# blank line

						pub = rospy.Publisher('cmd_vel',Twist)
						msg.linear.y = vel
						pub.publish(msg)				# publish ROS message to move eye


				elif mode == 2:						# mode 2 freezes all the variables and operates the arm
					#print("grabbing mode", mode)
					pubCoords = rospy.Publisher('armCoords',Vector3)
					xPos = (horizontal*1000) + 300				# scale and remove offset from back of robot
					msgCoords.x = xPos
					yPos = ((detectX-320)*0.6)+50				# scale and remove offset					
					msgCoords.y = yPos
					zPos = ((vertical*1000)-200)*-95			# scale for linear Z axis and remove offset from camera to arm
					msgCoords.z = zPos
					pubCoords.publish(msgCoords)
					print("xPos = ", xPos)
					print("zPos = ", zPos)
					print("yPos = ", yPos)			


	exit(0)
#except rs.error as e:
#    # Method calls agaisnt librealsense objects may throw exceptions of type pylibrs.error
#    print("pylibrs.error was thrown when calling %s(%s):\n", % (e.get_failed_function(), e.get_failed_args()))
#    print("    %s\n", e.what())
#    exit(1)
except Exception as e:
	print(e)
	pass





