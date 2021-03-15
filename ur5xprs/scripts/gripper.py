#!/usr/bin/env python
import rospy
import sys
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
from ur5xprs.srv import *
from std_msgs.msg import Bool

def handle_grip_op(req):
	rob = urx.Robot("10.0.0.2")
	robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
	success = Bool(False)
	print "Operation: ",req.operation.data
	if(req.operation.data == "close"):
		# robotiqgrip.close_gripper()
		robotiqgrip.gripper_action(255)
		success = Bool(True)
	elif(req.operation.data == "open"):
		# robotiqgrip.open_gripper()
		robotiqgrip.gripper_action(100)
		success = Bool(True)
	rob.close()
	print "Grip operation is completed"
    
	return gripperResponse(success)
    
def gripper_controller():

	rospy.init_node('gripper', anonymous=True)
   
	print "Node created"
	s = rospy.Service('grip_control',gripper,handle_grip_op)
	rospy.spin()

if __name__ == '__main__':
		
	gripper_controller()
	
