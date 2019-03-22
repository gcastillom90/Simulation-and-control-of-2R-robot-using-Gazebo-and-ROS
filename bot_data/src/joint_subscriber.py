#! /usr/bin/env python

import rospy                                          
from control_msgs.msg import JointControllerState 

def callback1(data):                                          # Define a function called 'callback1' that receives a parameter named 'data'
    print " Joint 1 position  = %.2f" %(data.process_value)   # Print the variable 'process_value' inside the 'data' parameter
    j1.write("%.2f\r\n" %(data.process_value))		      # Save the value of the joint1 angle in the corresponding text file	

def callback2(data):                                          # Define a function called 'callback1' that receives a parameter named 'data'
    print " Joint 2 position  = %.2f" %(data.process_value)   # Print the variable 'process_value' inside the 'data' parameter
    j2.write("%.2f\r\n" %(data.process_value))		      # Save the value of the joint2 angle in the corresponding text file	

def listener():
	rospy.init_node('joints1_2_subscriber')               # Initiate a Node called 'topic_subscriber'
	sub = rospy.Subscriber('/bot/joint1_position_controller/state', JointControllerState, callback1)  # Create a Subscriber object that 								will listen to the "/bot/joint1_position_controller/state" topic and will call the 								'callback1' function each time it reads something from the topic
	sub = rospy.Subscriber('/bot/joint2_position_controller/state', JointControllerState, callback2)  
	rospy.spin()                                          # Create a loop that will keep the program in execution

if __name__ == '__main__':
	j1 = open("joint1_data.txt","w+")		      #Create text files to save the data of joint angles
	j2 = open("joint2_data.txt","w+")
	listener()
