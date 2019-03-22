#!/usr/bin/env python
import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class QuadJointMover(object):
    def __init__(self):
        # LEFT SIDE
        self.pub_legLB1_position = rospy.Publisher(
            '/rrbot/legLB1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLB2_position = rospy.Publisher(
            '/rrbot/legLB2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLB3_position = rospy.Publisher(
            '/rrbot/legLB3_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF1_position = rospy.Publisher(
            '/rrbot/legLF1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF2_position = rospy.Publisher(
            '/rrbot/legLF2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legLF3_position = rospy.Publisher(
            '/rrbot/legLF3_position_controller/command',
            Float64,
            queue_size=1)

        # RIGHT SIDE
        self.pub_legRB1_position = rospy.Publisher(
            '/rrbot/legRB1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRB2_position = rospy.Publisher(
            '/rrbot/legRB2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRB3_position = rospy.Publisher(
            '/rrbot/legRB3_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF1_position = rospy.Publisher(
            '/rrbot/legRF1_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF2_position = rospy.Publisher(
            '/rrbot/legRF2_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_legRF3_position = rospy.Publisher(
            '/rrbot/legRF3_position_controller/command',
            Float64,
            queue_size=1)

        
    def publish_pose(self, pose):

        legLB1 = Float64()
        legLB1.data = pose["legLB1"]
        legLB2 = Float64()
        legLB2.data = pose["legLB2"]
        legLB3 = Float64()
        legLB3.data = -pose["legLB3"]

        legLF1 = Float64()
        legLF1.data = pose["legLF1"]
        legLF2 = Float64()
        legLF2.data = pose["legLF2"]
        legLF3 = Float64()
        legLF3.data = -pose["legLF3"]

        legRB1 = Float64()
        legRB1.data = pose["legRB1"]
        legRB2 = Float64()
        legRB2.data = pose["legRB2"]
        legRB3 = Float64()
        legRB3.data = -pose["legRB3"]

        legRF1 = Float64()
        legRF1.data = pose["legRF1"]
        legRF2 = Float64()
        legRF2.data = pose["legRF2"]
        legRF3 = Float64()
        legRF3.data = -pose["legRF3"]

        self.pub_legLB1_position.publish(legLB1)
        self.pub_legLB2_position.publish(legLB2)
        self.pub_legLB3_position.publish(legLB3)
        self.pub_legLF1_position.publish(legLF1)
        self.pub_legLF2_position.publish(legLF2)
        self.pub_legLF3_position.publish(legLF3)

        # RIGHT SIDE
        self.pub_legRB1_position.publish(legRB1)
        self.pub_legRB2_position.publish(legRB2)
        self.pub_legRB3_position.publish(legRB3)
        self.pub_legRF1_position.publish(legRF1)
        self.pub_legRF2_position.publish(legRF2)
        self.pub_legRF3_position.publish(legRF3)
        
        #rospy.loginfo("Pose sent")

'''
def getHardCodedDemoPoses():
    pose1 = {'mouth_joint': 0.09, 'tailPan': -1.04, 
                'L_ear_joint': 0.06, 'R_ear_tilt': -0.23, 
                'tailTilt': 0.05, 'headPan': 0, 'headTilt': 0, 
                'R_ear_joint': 0.34, 'L_ear_tilt': -0.28, 'neck_joint': 0,
                'legRF2': 0, 'legRF3': -0.1, 'legRF1': 0.7, 'legLF1': 0.7, 'legLF2': 0, 
                'legLF3': 0.1, 'legLB1': 0.4, 'legLB2': 0, 'legLB3': 0.1, 'legRB2': 0, 'legRB3': 1.5, 'legRB1': -0.3}
    
    pose2 = {"legRF1": -0.3, "legRF2": 0, "legRF3": 0.7,
              "legLF1": -0.3, "legLF2": 0, "legLF3": 0.7,
              "legRB1": -0.3, "legRB2": 0, "legRB3": 0.1,
              "legLB1": -0.3, "legLB2": 0, "legLB3": 0.1,
              "headPan": 0, "headTilt": 0, "neck_joint": 0
              }
    
    return pose1, pose2
'''

def readMotionFromFile (file):
    
    f = open(file,'r')
    nframes = int(f.readline())
    rate = int(f.readline())
    njoints = int(f.readline())
    
    posesList = []
    for x in range(0, nframes):
        frameNumber = f.readline()
        joints=[]
        values=[]
        for y in range(0, njoints):
            joint =f.readline()
            joint = joint.rstrip('\n')
            joints.append(joint)
            
            value = f.readline()
            value = value.rstrip('\n')
            values.append(float(value))
        posesList.append(dict(zip(joints, values)))
    
    #print posesList
    return posesList, rate, nframes   

if __name__ == "__main__":
    
    rospy.init_node('quad_joint_control', anonymous=True)
    rospy.loginfo("quad_joint_control Initializing...")
        
    quad_jointmover_object = QuadJointMover()
 
    #pose = getHardCodedDemoPoses()
    
    pose, rate, nframes = readMotionFromFile ('/home/guillermo/catkin_ws/src/gazebo_ros_sim/motion/motion_file_final')
    
    print "Number of frames = " + str(nframes)
    
    print "Rate = " + str(rate)
    
    rospy.sleep(1)
    r = rospy.Rate(rate)
    frame = 0
    
    while not rospy.is_shutdown():
        #print "current frame=" + str(frame)
        quad_jointmover_object.publish_pose(pose[frame])
        frame = frame + 1
        if frame == nframes:
            frame = 0
        r.sleep()
