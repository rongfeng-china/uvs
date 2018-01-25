#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_image')
import sys, rospy, cv2, os, rosgraph.masterapi
import numpy as np
import numpy as np
import math
from transformations import euler_from_quaternion
from Quaternion import Quat
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage,RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from image_publisher import image_publisher
from wam import WAM
import logging
import select
import time
import scipy.misc
cv2.namedWindow("Display",1)
dL,dx,dy,dz,dax,day,daz = 100.,100.,100.,100., 100.,100.,100.
angles_current,angles_previous = np.array([100.,100.,100.]),  np.array([100.,100.,100.])
joint_current, joint_previous= np.array([0.,0.,0.,0.,0.,0.,0.]),np.array([0.,0.,0.,0.,0.,0.,0.])
counter = 0

def display(robot,cam):
    while not rospy.is_shutdown():
        (cv_image,imgstamp) = cam.get_img()
          
        if cv_image == None:
            continue
        print 'cartesian pose' 
        print robot.last_tool_pose
        print 'joint pose'
        print robot.last_joint_pose
        if not (cv_image == None):
            cv2.imshow("Display", cv_image)
            cv2.waitKey(5)    
        else:
            print 'None'

def init_move(robot,cam):
    global dL, dx, dy, dz, dax, day, daz
    global angles_current, angles_previous
    global L2,x2,y2,z2,qtx2,qty2,qtz2,qtw2
    global joint_current, joint_previous
    joint_current = np.array(list(robot.last_joint_pose))
    L = 0.3 # rong: read from image
    x,y,z = robot.last_tool_pose.position.x, robot.last_tool_pose.position.y,robot.last_tool_pose.position.z
    qtx,qty,qtz,qtw = robot.last_tool_pose.orientation.x,robot.last_tool_pose.orientation.y,robot.last_tool_pose.orientation.z,robot.last_tool_pose.orientation.w
    anglex,angley,anglez = euler_from_quaternion([qtw,-qtx,-qty,-qtz])
    joint_previous = joint_current
    joint_current = joint_previous+np.array([.002,.002,.002,.002,.002,.002,.002])
    #print new_joint
    robot.joint_move(joint_current)
    # after a shortmovement, read cartesian values
    L2 = L - 0.005  # rong: read from image
    x2,y2,z2 = robot.last_tool_pose.position.x, robot.last_tool_pose.position.y,robot.last_tool_pose.position.z
    qtx2,qty2,qtz2,qtw2 = robot.last_tool_pose.orientation.x,robot.last_tool_pose.orientation.y,robot.last_tool_pose.orientation.z,robot.last_tool_pose.orientation.w
    anglex2,angley2,anglez2 = euler_from_quaternion([qtw2,-qtx2,-qty2,-qtz2])
    dL = L2-L
    dx,dy,dz,dax,day,daz =  x2-x,y2-y, z2-z,anglex2-anglex,angley2-angley,anglez2-anglez
    angles_current = np.array([anglex2, angley2, anglez2])
    print '##### initial move done ########'
    print dL,dx,dy,dz,dax,day,daz

def main_process(robot,cam):
    global counter
    global angles_current, angles_previous
    global L,x,y,z,qtx,qty,qtz,qtw
    global L2,x2,y2,z2,qtx2,qty2,qtz2,qtw2
    global joint_current, joint_previous
    print 'counter'
    print counter
    while counter < 2:
        print 'update values'
        # upcate cartesian, joint values , angles
        angles_previous = angles_current
        joint_previous = joint_current
        L,x,y,z,qtx,qty,qtz,qtw = L2,x2,y2,z2,qtx2,qty2,qtz2,qtw2 
        # read new data: cartesian pose(x2,y2,z2,qtx2,qty2,qtz2,qtw2), pixel L2, joint_current
        print 'read'
        L2 = L - .005 ## rong: read from image
        qtx,qty,qtz,qtw = robot.last_tool_pose.orientation.x,robot.last_tool_pose.orientation.y,robot.last_tool_pose.orientation.z,robot.last_tool_pose.orientation.w        
        angles_current = np.array(euler_from_quaternion([qtw,-qtx,-qty,-qtz]))
        joint_current = np.array(list(robot.last_joint_pose))
        x2,y2,z2 =robot.last_tool_pose.position.x,robot.last_tool_pose.position.y,robot.last_tool_pose.position.z
      
        print 'calcu'
        dL,dx,dy,dz = L2-L, x2-x,y2-y, z2-z
        dax,day,daz = angles_current - angles_previous
        dq2,dq3,dq4,dq5,dq6,dq7 = .002,.002,.002,.002,.002,.002
        dq = np.array([dq2,dq3,dq4,dq5,dq6,dq7])
        dE = np.array([dL,dx,dy,dax,day,daz])
        J_k_transpose = np.array([dE/dq2,dE/dq3,dE/dq4,dE/dq5,dE/dq6,dE/dq7])
        J_k = np.transpose(J_k_transpose)

        mid_step = np.transpose(dE)-np.dot(J_k,np.transpose(dq))
        mid_step2 = np.dot(dq,np.transpose(dq))
        J_k_update = J_k + 0.1*np.dot(mid_step,dq)/mid_step2  # alpha = .1
        print 'calcu2'
        print J_k_update
        try:
            joint_update = -.1*np.dot(np.linalg.inv(J_k_update),np.transpose(dE)) # lambda = .1
            print 'joint_updadate'
            print joint_update
        except:
            print 'inverse not work'
            pass

        

        counter += 1


def GetChar(Block=False):
    if Block or select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.read(1)
    return None

def main(args):
    controls = {"init_move" : 'i',
                "main_process" : 'p',
                "terminate": 'x',
                "display": 'd' }
    rospy.init_node('read_pose', anonymous=True)
    for k,v in controls.items():
        rospy.loginfo("to {} press {}".format(k,v))


    traj_root = rospy.get_param('~output_root',
                os.path.join(os.path.expanduser("~"),'task_back'))
    count_records = 0

    try:
        robot = WAM()
        cam = image_publisher()
    except:
        print "error at start"

    while True:
        command = GetChar()     
        if command=='x':
            cv2.destroyAllWindows()
            rospy.loginfo("shutting down the read_pose node")
            break
        if command == 'd':
            display(robot,cam)
        if command == 'i':
            init_move(robot,cam)
        if command == 'p':
            main_process(robot,cam)

if __name__ == '__main__':
    main(sys.argv)

