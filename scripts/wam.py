#!/usr/bin/env python
import rospy
import decimal
import numpy

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from wam_srvs.srv import Hold
from wam_srvs.srv import JointMove
from wam_srvs.srv import CartPosMove
from wam_srvs.srv import OrtnMove
from wam_srvs.srv import OrtnSplitMove
import pdb
from wam_planning.srv import IKPose

class WAM(object):
    """ Wrapper for the WAM services and messages
    """

    def tool_pose_cb(self, data):
        """ Callback function for the /wam/pose topic.
            The last tool pose is saved in self.last_tool_pose
            This is an object with two members position and orientationi where
            the orientation is a quaternion:

            eg.
            xyz_position = [wam.last_tool_pose.position.x,
                            wam.last_tool_pose.position.y,
                            wam.last_tool_pose.position.z
                           ]
            pose_quaternion = [wam.last_tool_pose.orientatoin.x,
                               wam.last_tool_pose.orientatoin.y,
                               wam.last_tool_pose.orientatoin.z,
                               wam.last_tool_pose.orientatoin.w
                              ]

            NOTE:
                Do not call manually
        """
        self.last_tool_pose = data.pose

    def joint_pose_cb(self, data):
        """ Callback function for the /wam/joint_states topic.

            last_joint_pose is a 4-tuple with radian angles of each joint

            NOTE:
                Do not call manually
        """
        self.last_joint_pose = data.position

    def go_home(self):
        """ Sends the robot safely to it's home position
        """
        self.go_home_srv()

    def joint_move(self, joints):
        """ Given a list of 4 angles in rads it will command the robot to move
            to the given joint angles.

            eg. joint_move([0,0,0,1.57])

            Args:
                joints (List[double]): Joint angles in radians for the robot to
                                      move to
        """
        #if len(joints) != 4:
        #    rospy.logwarn("Invalid joint move target: " + str(joints))
        #    return
        self.joint_move_srv.call(joints)

    def hold(self, hold):
        """ Allows for the robot to be set in a hold state or in gravity
            compensation. When in hold, the robot can only be moved through
            commands. When in gravity compensation you can manually move the
            arm around.

            Args:
                hold (bool): Whether the robot will be in hold or not
        """
        self.joint_hold_srv.call(hold)

    def pose_move(self, pose):
        self.pose_move_srv.call(pose)
        rospy.loginfo(pose)

    def ori_move(self,ori):
        #pdb.set_trace()
        self.ori_move_srv.call(ori)
        rospy.loginfo(ori)

    def ori_split_move(self,ori,kp,kd):
   	#pdb.set_trace()
        self.ori_split_move_srv.call(ori,kp,kd)
        rospy.loginfo(ori)

    def hold_ori(self, hold):
        self.hold_ori_srv.call(hold)
        rospy.loginfo(hold)

    def hold_cart(self, hold):
        self.cart_hold_srv.call(hold)
        rospy.loginfo(hold)

    def go_init_pose(self):
        self.joint_move([0,0,0,1.57,0,0,0])



    def move_to_pose(self, pose):
        '''pose2 = Pose( 
                  position = Point(x = 0.08,y = .1 ,z =.8 ),
                  orientation = Quaternion( x =0 ,y =0.707 ,z= 0 , w= 0.707)
                  )

        self.move_to_pose_srv.call(pose2)'''
        # This service is written by Camilo, the sign of quaternion (x,y,z) should be changed. 
        print str(decimal.Decimal(pose[0])) 
        a = 0.1
        '''pose2 = Pose( 
                  position = Point(x = pose[0],y = pose[1] ,z =pose[2] ),
                  orientation = Quaternion( x =-pose[3] ,y =-pose[4] ,z= -pose[5] , w= pose[6])
                  )'''
	pose2=Pose()
	pose2.position.x=pose[0] #0.40515,-0.10303,0.45994
	pose2.position.y=pose[1]
	pose2.position.z=pose[2]	
	pose2.orientation.x=-pose[3] #-9.1450000e-02,-7.5910000e-01,8.8820000e-02,6.3837000e-01
	pose2.orientation.y=-pose[4]
	pose2.orientation.z=-pose[5]
        pose2.orientation.w=pose[6]

        self.move_to_pose_srv.call(pose2)
        rospy.loginfo(pose2)

    def __init__(self):
        """ Creates the WAM object and subscribes to the approriate services and topics
        """
        super(WAM, self).__init__()
        self.last_tool_pose = None
        self.last_joint_pose = None

        robot_ns = rospy.get_param('~robot_namespace', 'zeus')
        self.tool_pose_sub = rospy.Subscriber(robot_ns + "/wam/pose", PoseStamped, self.tool_pose_cb)
        self.joint_pose_sub = rospy.Subscriber(robot_ns + "/wam/joint_states", JointState, self.joint_pose_cb)

        self.go_home_srv = rospy.ServiceProxy(robot_ns + "/wam/go_home", Empty)
        self.joint_move_srv = rospy.ServiceProxy(robot_ns + "/wam/joint_move", JointMove)
        self.joint_hold_srv = rospy.ServiceProxy(robot_ns + "/wam/hold_joint_pos", Hold)
        self.pose_move_srv = rospy.ServiceProxy(robot_ns + "/wam/cart_move", CartPosMove)
	self.ori_move_srv = rospy.ServiceProxy(robot_ns + "/wam/ortn_move", OrtnMove)
        self.cart_hold_srv = rospy.ServiceProxy(robot_ns + "/wam/hold_cart_pos", Hold)
        self.hold_ori_srv = rospy.ServiceProxy(robot_ns + "/wam/hold_ortn", Hold)
        self.ori_split_move_srv = rospy.ServiceProxy(robot_ns + "/wam/ortn_split_move",OrtnSplitMove) #ortn_split_move
        self.move_to_pose_srv = rospy.ServiceProxy("/move_to_pose",IKPose)
