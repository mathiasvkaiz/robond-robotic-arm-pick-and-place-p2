#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def rad(deg):
    ''' 
    Convert degree to radian.
    '''

    return deg * pi/180.


def deg(rad):
    '''
    Convert radian to degree.
    '''

    return rad * 180 / pi


def get_transformation_matrix(alpha, a, d, q):
    '''
    Calculates transformation matrix with given values and returns it.
    '''

    matrix = Matrix([[            cos(q),             -sin(q),           0,             a],
                    [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                  0,                   0,           0,             1]])
    
    return matrix


def get_rotation_matrix(motion, angle):
    '''
    Calculates the according motion rotation matrix by a given angle and returns it.
    '''

    matrix = {
        'roll': Matrix([[          1,            0,            0],
                        [          0,    cos(angle), -sin(angle)],
                        [          0,    sin(angle),  cos(angle)]]),
        'pitch': Matrix([[cos(angle),             0,  sin(angle)],
                        [          0,             1,           0],
                        [-sin(angle),             0,  cos(angle)]]),
        'yaw':  Matrix([[ cos(angle),   -sin(angle),            0],
                        [ sin(angle),    cos(angle),            0],
                        [          0,             0,            1]])}

    return matrix[motion]


def calculate_ee(R_EE, p_x, p_y, p_z, roll, pitch, yaw):
    '''
    Corrects End Effector rotation matrix with error and align the given parameters.

    With that the wrist center and the according theta angles are caclulated and returned.
    '''

    # Position matrix of End Effector
    P_EE = Matrix([[p_x],
                   [p_y],
                   [p_z]])
    

    # Error calculation
    error = get_rotation_matrix('yaw', rad(180)) * get_rotation_matrix('pitch', rad(-90))


    # Rotation matrix of End Effector
    R_EE = R_EE * error # compensate errors on it
    R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    # Position matrix of wrist center 
    P_WC = P_EE - (0.303) * R_EE[:, 2] # EE position + offset - EE position = wrist center position
  
    
    # SSS triangle sides and angles
    a = 1.50
    b_xy = sqrt(P_WC[0] * P_WC[0] + P_WC[1] * P_WC[1]) - 0.35
    b_z = P_WC[2] - 0.75
    b = sqrt(pow((b_xy), 2) + pow((b_z), 2))
    c = 1.25 # Length of joint 1 to 2.

    # Triangle angles
    angle_a = acos((b * b + c * c - a * a) / (2 * b * c))
    angle_b = acos((-b * b + c * c + a * a) / (2 * a * c))
    
    # Joint angles
    th_1 = atan2(P_WC[1], P_WC[0])
    th_2 = pi / 2 - angle_a - atan2(b_z, b_xy)
    th_3 = pi / 2 - (angle_b + 0.036)
    
    return (R_EE, P_WC, th_1, th_2, th_3)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        ### Your FK code here
        # Create symbols
        # Joint angles
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

        # DH
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # offset for links
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # length of links
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle

        # EE Poses
        r, p, y = symbols('r p y')
    
        # Create Modified DH parameters
        s = {alpha0:        0, a0:      0, d1:  0.75, q1:           q1,
             alpha1: rad(-90), a1:   0.35, d2:     0, q2: q2 - rad(90),
             alpha2:        0, a2:   1.25, d3:     0, q3:           q3,
             alpha3: rad(-90), a3: -0.054, d4:  1.50, q4:           q4,
             alpha4: rad( 90), a4:      0, d5:     0, q5:           q5,
             alpha5: rad(-90), a5:      0, d6:     0, q6:           q6,
             alpha6:        0, a6:      0, d7: 0.303, q7:            0}
                
        # Create individual transformation matrices
        T_0_1 = get_transformation_matrix(alpha0, a0, d1, q1).subs(s)
        T_1_2 = get_transformation_matrix(alpha1, a1, d2, q2).subs(s)
        T_2_3 = get_transformation_matrix(alpha2, a2, d3, q3).subs(s)
        T_3_4 = get_transformation_matrix(alpha3, a3, d4, q4).subs(s)
        T_4_5 = get_transformation_matrix(alpha4, a4, d5, q5).subs(s)
        T_5_6 = get_transformation_matrix(alpha5, a5, d6, q6).subs(s)
        T_6_EE = get_transformation_matrix(alpha6, a6, d7, q7).subs(s)

        # Extract rotation matrices from the transformation matrices
        T_0_EE = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6 * T_6_EE
        
        # Initialize inverse kinematic rotation matrix
        R_0_3 = T_0_1[0:3, 0:3] * T_1_2[0:3, 0:3] * T_2_3[0:3, 0:3]
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        R_x = get_rotation_matrix('roll', r)
        R_y = get_rotation_matrix('pitch', p)
        R_z = get_rotation_matrix('yaw', y)
        
        # Initialize rotation matrix of End Effector
        R_EE = R_z * R_y * R_x
        
        # Calculate ee values and correct rotation matrix of EE
        R_EE, P_WC, theta1, theta2, theta3 = calculate_ee(R_EE, px, py, pz, roll, pitch, yaw)

        # Inverse kinematic rotation matrix from wraist to EE
        R_0_3 = R_0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        R_3_6 = R_0_3.T * R_EE

        # Angles from rotation matrix
        theta4 = atan2(R_3_6[2, 2], -R_3_6[0, 2])    
        theta5 = atan2(sqrt(R_3_6[0, 2] * R_3_6[0, 2] + R_3_6[2, 2] * R_3_6[2, 2]), R_3_6[1, 2])
        theta6 = atan2(-R_3_6[1, 1], R_3_6[1, 0])
    
        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
