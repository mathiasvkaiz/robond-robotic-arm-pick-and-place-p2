from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[P_WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


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


def get_ee_poses(poses):
	'''
	Gettr for extract end-effector position and orientation from request.

	px,py,pz = end-effector position
	roll, pitch, yaw = end-effector orientation 
	'''
	
	p_x = poses.position.x
	p_y = poses.position.y
	p_z = poses.position.z

	(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([poses.orientation.x, 
																   poses.orientation.y, 
														   		   poses.orientation.z, 
														   		   poses.orientation.w])

	return p_x, p_y, p_z, roll, pitch, yaw


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
                        [          0,    cos(angle),  sin(angle)],
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
	error = get_rotation_matrix('pitch', rad(180)) * get_rotation_matrix('yaw', rad(-90))

	# Rotation matrix of End Effector
	R_EE = R_EE * error # comepnsate errors on it
	R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

	# Position matrix of wrist center 
	P_WC = P_EE - (0.303) * R_EE[:, 2] # EE position + offset - EE position = wrist center position

	
	
	# SSS triangle sides and angles
	a = 1.501
	b = sqrt(pow((sqrt(P_WC[0] * P_WC[0] + P_WC[1] * P_WC[1]) - 0.35), 2) + pow((P_WC[2] - 0.75), 2))
	c = 1.25 # Length of joint 1 to 2.

	angle_a = acos((b * b + c * c - a * a) / (2 * b * c))
	angle_b = acos((a * a + c * c - b * b) / (2 * a * c))
	

	# Joint angles
	th_1 = atan2(P_WC[1], P_WC[0])
	th_2 = pi / 2 - angle_a - atan2(P_WC[2] - 0.75, sqrt(P_WC[0] * P_WC[0] + P_WC[1] * P_WC[1]) - 0.35)
	th_3 = pi / 2 - (angle_b + 0.036)
	
	return (R_EE, P_WC, th_1, th_2, th_3)




def test_code(test_case):
	## Set up code
	## Do not modify!
	x = 0
	class Position:
	    def __init__(self,EE_pos):
	        self.x = EE_pos[0]
	        self.y = EE_pos[1]
	        self.z = EE_pos[2]
	class Orientation:
	    def __init__(self,EE_ori):
	        self.x = EE_ori[0]
	        self.y = EE_ori[1]
	        self.z = EE_ori[2]
	        self.w = EE_ori[3]

	position = Position(test_case[0][0])
	orientation = Orientation(test_case[0][1])

	class Combine:
	    def __init__(self,position,orientation):
	        self.position = position
	        self.orientation = orientation

	comb = Combine(position,orientation)

	class Pose:
	    def __init__(self,comb):
	        self.poses = [comb]

	req = Pose(comb)
	start_time = time()

	########################################################################################
	## 
	
	# Symbols for joint variables
	# Joint angles
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	# DH
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # offset for links
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # length of links
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle

	# EE Poses
	r, p, y = symbols('r p y')

	#-----------------------------------#

	# DH table
	s = {alpha0:        0, a0:      0, d1:  0.75, q1: 	 	    q1,
  	 	 alpha1: rad(-90), a1:   0.35, d2:     0, q2: q2 - rad(90),
     	 alpha2:        0, a2:   1.25, d3:     0, q3: 		    q3,
     	 alpha3: rad(-90), a3: -0.054, d4:  1.50, q4: 		    q4,
         alpha4: rad( 90), a4:      0, d5:     0, q5: 		    q5,
         alpha5: rad(-90), a5:      0, d6:     0, q6: 		    q6,
         alpha6:        0, a6:      0, d7: 0.303, q7:  			 0}

	
	# Transformation matrices
	T_0_1 = get_transformation_matrix(alpha0, a0, d1, q1).subs(s)
	T_1_2 = get_transformation_matrix(alpha1, a1, d2, q2).subs(s)
	T_2_3 = get_transformation_matrix(alpha2, a2, d3, q3).subs(s)
	T_3_4 = get_transformation_matrix(alpha3, a3, d4, q4).subs(s)
	T_4_5 = get_transformation_matrix(alpha4, a4, d5, q5).subs(s)
	T_5_6 = get_transformation_matrix(alpha5, a5, d6, q6).subs(s)
	T_6_EE = get_transformation_matrix(alpha6, a6, d7, q7).subs(s)

	T_0_EE = T_0_1 * T_1_2 * T_2_3 * T_3_4 * T_4_5 * T_5_6 * T_6_EE

	# EE poses
	p_x, p_y, p_z, roll, pitch, yaw = get_ee_poses(req.poses[x])

	# Rotation matrices
	R_x = get_rotation_matrix('roll', r)
	R_y = get_rotation_matrix('pitch', p)
	R_z = get_rotation_matrix('yaw', y)

	# Initialize rotation matrix of End Effector
	R_EE = R_z * R_y * R_x

	# Calculate wrist values and correct rotation matrix of EE
	R_EE, P_WC, th_1, th_2, th_3 = calculate_ee(R_EE, p_x, p_y, p_z, roll, pitch, yaw)

	# Inverse kinematic rotation matrix from wraist to EE
	R_0_3 = T_0_1[0:3, 0:3] * T_1_2[0:3, 0:3] * T_2_3[0:3, 0:3]
	R_0_3 = R_0_3.evalf(subs={q1: th_1, q2: th_2, q3: th_3})
	R_3_6 = Transpose(R_0_3) * R_EE

	#print R_0_3.T
	#print Transpose(R_0_3)

	# Angles from rotation matrix
	th_4 = atan2(R_3_6[2, 2], -R_3_6[0, 2])    
	th_5 = atan2(sqrt(R_3_6[0, 2] * R_3_6[0, 2] + R_3_6[2, 2] * R_3_6[2, 2]), R_3_6[1, 2])
	th_6 = atan2(-R_3_6[1, 1], R_3_6[1, 0])
	
	print ("Theta4:", th_4)
	print ("Theta5:", th_5)
	print ("Theta6:", th_6)


	# set angles
	theta1 = th_1
	theta2 = th_2
	theta3 = th_3
	theta4 = th_4
	theta5 = th_5
	theta6 = th_6

	

	## 
	########################################################################################

	########################################################################################
	## For additional debugging add your forward kinematics here. Use your previously calculated thetas
	## as the input and output the position of your end effector as your_ee = [x,y,z]

	## (OPTIONAL) YOUR CODE HERE!
	EE = T_0_EE.evalf(subs={'q1':theta1, 'q2':theta2, 'q3':theta3, 'q4':theta4, 'q5':theta5, 'q6':theta6})

	## End your code input for forward kinematics here!
	########################################################################################

	## For error analysis please set the following variables of your P_WC location and EE location in the format of [x,y,z]
	
	your_P_WC = [P_WC[0], P_WC[1], P_WC[2]]
	your_ee = EE[:3, 3]
	########################################################################################

	## Error analysis
	print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

	# Find P_WC error
	if not(sum(your_P_WC)==3):
	    P_WC_x_e = abs(your_P_WC[0]-test_case[1][0])
	    P_WC_y_e = abs(your_P_WC[1]-test_case[1][1])
	    P_WC_z_e = abs(your_P_WC[2]-test_case[1][2])
	    P_WC_offset = sqrt(P_WC_x_e**2 + P_WC_y_e**2 + P_WC_z_e**2)
	    print ("\nWrist error for x position is: %04.8f" % P_WC_x_e)
	    print ("Wrist error for y position is: %04.8f" % P_WC_y_e)
	    print ("Wrist error for z position is: %04.8f" % P_WC_z_e)
	    print ("Overall wrist offset is: %04.8f units" % P_WC_offset)

	# Find theta errors
	t_1_e = abs(theta1-test_case[2][0])
	t_2_e = abs(theta2-test_case[2][1])
	t_3_e = abs(theta3-test_case[2][2])
	t_4_e = abs(theta4-test_case[2][3])
	t_5_e = abs(theta5-test_case[2][4])
	t_6_e = abs(theta6-test_case[2][5])
	print ("\nTheta 1 error is: %04.8f" % t_1_e)
	print ("Theta 2 error is: %04.8f" % t_2_e)
	print ("Theta 3 error is: %04.8f" % t_3_e)
	print ("Theta 4 error is: %04.8f" % t_4_e)
	print ("Theta 5 error is: %04.8f" % t_5_e)
	print ("Theta 6 error is: %04.8f" % t_6_e)
	print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
	       \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
	       \nconfirm whether your code is working or not**")
	print (" ")

	# Find FK EE error
	if not(sum(your_ee)==3):
	    ee_x_e = abs(your_ee[0]-test_case[0][0][0])
	    ee_y_e = abs(your_ee[1]-test_case[0][0][1])
	    ee_z_e = abs(your_ee[2]-test_case[0][0][2])
	    ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
	    print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
	    print ("End effector error for y position is: %04.8f" % ee_y_e)
	    print ("End effector error for z position is: %04.8f" % ee_z_e)
	    print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    #test_case_number = 1
    #test_code(test_cases[test_case_number])
    for i in range(1, 3):
		print ("################# Test Case" + str(i) + " #################")
		test_case_number = i
		test_code(test_cases[test_case_number])
