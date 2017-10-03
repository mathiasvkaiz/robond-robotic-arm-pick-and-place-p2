## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./images/dh.jpg
[image2]: ./images/homogenous.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Based on KuKa Forward Kinematics i derived following Scheme of the setup and relevant variables.

In combination with the `kr210.urdf.xacro` file, especially the joints part i could derive following DH table. in this section the joints relative to their origin are defined, so i can derive relevant `a` and `d`.

It is important to note:
- alpha is the twist angles of the z-Axis
- a is the distance between the z- Axis
- d is the offset between the x-Axis
- q is the joint vraible (theta)

This is related to joint `i` with respect to joint `i-1`

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

For deriving homogenous transforms i wrote a function (placed in file `IK_server.py` from line 58 on) called `get_transformation_matrix()` tha created the transformation matrix based on the input parameters `alpha, a, d and q`.
The function body looks like this

```python

matrix = Matrix([[            cos(q),             -sin(q),           0,             a],
                [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                  0,                   0,           0,             1]])
```

Subsituting the parametrs/values into this matrix for each adjacent link folloing results are dervied:

```python

Matrix([[cos(q1), -sin(q1), 0, a0], 
        [sin(q1)*cos(alpha0), cos(alpha0)*cos(q1), -sin(alpha0), -d1*sin(alpha0)],                                                   [sin(alpha0)*sin(q1), sin(alpha0)*cos(q1), cos(alpha0), d1*cos(alpha0)], 
        [0, 0, 0, 1]])

Matrix([[cos(q2), -sin(q2), 0, a1], 
        [sin(q2)*cos(alpha1), cos(alpha1)*cos(q2), -sin(alpha1), -d2*sin(alpha1)],                                                   [sin(alpha1)*sin(q2), sin(alpha1)*cos(q2), cos(alpha1), d2*cos(alpha1)], 
        [0, 0, 0, 1]])

Matrix([[cos(q3), -sin(q3), 0, a2], 
        [sin(q3)*cos(alpha2), cos(alpha2)*cos(q3), -sin(alpha2), -d3*sin(alpha2)],                                                   [sin(alpha2)*sin(q3), sin(alpha2)*cos(q3), cos(alpha2), d3*cos(alpha2)], 
        [0, 0, 0, 1]])

Matrix([[cos(q4), -sin(q4), 0, a3], 
        [sin(q4)*cos(alpha3), cos(alpha3)*cos(q4), -sin(alpha3), -d4*sin(alpha3)],                                                   [sin(alpha3)*sin(q4), sin(alpha3)*cos(q4), cos(alpha3), d4*cos(alpha3)], 
        [0, 0, 0, 1]])

Matrix([[cos(q5), -sin(q5), 0, a4], 
        [sin(q5)*cos(alpha4), cos(alpha4)*cos(q5), -sin(alpha4), -d5*sin(alpha4)],                                                   [sin(alpha4)*sin(q5), sin(alpha4)*cos(q5), cos(alpha4), d5*cos(alpha4)], 
        [0, 0, 0, 1]])

Matrix([[cos(q6), -sin(q6), 0, a5], 
        [sin(q6)*cos(alpha5), cos(alpha5)*cos(q6), -sin(alpha5), -d6*sin(alpha5)],                                                   [sin(alpha5)*sin(q6), sin(alpha5)*cos(q6), cos(alpha5), d6*cos(alpha5)], 
        [0, 0, 0, 1]])

Matrix([[cos(q7), -sin(q7), 0, a6], 
        [sin(q7)*cos(alpha6), cos(alpha6)*cos(q7), -sin(alpha6), -d7*sin(alpha6)],                                                   [sin(alpha6)*sin(q7), sin(alpha6)*cos(q7), cos(alpha6), d7*cos(alpha6)], 
        [0, 0, 0, 1]])
```


A generalized matrix from base link tor gripper is showed in following image:
![alt text][image2]

The transformation matrix from base link to EE is derived in `IK_server.py` code line 170.
Correction of Grabber (line 104)  and pose of Grabber (line 109) are applied in function `calculate_ee()` from line 90 on.



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

I used the closed-form approach that is much faster than a numerical approach to get the joint angles by known pose of End Effector (Grabber).



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


