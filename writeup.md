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
[image2]: ./images/homogenous_transform.jpg
[image3]: ./images/homogenous.jpg
[image4]: ./images/wrist_center.jpg
[image5]: ./images/sss.jpg
[image6]: ./images/first_try.jpg
[image7]: ./images/rotate_before_pickup.jpg
[image8]: ./images/pick1.jpg

[video1]: ./images/rotating.mov "Video"
[video2]: ./images/release.mov "Video"
[video3]: ./images/final.mov "Video"


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

For deriving homogenous transforms i wrote a function (placed in file `IK_server.py` from line 51 on) called `get_transformation_matrix()` tha created the transformation matrix based on the input parameters `alpha, a, d and q`.
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


A generalized homogenous transformation matrix between neighbor links is shown in following image: 
![alt text][image2]

The overall matrix from base_link to Grapper with respect to pose (position and orientation) is shown following:
![alt text][image3]

The transformation matrix from base link to EE is derived in `IK_server.py` code line 152.
Correction of Grabber (line 84)  and pose of Grabber (line 88) are applied in function `calculate_ee()` from line 70 on.



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

I used the closed-form approach that is much faster than a numerical approach to get the joint angles by known pose of End Effector (Grabber). But it has it's limitations that only on cetrain cases this approach can be used.

There are two conditions of whose one must be satisfied:
- Three neighboring joint axes intersect at a single point
- Three neighboring joint axes are parallel (which is technically a special case of 1, since parallel lines intersect at infinity)

Since the last three joints have an intersction point in joint 5 one of the above conditions is satisfied and therefor we have a sqherical wrist with joint 5 as it's center. This leads to an easier calculation (kunematic decoupling of the position and orientation of the end effector) by calculation the coordinates of the wrist center and then the composition of rotations to orient the end effector.

As basis for the wrist center calculation i used the transformation matrix based on the end-effector pose (code line 109).
The wrist center is calculated in line 117 based on following formula:

![alt text][image4]


Looking at the code line we can see following:
```python
P_WC = P_EE - (0.303) * R_EE[:, 2] # EE position + offset - EE position = wrist center position
```
The .303 is derived from the DH table as d7, meaning the distance between x-Axis of base link and End Effector. R_EE[:, 2] is the third line of the matrix according to the formula.

Derivation of the first three thetas:
- Theta 1: The wrist center needs to be projected onto the ground plane meaning set the z-coordinate = 0 (code line 131)
- Theta 2, 3: Triangle in image below is calculation base for Theta2 and Theta3. It is assumed that z of wrist center equals 0 and that we have also link 2 and 3 projected onto the ground plane. Then i can calculate by cosine law the angles (code line 96 - 109)

Derivation of Theta 4 - 6:
We are now coming to the Inverse Orientation problem. As the overall RPY (Roll, Pitch Yaw) rotation between base link and Grapper is equal to the overall product of each links i can inverse the matrix for the first three joints and multiply it with the RPY (code lines 187 - 188).

Based on Euler Angles from a Rotation Matrix section in the Lesson and the walkthrough i got the Thetas 4 - 6 (code line 191 - 193)

To avoid multiple solutions i used the atan2 function so that signs can be regarded and the correct quadrant can be derived.


![alt text][image5]


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

I used functional programming techniques and defined several methods for getting / applying matrices (transformation and rotation). These functions where placed outside the `handle_calculate_IK()` function to get some performance improvements as in this function we have a loop iterating through multipe values (poses).

I also put the calculation and error correction of the End Effector rotation matrix (including the wrist center calculation and Thetas 1-3) in one function `calculate_ee()` as these are all related to forward kinematics.

The definitions of the martices and dh table is placed outside the loop, as do need to be initialized only once. 

As calculating with matrices this can lead to enormous delays. It is clear to me, that this apporach does only lead to minimal performance gains, there could be several other methods like using other (performance optimized libraries like numpy).

To get better results the focus should lie on error calcluation. By comparing the calculated position with the received ones from the pose i could do a much better error handling and therefor calculation of the expected position. 


There were several issues while the Robotic Arm was moving. First the overall performance was very poor. There were many rotating iterations while moving to the goal:

![alt text][image7]


After it reached the goal it was not sure that it could grab the blue stick accordingly:

![alt text][image6]

In case it got it correct

![alt text][image8]

there were several related issues moving to the drop zone.
First there were also many rotation cycles while moving:

Here's a [link to the rotation cylcles](./images/rotating.mov)

After it got to the drop zone there were also many cylces befor drop off.

Here's a [link to the drop off](./images/release.mov)


So there are definitively possibilities of improvement. The cycles seem to be related to the overall calculation. So it has to be possible, to switch off calculation/movement of special jonts and only the absolute relevant should move (not the whole construct). 

For the correct pick up, the robot should be somehow able to "see" objects in sight so that he could measure the dimensions of the object and then plan its detailed movement according to it.

Update after first submission:
In my first attempt i tried to substitute  those expressions/matrices with the received/calculated values inside the loop. Also all inverse kinematic topics were then calculated inside the main loop. This lead to the described issues above. I move the whole calculation outide the for loop (related to code from line 176ff). This lead to the desired results that my robot behaves like wanted.

Here's a [link to the successful drop off](./images/final.mov)



