## Project: Kinematics Pick & Place


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I first ran the demo by following the instructions. Manipulated the joints in Rviz to better understand the links of the robot arm and how the rotations affect each other.
From the kr201.urdf.xacro, we can get the DH parameters of the kuka arm and the pose of the robot when all the joint variables are equal to zero. The arm is referenced in the xz frame. The number of joints in arm are numbered from 1 to 6 and the no of links are numbered from 0-6.
The values we get from the file are alpha0-6, a0-6, d1-7 
alpha(i-1) are twist angles between z(i-1) and z(i) measured about x(i-1).
a(i-1) are link lengths ,distance from z(i-1) and z(i) measured along  x(i-1) where x(i-1) is perpendicular to both  z(i-1) and z(i).
d(i-1) are link offsets , signed distance from  x(i-1) to x(i),measured along z(i).
theta(i) are joint angles, angle between  x(i-1) to x(i) measured about z(i). 

Model to derive DH parameters.
![alt text][misc_images/diagram.png]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35	 | 0 | -pi/2 + q2
2->3 | 0 | 1.25	 | 0 | q3
3->4 |  - pi/2| -0.054 | 1.5	 | q4
4->5 |  pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
In order to create individual transformation matrices we are first going to create a reusable function based on the following equation: 

![alt text][misc_images/dh-transform-matrix.png]

The code below creates the full transform to the End-effector gripper pose from the fixed base link. Substituted DH parameters into the Transformation Matrix function, for each individual length between links and then multiplied them all together to find the full transformation from base to end-effector(T0_EE).

   def TF_Matrix(alpha, a, d, q):
           TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])
           return TF

        #
        # Create individual transformation matrices
        T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(DH)
        T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(DH)
        T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(DH)
        T3_4 = Trans_Matrix(alpha3, a3, d4, q4).subs(DH)
        T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(DH)
        T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(DH)
        T6_EE = Trans_Matrix(alpha6, a6, d7, q7).subs(DH)

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        #Set Roll Pitch and Yaw to end-effector postion
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [req.poses[x].orientation.x, req.poses[x].orientation.y,
               req.poses[x].orientation.z, req.poses[x].orientation.w])

        # Create Rotation Matrices
        Roll_rot = Matrix([[ 1,         0,          0],
                         [ 0, cos(r), -sin(r)],
                         [ 0, sin(r), cos(r)]])

        Pitch_rot = Matrix([[ cos(p),  0, sin(p)],
	                       [          0,  1,          0],
	                       [-sin(p),  0, cos(p)]])

        Yaw_rot = Matrix([[ cos(y), -sin(y), 0],
	                    [ sin(y),  cos(y), 0],
	                    [        0,         0, 1]])

        EE_rot = Yaw_rot * Pitch_rot * Roll_rot

        #Numerically Evaluate transforms to compare with tf_echo
        # print("T0_1 = ",T0_1 = Trans_Matrix(alpha0, a0, d1, q1).subs(DH_test))
        # print("T1_2 = ",T1_2 = Trans_Matrix(alpha1, a1, d2, q2).subs(DH_test))
        # print("T2_3 = ",T2_3 = Trans_Matrix(alpha2, a2, d3, q3).subs(DH_test))
        # print("T3_4 = ",T3_4 = Trans_Matrix(alpha3, a3, d5, q4).subs(DH_test))
        # print("T4_5 = ",T4_5 = Trans_Matrix(alpha4, a4, d5, q5).subs(DH_test))
        # print("T5_6 = ",T5_6 = Trans_Matrix(alpha5, a5, d6, q6).subs(DH_test))
        # print("T6_G = ",T6_G = Trans_Matrix(alpha6, a6, d7, q7).subs(DH_test))

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        # calculate error between urdf and dh
        #rotate 180 degrees around z
        R_z = Yaw_rot.subs(y, radians(180))
        #Rotate 90 degrees around y
        R_y = Pitch_rot.subs(p, radians(-90))

        R_error = simplify(R_z * R_y)

        EE_rot = EE_rot * R_error

        EE_rot = EE_rot.subs({'r': roll, 'p': pitch, 'y': yaw})

        #
        # Extract rotation matrices from the transformation matrices
        # End effector positions:
        px = req.poses[x].position.x
        py = req.poses[x].position.y
        pz = req.poses[x].position.z
        
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.        
        
Wrist center(WC) is created with the following function using joint 5.
       
        # Wrist center calculation  
        WC = Matrix([px, py, pz]) - (DH[d7]*1.23) * EE_rot[:,2]
        
Joints 1-3 are used in order to position the WC correctly which is inverse position kinematics. They are called Theta 1-3 in the code. Theta 1 is calculated using WC array. 

        theta1 = atan2(WC[1], WC[0]) # Equation = atan2(WCy, WCx)

We get Theta2 with SSS triangle constructed using joint 2, joint 3, and WC.
![alt text][misc_images/angle.png]



And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


