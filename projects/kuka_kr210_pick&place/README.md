## Robotic Arm - Pick & Place Project
###1. Kinematic Model
####1-1 Denavit-Hartenberg Diagram
Here below is a Denavit-Hartenberg (DH) diagram of the Kuka arm:   
![](https://i.imgur.com/AqkSogz.png)

The Kuka kr210 manipulator has six revolute joints.    
####1-2 Denavit-Hartenberg Parameters

The DH diagram is created. Now we will define the Denavit-Hartenberg (DH) parameters for describing manipulator kinematics, and the moddified DH method by Craig is used. The DH table are shown as below.

|    i|![alpha][alpha_i-1] |![a][a] |![d][d] |   ![theta][theta] |
|:---:|:------------------:|:------:|:------:|:-----------------:| 
|    1|                  0 |      0 |   0.75 |     ![q1][theta1] |
|    2|      ![-pi2][-pi2] |   0.35 |      0 |  ![q2][theta2-90] |
|    3|                  0 |   1.25 |      0 |     ![q3][theta3] |
|    4|      ![-pi2][-pi2] | -0.054 |  1.501 |     ![q4][theta4] |
|    5|        ![pi2][pi2] |      0 |      0 |     ![q5][theta5] |
|    6|      ![-pi2][-pi2] |      0 |      0 |     ![q6][theta6] |
|   EE|                  0 |      0 |  0.303 |                 0 |

And it is implemented with the below code.

    DH_Table = {alpha0:      0, a0:      0, d1:  0.75, q1:         q1,
                alpha1: -pi/2., a1:   0.35, d2:     0, q2: -pi/2 + q2,
                alpha2:      0, a2:   1.25, d3:     0, q3:         q3,
                alpha3: -pi/2., a3: -0.054, d4:   1.5, q4:         q4,
                alpha4: -pi/2., a4:      0, d5:     0, q5:         q5,
                alpha5: -pi/2., a5:      0, d6:     0, q6:         q6,
                alpha6:      0, a6:      0, d7: 0.303, q7:          0}

After that we move to the forward kinematic of the robot arm.

###2 Forward Kinematic Analasis
The goal of forward kinematics is to calculate the pose of the end-effector given all the six joint angles. 
####2-1 Joint Based Transformation Matrices

The DH convention uses four individual transforms:
![](https://i.imgur.com/imB7wXV.png)
And the code for defining modified DH transformation matrix:

    # Define Modified DH Transformation matrix
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([
                    [           cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                0,                 0,           0,             1]])

        return TF

Using the created DH parameter table and the DH Transformation matrix, we can create individual transforms between various links. 

    # Create individual transformation matrices
    T0_1  = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    T1_2  = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    T2_3  = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    T3_4  = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
    T4_5  = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    T5_6  = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
    
####2-2 Base to Gripper Homogeneous Transformation Matrix
With the individual tranforms  being created above, we obtain the transformation matrix between Gripper and Base.

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

###3. Inverse Kinematic Calculations
In this project, we use the "analytical" or “closed-form” solution to solving the IK problem. Closed-form solutions are specific algebraic equations that do not require iteration to solve. generally they are much faster to solve than numerical approaches and it is easier to develop rules for which of the possible solutions is the appropriate one.     

The last three joints in KUKA KR210 robot are revolute joints that intersect at a single point, such a design is called a spherical wrist and the common point of intersection is called the wrist center. The advantage of such a design is that it kinematically decouples the position and orientation of the end effector.
####3-1 Inverse Kinematic Position    
Provided with the pose of the end effector (The end-effector position and orientation are obtained from the simulation in ROS. ), the first three joint angles of the robot can be calculated.    

First, we need to derive the wrist center position from the transformation matrix as mentioned below based on the end-effector position and orientation. Once we have the pose of end-effector, we can calculate the rotational matrix of end-effector with respect to the base. Note that we must compensate for rotation discrepancy between DH parameters and Gazebo.    

    # Correctional rotation matrix
    Rot_corr = ROT_z.subs(y, pi) * ROT_y.subs(p, -pi/2)

    # EE Rotation
    ROT_EE = ROT_z * ROT_y * ROT_x

    # Compensate for rotation discrepancy between DH parameters and Gazebo
    ROT_EE = ROT_EE * Rot_corr

Second, calculate the joint angle theta1 using atan2 method. The geometry relationship is very direct.   
![](https://i.imgur.com/ymDdeuF.png)  

Third, calculate the joint angle theta2 and theta3 using cosine laws. However, the geometric schematic diagram in this phase is a little tricky!
![](https://i.imgur.com/0cSyIUt.png)

With the above diagram, we can calculate the theta2 and theta3 by the below equations.
![](https://i.imgur.com/PYtr2Mi.png)

Note that the constants in the above equations is the geometric parameters of the arm, which can be obtained by checking the URDF file.

The implemented code of this part are shown as below.

    # Solve inverse position problem
    # Compute WC position
    ROT_ee = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

    EE = Matrix([[px],
                 [py],
                 [pz]])

    WC = EE - (0.303) * ROT_ee[:,2]

    # Calculate joint angles using Geometric IK method
    # Triangle to calculate theta2 and theta3
    side_wc = sqrt(WC[0]**2 + WC[1]**2)
    side_a = 1.501
    side_b = sqrt(pow((side_wc - 0.35), 2) + pow((WC[2] - 0.75), 2))
    side_c = 1.25

    angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
    angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))
    angle_c = acos((side_a**2 + side_b**2 - side_c**2) / (2 * side_a * side_b))

    theta1 = atan2(WC[1], WC[0])
    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, side_wc - 0.35)
    theta3 = pi / 2 - (angle_b + 0.036)
    

####3-2 Inverse Kinematic Orientation   
With the first three joint angle values, and the rotational matrix form base to end effector ( note that the end effector and link 6 has a same orientation) being calculate, we are now able to solve the inverse orientation problem.  

We first calculte the rotation matrix from link3 to link6, ang the following equesions hold true:
![](https://i.imgur.com/zBca0xU.png)


Then the joint angles can be calculted by deriving enler angles from the rotation matrix:
![](https://i.imgur.com/P5vtZ7L.png)
 
The code for solving inverse orientation are shown as below:

    # Solve inverse orientation problem
    R0_3_inv = R0_3_INV.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3_inv * ROT_ee

    theta5 = atan2(sqrt(R3_6[0, 2]**2 + R3_6[2, 2]**2), R3_6[1, 2])
    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

I put the defining code of [R_3_6] before the for loop, and I find it performs faster. Further more, I replace the inverse method as the transpose method for transpose method performs well both in precision and computation speed on the process of simulation.

    # Compute inverse of rotation matrix R0_3 preparing for solving 
    # inverse orientation problem
    R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
    R0_3_INV = R0_3.T

###4. Simulation Result
The simulation result are showed as below. The kuka arm is able to successfully complete 8/10 pick and place cycles. I picked out an interesting simulation moment--the jars are stacked together, which proves that the algorithm is of better accuracy. 
![](https://i.imgur.com/NcJANQQ.jpg)

![](https://i.imgur.com/vnui3PW.png)

###5. Future Works
However, there are two distinct solution methods to solving the IK problem in robotics. The first is a purely numerical approach. The other, and much preferred solution method is a known as an "analytical" or “closed-form” solution. In this project, we use the analytical solution method. Although this method works very well in this project, it can’t be used in all serial manipulators. So I think it's necessary to learn the numerical approach, which is a very important method with general applicability. I plan to learning this technique after I graduate form this Nanodegree.

