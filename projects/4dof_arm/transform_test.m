% This is the coordinate transformation matrix of adjacent link
syms alpha a d theta;
syms theta1 theta2 theta3 theta4
syms l1 l2 l3
T = [cos(theta)             -sin(theta)             0             a            ;
     sin(theta)*cos(alpha)  cos(theta)*cos(alpha)   -sin(alpha)   -sin(alpha)*d;
     sin(theta)*sin(alpha)  cos(theta)*sin(alpha)   cos(alpha)    cos(alpha)*d ;
     0                      0                       0             1            ];

T00 = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
T01 = subs(T,[alpha a d theta],[0 0 l1 theta1]);
T12 = subs(T,[alpha a d theta],[pi/2 0 0 theta2]);
T23 = subs(T,[alpha a d theta],[0 l2 0 theta3]);
T34 = subs(T,[alpha a d theta],[0 l3 0 theta4]);

T02 = T01*T12
T24 = T23 * T34
T14 = T12 * T24

T04 = T01 * T14
T0404 = T02 * T24


T01 = subs(T01,[l1 theta1],[2 0]);
T12 = subs(T12,theta2,0);
T23 = subs(T23,[l2 theta3],[2 0]);
T34 = subs(T34,[l3 theta4],[2 0]);

T01 = eval(T01);
T12 = eval(T12);
T23 = eval(T23);
T34 = eval(T34);

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;

transform3d(T00,0)
hold on
axis([-1 5 -3 3 -1 5])
transform3d(T01,0)
transform3d(T02,0)
transform3d(T03,0)
transform3d(T04,0)
