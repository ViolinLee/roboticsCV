syms theta1 theta2 theta3 theta4
l1 = 4; l2 =4; l3 = 4;
T04 = [[ cos(theta1)*cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - cos(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)), - cos(theta1)*cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) - cos(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)),  sin(theta1), cos(theta1)*cos(theta2)*(l2 + l3*cos(theta3)) - l3*cos(theta1)*sin(theta2)*sin(theta3)];
[ cos(theta2)*sin(theta1)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta1)*sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)), - cos(theta2)*sin(theta1)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) - sin(theta1)*sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)), -cos(theta1), cos(theta2)*sin(theta1)*(l2 + l3*cos(theta3)) - l3*sin(theta1)*sin(theta2)*sin(theta3)];
[                         cos(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)) + sin(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)),                           cos(theta2)*(cos(theta3)*cos(theta4) - sin(theta3)*sin(theta4)) - sin(theta2)*(cos(theta3)*sin(theta4) + cos(theta4)*sin(theta3)),            0,                    l1 + sin(theta2)*(l2 + l3*cos(theta3)) + l3*cos(theta2)*sin(theta3)];
[                                                                                                                                                         0,                                                                                                                                                           0,            0,                                                                                      1]];

T4tool = [[1 0 0 4];
          [0 1 0 0];
          [0 0 1 0];
          [0 0 0 1]];
      
T0tool = T04 * T4tool;

Tx = T0tool(1,4);
Ty = T0tool(2,4);
Tz = T0tool(3,4);

num = 1;
[th2min, th2max, th3min, th3max, th4min, th4max] = deal(-10, 10, -10, 10, 0, 60);
inter = 2;
p_num = (th2max/inter - th2min/inter + 1) * (th3max/inter - th3min/inter + 1) * (th4max/inter - th4min/inter + 1);
point = zeros(1, p_num);
[x, y, z] = deal(point);
th1rad = 0;

for th2= th2min:inter:th2max
    for th3=th3min:inter:th3max
        for th4=th4min:inter:th4max
                       
            th2rad = th2 * pi / 180;
            th3rad = th3 * pi / 180;
            th4rad = th4 * pi / 180;
                
            x(num) = eval(subs(Tx,[theta1 theta2 theta3 theta4],[th1rad th2rad th3rad th4rad]));
            y(num) = eval(subs(Ty,[theta1 theta2 theta3 theta4],[th1rad th2rad th3rad th4rad]));
            z(num) = eval(subs(Tz,[theta1 theta2 theta3 theta4],[th1rad th2rad th3rad th4rad]));

            num=num+1;
                
        end
    end
end
plot(x,z,'g*')
axis equal
 