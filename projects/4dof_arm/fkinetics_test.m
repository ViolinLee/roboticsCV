[l1, l2, l3, l4] = deal(0.4, 0.4, 0.4, 0.4);

L1 = Revolute('d', l1, 'a',0,'alpha',pi/2);
L2 = Revolute('d', 0, 'a',l2,'alpha',0);
L3 = Revolute('d', 0, 'a',l3,'alpha',0);
L4 = Revolute('d', 0, 'a',l4,'alpha',0);

vactube = SerialLink([L1, L2, L3, L4], 'name', 'vacuum tube')
%vactube.plot([30*pi/180,0,0,0])

%vactube.plot([-45*pi/180,30*pi/180,-15*pi/180,-30*pi/180])

vactube.plot([0,30*pi/180,-15*pi/180,-30*pi/180])

vactube.fkine([0,30*pi/180,-15*pi/180,-30*pi/180])

Td = [[sqrt(3)/2 1/2 0 1];[0 0 -1 0];[-1/2 sqrt(3)/2 0 0.6];[0 0 0 1]];
q_goal = vactube.ikine(Td,'q0', [0 0 0.6 -1], 'mask', [1 1 1 0 0 1])
% Td, 'q0', [0.5 1 -1 -0.5], 'mask', [1 1 1 0 0 0]

vactube.plot(q_goal)

%% ikine geometric solution
% syms z2 z4 l2 l3 th2 th3;
% z4 = z2 + l2*sin(th2) - l3*sin(th3 - th2);
% u4 = l2*cos(th2) + l3*cos(th3 - th2);
% [th2, th3] = solve(z4,u4);
% th2 = simplify(th2), th3 = simplify(th3)
l1 = 4;l2 = 4; l3 = 4; l4 = 4;

th1 = 0;z2 = l2;
[xt, yt, zt, sigma] = deal(1, 0, 0.6, 30*pi/180); 

ut = sqrt(xt^2 + yt^2);

x4 = xt - l4*cos(sigma)*sin(th1);
y4 = yt - l4*cos(sigma)*cos(th1);
z4 = zt + l4*sin(sigma);
u4 = sqrt(x4^2 + y4^2);

r = sqrt(u4^2+(z4-z2)^2);

th3 = acos((l2^2+l3^2-r^2)/(2*l2*l3))
th2 = atan(l3*sin(th3)/(l2+l3*cos(th3))) + atan(z4 - z2)/u4
th4 = th2 - th3 + sigma