xt = 1; yt = 0; zt = 0.6; sigma = 30 * pi /180;

ut = sqrt(xt^2 + yt^2);
theta1 = 0;

z4 = zt + 0.4 * sin(sigma);
y4 = yt - 0.4 * cos(sigma)*sin(theta1);
x4 = xt - 0.4 * cos(sigma)*cos(theta1);

u4 = sqrt(x4^2 + y4^2);

r2 = u4^2 + (z4-0.4)^2;
theta3 = acos((0.4^2+0.4^2-r2)/(2*0.4*0.4))-pi;

beta = atan2(-0.4*sin(theta3), (0.4+0.4*cos(theta3)));

gamma = atan2((z4 - 0.4),u4);

theta2 = beta + gamma;

theta4 = -(theta3 + theta2 + sigma);