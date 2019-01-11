function endeff = computeForwardKinematics(rads)

L1 = 1;

x = L1 * cos(rads);
y = L1 * sin(rads);


endeff = [x,y];

