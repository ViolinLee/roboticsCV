function [endeff] = computeMiniForwardKinematics(rads1,rads2)

L1 = 1; L2 = 2;

alpha = 1/2 * (rads1 + rads2) + pi;
beta = rads1 - rads2;

Y = sqrt(L1^2 + L1^2 - 2*L1*L2*cos(beta));
Z = sqrt(L1^2 - (Y/2)^2);
X = sqrt(L2^2 - (Y/2)^2);

l = X - Z;

x = l * cos(alpha);
y = l * sin(alpha);

endeff = [x, y];