function [rads1,rads2] = computeRrInverseKinematics(X,Y)

L1 = 1; L2 = 1;
syms theta1 theta2 ;

rads1=0;
rads2=0;

%% Calculate theta2
% calculate cosine value of theta2
theta2_cos = (X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2);

% Calculate sine value of theta2
theta2_sin_pos = sqrt(1-((X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2))^2);
theta2_sin_neg = -theta2_sin_pos;

theta2_1 = atan2(theta2_sin_pos, theta2_cos);
theta2_2 = atan2(theta2_sin_neg, theta2_cos);

%% Calculate theta1
theta1_cos_1 = (X * (L1 + L2 * theta2_cos) + Y * L2 * theta2_sin_pos) / (X^2 + Y^2);
theta1_cos_2 = (X * (L1 + L2 * theta2_cos) + Y * L2 * theta2_sin_neg) / (X^2 + Y^2);

theta1_sin_1 = sqrt(1-(theta1_cos_1));
theta1_sin_2 = sqrt(1-(theta1_cos_2));

theta1_1 = atan2(theta1_sin_1, theta1_cos_1);
theta1_2 = atan2(theta1_sin_2, theta1_cos_2);

%% Limit the return value: assume 0<theta1<pi/2
rads1 = theta1_1;
rads2 = theta2_1;