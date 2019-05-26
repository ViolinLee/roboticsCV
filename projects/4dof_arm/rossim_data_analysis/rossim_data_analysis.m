%% Load simulation data
load('grinder_position_true.mat');
load('grinder_position_detection.mat');

% figure(1);
% subplot(1,3,1);plot(x_true);hold on; plot(xx);hold on;
% subplot(1,3,2);plot(y_true);hold on; plot(yy);hold on;
% subplot(1,3,3);plot(z_true);hold on; plot(zz);hold on;
%% Normal distribution to simulate tracking errors
moveit_threhold = 0.002;
mu = 0;
sigma = moveit_threhold/2.2;

errors = normrnd(mu,sigma,[3,61]);
max(errors(1,:))
max(errors(2,:))
max(errors(3,:))
%% Adjust position value using mean value
x_mean = sum(xx-x_true)/61;
y_mean = sum(yy-y_true)/61;
z_mean = sum(zz-z_true)/61;

x_true = x_true + x_mean;
y_true = y_true + y_mean;
z_true = z_true + z_mean;

figure(2);
subplot(1,3,1);plot(x_true);hold on; plot(xx);hold on; xlabel('变量 X'); ylabel('变量 Y1 & Y2');
subplot(1,3,2);plot(y_true);hold on; plot(yy);hold on; xlabel('变量 X'); ylabel('变量 Y1 & Y2');
subplot(1,3,3);plot(z_true);hold on; plot(zz);hold on; xlabel('变量 X'); ylabel('变量 Y1 & Y2');

%% Add normal distribution value to simulation moveit! errors
x_moveit = xx + errors(1,:);
y_moveit = yy + errors(2,:);
z_moveit = zz + errors(3,:);

figure(3);
subplot(1,3,1);plot(x_true);hold on; plot(x_moveit);hold on;
subplot(1,3,2);plot(y_true);hold on; plot(y_moveit);hold on;
subplot(1,3,3);plot(z_true);hold on; plot(z_moveit);hold on;

%% 3d curve
figure(4);
plot3(x_true,y_true,z_true,'r--','LineWidth',2); hold on;       % Ture position
plot3(xx,yy,zz,'g','LineWidth',2); hold on;                     % Detection position 

figure(5);
plot3(x_true,y_true,z_true,'r--','LineWidth',2); hold on;       % Ture position
plot3(x_moveit,y_moveit,z_moveit,'b','LineWidth',2); hold on;   % Moveit! position

%% Tracking errors
figure(6);
x_errors = xx - x_moveit;
y_errors = yy - y_moveit;
z_errors = zz - z_moveit;

subplot(1,3,1);plot(x_errors);hold on;
subplot(1,3,2);plot(y_errors);hold on;
subplot(1,3,3);plot(z_errors);hold on;

figure(7);
x_errors = x_true - x_moveit;
y_errors = y_true - y_moveit;
z_errors = z_true - z_moveit;

subplot(1,3,1);plot(x_errors);hold on;
subplot(1,3,2);plot(y_errors);hold on;
subplot(1,3,3);plot(z_errors);hold on;
