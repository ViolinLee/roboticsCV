%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%               Endeffector2.m
%This program gives the end effector position as a points
% for the Ryerson Schuck Robotic Arm and store the result 
%of the end-effector axis in a file colled:
% Endeffector count Pex Pey Pez 
% where 
%       Endeffector : a data file
%       count :  number of samples
%       Pex, Pey, Pez : end-effector point
% 
% It use a single fuction called:
% initialEndeffector2:
%                 which used to initialization
% Written by Salam 7 Feb 2014
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clc;
 
initialEndeffector2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Calculation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




THETA2 = theta2_Min:Stepsize:theta2_Max; % all possible theta2 values
THETA3 = theta3_Min:Stepsize:theta3_Max;
THETA4 = theta4_Min:Stepsize:theta4_Max;

[theta2, theta3, theta4] = ndgrid(THETA2, THETA3, THETA4); % generate a grid of theta2, theta3 and theta4 values


Pex = cosd(theta1)*(-l4 * sind(theta2+90 + theta3 + theta4-90) + l3 * cosd(theta2+90 + theta3) + l2 * cosd(theta2+90));
Pey = sind(theta1)*(-l4 * sind(theta2+90 + theta3 + theta4-90) + l3 * cosd(theta2+90 + theta3) + l2 * cosd(theta2+90));
Pez = l1 + l4 * cosd(theta2+90 + theta3 + theta4-90)+ l3 * sind(theta2+90 + theta3)+ l2 * sind(theta2+90);

%if Pex=-0.5414   21.2254
% data1 = [Pex(:) Pey(:) Pez(:) theta12(:)]; % create x-y-z-theta1 dataset

%plot3(Pex(:), Pey(:), Pez(:), 'r.');
plot(Pex(:), Pez(:), 'r.');
grid;

axis equal;
  xlabel('X')
  ylabel('Z')
  %zlabel('Z')
  title('X-Z co-ordinates generated from theta2, theta3 and theta4 combinations using FK formula')
 h = rotate3d;
 set(h,'RotateStyle','box','Enable','on');
 axis([-70 70 0  100]);       
 
 
 
  X = Pex(:);
  Y = Pey(:);
  Z = Pez(:);
 [xmax label_maxx] = max(X);
  label_maxx = Z(label_maxx);
  text(xmax ,label_maxx,sprintf('%s   %2.4f \n','.', xmax),'FontSize',8);
  
  [xmin label_minx] = min(X);
  label_minx = Z(label_minx);
  text(xmin ,label_minx,sprintf('%s  %2.4f \n','.', xmin),'FontSize',8);

  [zmax label_maxz] = max(Z);
  text(0, zmax,sprintf('%s   %2.4f \n','.', zmax),'FontSize',8);
  
  [zmin label_minz] = min(abs(Z));
  text(0, zmin ,sprintf('%s %2.4f \n','.', zmin),'FontSize',8);


   
%    save Endeffector count Pex Pey Pez 



 
 
 
 
 
 