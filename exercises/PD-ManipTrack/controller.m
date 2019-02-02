
function u = controller(params, t, X)
  kp = 1000;
  kd = 4;
  
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  p = [0,0]';
  
  l = params.l;
  theta1 = X(1); theta2 = X(2);
  dtheta1 = X(3); dtheta2 = X(4);
  
  p(1) = l*cos(theta1) + l*cos(theta1+theta2);
  p(2) = l*sin(theta1) + l*sin(theta1+theta2);
  
  % 2. Let e = p - params.traj(t) be the task-space error
  e = p - params.traj(t);
  
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  J = [-l*sin(theta1) - l*sin(theta1+theta2) , -l*sin(theta1+theta2);
        l*cos(theta1) + l*cos(theta1+theta2) ,  l*cos(theta1+theta2)];
  
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  u = -kp * J' * e - kd * [dtheta1; dtheta2];
end

