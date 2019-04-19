
function u = controllerNoisy(params, t, obs)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

  % This template code calls the function EKFupdate that you must complete below
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % The rest of this function should ideally be identical to your solution in week 4
  % Student completes this
  %% Initialize states using persistent variables
  persistent T ei_T
  if isempty(T)
      % initialize
      T = t; ei_T = 0;
  end
  
  dt = t - T;
  T = t;
  
  %% Controller
  kp = 38;
  kd = 0.4;
  ki = 800;
  ei_T = ei_T + (0-phi)*dt;
  
  u = kp*(0-phi) + kd*(0-phidot) + ki*(ei_T);
  u = -u;
  
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

  % Student completes this
  %% Create persistent state to perform real-time implementation
  persistent xhat P T
  if isempty(T)
    phi = atan2(z(1), z(2));
    dphi = z(3);
    xhat = [phi; dphi];
    P = 1e3*eye(2);
    T = t;
    xhatOut = xhat;
    return;
  end
  
  dt = t - T;
  T = t;
  
  A = [1 dt; 0 1];
  Q = diag([0.1, 0.1]);
  R = diag([0.002, 0.002, 2]);
  
  %% Predict
  xhat = A*xhat;
  P = A*P*A' + Q;
  
  %% Update
  % Measurement vector: h(xhat)
  h = [sin(xhat(1)); cos(xhat(1)); xhat(2)]; 
  % Calculate the Jacobian - The observation model
  H = [cos(xhat(1)),  0;
       -sin(xhat(1)), 0;
       0,             1]; 
  % Optimal Kalman gain
  K = P * H' * inv(H * P * H' + R); % Note that "inv(H * P * H' + R)" is the Innovation(or pre-fit residual) covatiance
  % Updated state estimate
  xhat = xhat + K * (z - h); % Note that "(z - h)" is the Innovation (or measurement pre-fit residual)
  % Updated estimate covariance
  P = (eye(2)-K*H) * P;
  
  xhatOut = xhat;
end
