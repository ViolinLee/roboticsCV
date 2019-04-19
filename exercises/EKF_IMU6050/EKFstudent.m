
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  % Note from LeeChan: Each row of output, xhat(:,i) must contain [phi, phidot] where phi is the roll angle in degrees, and phidot is the angular velocity in deg/s.
  xhat = zeros(2,length(t)); % Posteriori state estimate vector

  % Student completes this
  P = 1e3 * eye(2);  % Posteriori error covariance
  Q = diag([0.1, 0.1]); % The covariance of the process noise
  R = diag([0.002, 0.002, 2]); % The covariance of the observation noise
  
  for k=2:length(t)
      dt = t(k) - t(k-1);
      A = [1 dt; 0 1]; % The state transition model which is applied to the previous state
      
     %% Predict
      xhat(:,k) = A*xhat(:,k-1);
      P = A*P*A' + Q;
      
     %% Update
      % Measurement vector: h(xhat(:,k-1))
      h = [sind(xhat(1,k)); cosd(xhat(1,k)); xhat(2,k)]; 
      % Calculate the Jacobian - The observation model
      H = [cosd(xhat(1,k))*(pi/180),  0;
           -sind(xhat(1,k))*(pi/180), 0;
           0,                         1]; 
      % Optimal Kalman gain
      K = P * H' * inv(H * P * H' + R); % Note that "inv(H * P * H' + R)" is the Innovation(or pre-fit residual) covatiance
      % Updated state estimate
      xhat(:,k) = xhat(:,k) + K * (z(:,k) - h); % Note that "(z(:,k) - h)" is the Innovation (or measurement pre-fit residual)
      % Updated estimate covariance
      P = (eye(2)-K*H) * P;
end
