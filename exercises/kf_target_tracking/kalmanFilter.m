function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% Naive estimate
%     % As an example, here is a Naive estimate without a Kalman filter
%     % You should replace this code
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
%     % State is a four dimensional element
%     state = [x, y, vx, vy];
    %% TODO: Add Kalman filter updates
    dt = t - previous_t; % Time interval
    
    % PREDICT
    A = [1 0 dt 0; % Transform matrix A
         0 1 0 dt;
         0 0 1  0;
         0 0 0  1];
     
    C = [1 0 0 0; % Observation matrix C
         0 1 0 0];
     
    z_t = [x, y]'; % Measurement data
    
    % Q = 0.005 * eye(4);
    Q = [dt^2  0        0       0; % System (Motion) noise covariance
         0        dt^2  0       0;
         0        0        1       0;
         0        0        0       1];
           
    R = 0.005 * eye(2); % Measurement noise covariance
    
    P = A * param.P * transpose(A) + Q; % Prior estimation covariance
    
    % UPDATE
    K = P * transpose(C) * inv(R + C * P * transpose(C)); % Kalman gain
    state = (A * (state') + K * (z_t - C * A * (state')))'; % Posterior state estimation
    param.P = P - K * C * P; % Posterior estimation covariance
    
    % Predict 330ms into the future
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
end
