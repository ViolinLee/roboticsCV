% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);

% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin;

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

n = size(scanAngles); % Row number:divisions number of single measurement --> number of rays = 1081

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 500;                       % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);

% System model parameters
noise_sigma = diag([0.03 0.03 0.03]); % System noise covariance
noise_mu = [0 0 0];
radius = 0.02; % Consider robot movement (advanced prediction method can be using kalman filter)

% Map registration parameters
map_threshold_low = mode(mode(map)) - 0.4;
map_threshold_high = mode(mode(map)) + 0.6;

% Correlation 
P_corr = zeros(1, M);

% For visualization
robotGrid = [];

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    W = repmat(1.0/M, [1, M]);    % reset the weights every cycle
    P = repmat(myPose(:,j-1), [1, M]);
    P(1:2, :) = P(1:2, :) + radius * [cos(P(3, 1:M)); -sin(P(3, 1:M))];
    
    % 1) Propagate the particles 
    for m = 1:M
    
    P(:, m) = P(:, m) + mvnrnd(noise_mu, noise_sigma)';
    
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame) 
    %   2-1-1) Obstacle (edges) cells
    x_occ = ranges(:, j) .* cos(scanAngles + P(3, m)) + P(1, m);
    y_occ = -1 * ranges(:, j) .* sin(scanAngles + P(3, m)) + P(2, m);
    occ_id = ceil([x_occ, y_occ] * myResolution + repmat(myOrigin', [n, 1]));
    
    % Delete overange index
    del_occ =  occ_id(:,1)<1 | occ_id(:,2)<1 |  occ_id(:,1) > size(map,2) |  occ_id(:,2) > size(map,1); % T/F index
    del_num = sum(del_occ);
    
    occ_idx = occ_id(:,1); occ_idx(del_occ) = [];
    occ_idy = occ_id(:,2); occ_idy(del_occ) = [];
    occ_id = [occ_idx occ_idy];
    
    
    if del_num>0
        %fprintf('valid range = %d', n-del_num);
        fprintf('delete number = %d\n', del_num);
    end
    
    lidar_occupied = sub2ind(size(map), occ_id(:,2), occ_id(:,1));
    
    %   2-1-2) Free cells
    car_id = ceil([P(1, m), P(2, m)] * myResolution + myOrigin');
    
    lidar_free = [];
    for cnt = 1:(n-del_num)
        %fprintf('gridcell(%d, %d)', occ_id(cnt, 1), occ_id(cnt,2));
        % Compute ID of free cells
        [free_x, free_y] = bresenham(car_id(1),car_id(2),occ_id(cnt,1),occ_id(cnt,2));  
        % Convert to 1d index
        lidar_free = [lidar_free; sub2ind(size(map), free_y, free_x)];
    end
    
    %   2-2) For each particle, calculate the correlation scores of the particles
    %corr_vals = map(occ_id);
    %P_corr(m) = -3 * sum(sum(corr_vals <= map_threshold_low)) + 10 * sum(sum(corr_vals >= map_threshold_high));
    
    P_corr(m) = -5 * sum(map(lidar_occupied) <= map_threshold_low) + 15 * sum(map(lidar_occupied) >= map_threshold_high);
    P_corr(m) = 3 * P_corr(m) + sum(map(lidar_free) <= map_threshold_low) - 5 * sum(map(lidar_free) >= map_threshold_high);
    P_corr = P_corr - min(P_corr);
    end
    
    %   2-3) Update the particle weights
    W = W .* P_corr;
    W = W / sum(W);
    
    %   2-4) Choose the best particle to update the pose
    [max_val, id] = max(W);
    myPose(:, j) = P(:, id);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    %n_eff = sum(W)^2 / sum(W.^2);
    %if (n_eff < resample_threshold * M)
    
    % 4) Visualize the pose on the map as needed
    CAR_ID = ceil([myPose(1, j), myPose(2, j)] * myResolution + myOrigin');
    robotGrid = [robotGrid; CAR_ID];
    
    imagesc(map); hold on;
    plot(robotGrid(:,1),robotGrid(:,2),'r.-','LineWidth',2.5); % indicate robot trajectory
    axis equal;
    lidar_global(:,1) =  (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*myResolution + myOrigin(1);
    lidar_global(:,2) = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*myResolution + myOrigin(2);
    plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
    hold off;
    drawnow; 
end

end

