% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2); % Col number:measurement times = 3701
n = size(scanAngles); % Row number:divisions number of single measurement --> number of rays = 1081
robotGrid = [];

for j = 1:N % for each time, (up to N=3701 times)
    % Vectorization can avoids double for loops, so 'n' is not used
    
    % Find grids hit by the rays (in the gird map coordinate)
    x_occ = ranges(:, j) .* cos(scanAngles + pose(3, j)) + pose(1, j);
    y_occ = -1 * ranges(:, j) .* sin(scanAngles + pose(3, j)) + pose(2, j);
    car_id = ceil([pose(1, j), pose(2, j)] * myResol + myorigin');
    
    % Find occupied-measurement cells and free-measurement cells
    occ_id = ceil([x_occ, y_occ] * myResol + repmat(myorigin', [n, 1]));
    %occ_id = sub2ind(size(myMap), occ_id(:,2),occ_id(:,1)); % convert to 1d index
    myMap(occ_id) = myMap(occ_id) + lo_occ;
    
    free = [];
    for cnt = 1:n
        % Compute ID of free cells
        [free_x, free_y] = bresenham(car_id(1),car_id(2),occ_id(cnt,1),occ_id(cnt,2));  
        % Convert to 1d index
        free = [free;sub2ind(size(myMap), free_y, free_x)];
        
        myMap(occ_id(cnt,2),occ_id(cnt,1)) = myMap(occ_id(cnt,2),occ_id(cnt,1)) + lo_occ;
    end
    
    %[free_x, free_y] = bresenham(repmat(car_id(1),[n, 1]), repmat(car_id(2),[n, 1]), occ_id(:,1), occ_id(:,2));
    %free = sub2ind(size(myMap),free_y,free_x); % convert to 1d index
    

    % Update the log-odds
    myMap(free) = myMap(free) - lo_free;
    
    % Saturate the log-odd values
    myMap = min(myMap,lo_max);
    myMap = max(myMap,lo_min);
    
    % Visualize the map as needed
    robotGrid = [robotGrid; car_id];
    imagesc(myMap); hold on;
    plot(robotGrid(:,1),robotGrid(:,2),'r.-','LineWidth',2.5); % indicate start point
    axis equal;
    lidar_global(:,1) =  (ranges(:,j).*cos(scanAngles + pose(3,j)) + pose(1,j))*myResol + myorigin(1);
    lidar_global(:,2) = (-ranges(:,j).*sin(scanAngles + pose(3,j)) + pose(2,j))*myResol + myorigin(2);
    plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 
    hold off;
    drawnow;    

end

end

