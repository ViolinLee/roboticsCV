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

N = size(pose,2); % Col number:measurement times
n = size(scanAngles); % Row number:divisions number of single measurement

for j = 1:N % for each time, (up to N=3701 times)
    % Vectorization can avoids double for loops, so 'n' is not used
    
    % Find grids hit by the rays (in the gird map coordinate)
    x_occ = ranges(:, j) .* cos(scanAngles + pose(3, j)) + pose(1, j);
    y_occ = -1 * ranges(:, j) .* sin(scanAngles + pose(3, j)) + pose(2, j);
    car_id = [ceil(pose(1, j) * myResol), ceil(pose(2, j) * myResol)] + myorigin;
    
    % Find occupied-measurement cells and free-measurement cells
    occ_id = [ceil(x_occ * myResol), ceil(y_occ * myResol)] + myorigin;
    occ_id_del = occ_id(:,2)<1 | occ_id(:,1)<1 | occ_id(:,2)>param.size(1) | occ_id(:,1)>param.size(2); % Crop Map
    occ_id(occ_id_del) = [];

    % Update the log-odds
    % Saturate the log-odd values
    % Visualize the map as needed
    

end

end

