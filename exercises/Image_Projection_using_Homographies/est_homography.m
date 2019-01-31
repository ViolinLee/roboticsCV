function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
H = zeros(9,1);
A = zeros(8,9);

for m=1:4
   A(2*m-1,1) = -video_pts(m,1);
   A(2*m-1,2) = -video_pts(m,2);
   A(2*m-1,3) = -1;
   A(2*m-1,4) = 0;
   A(2*m-1,5) = 0;
   A(2*m-1,6) = 0;
   A(2*m-1,7) = video_pts(m,1)*logo_pts(m,1);
   A(2*m-1,8) = video_pts(m,2)*logo_pts(m,1);
   A(2*m-1,9) = logo_pts(m,1);
   A(2*m,1) = 0;
   A(2*m,2) = 0;
   A(2*m,3) = 0;
   A(2*m,4) = -video_pts(m,1);
   A(2*m,5) = -video_pts(m,2);
   A(2*m,6) = -1;
   A(2*m,7) = video_pts(m,1)*logo_pts(m,2);
   A(2*m,8) = video_pts(m,2)*logo_pts(m,2);
   A(2*m,9) = logo_pts(m,2);
end    

[U, S, V] = svd(A);

H = V(:,9);
H = reshape(H,3,3);
H = H';

end

