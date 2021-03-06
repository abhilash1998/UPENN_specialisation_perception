function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
if H(3,3)<0
    H=-H;
end
%R1=(normalize(K\H(:,1)));
%R2=(normalize((K)\H(:,2)));
%R3=cross(R1,R2);
%R=[R1 ,R2 ,R3];

[u, s ,v]=svd([H(:,1),H(:,2),cross(H(:,1),H(:,2))]);
R=u*diag([1 1 det(u*v')])*v';
t=H(:,3)/norm(H(:,1));

% YOUR CODE HERE: Project the points using the pose
F=[R t];
n = size(render_points, 1);
proj_points=(K*(R*render_points' + repmat(t, 1, n)))';

proj_points = proj_points./repmat(proj_points(:,3), 1, 3);

proj_points = proj_points(:,1:2);
end
