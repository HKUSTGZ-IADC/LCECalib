% input: 
% output: err: the total error by summing all absolute planar error terms
%         cnt: number of planar error terms
function [err, cnt] = evaluateTotalPlanarError(T_est, cam_plane_coeff, lidar_pts)
  cam_pts = T_est(1:3, 1:3) * lidar_pts + T_est(1:3, 4);
  d = cam_plane_coeff(1:3)' * cam_pts + cam_plane_coeff(4);
%   err = sum(abs(d));
%   cnt = size(d, 2);
  err = sum(abs(d)) / size(d, 2);
  cnt = 1;
end