function [err, cnt] = evaluatePlanarError(T_est, cam_plane_coeff, lidar_pts)
  cam_pts = T_est(1:3, 1:3) * lidar_pts + T_est(1:3, 4);
  d = cam_plane_coeff(1:3)' * cam_pts + cam_plane_coeff(4);
  err = sum(abs(d));
  cnt = size(d, 2);
end