% input:  world_pts: (3xN)
%         Rwc
%         twc
%         K
% output: cam_pts: (2xN)
function [cam_pts] = worldpts_to_cam(world_pts, Rwc, twc, K)
  cam_pts_3d = K * (Rwc * world_pts + twc);
  cam_pts = cam_pts_3d(1:2, :) ./ cam_pts_3d(3, :);
end