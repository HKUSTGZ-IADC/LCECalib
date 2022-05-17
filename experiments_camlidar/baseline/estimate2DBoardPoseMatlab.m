function T_cam_world = estimate2DBoardPoseMatlab(imagePoints, worldPoints, camParams)
  [R_world_cam, t_world_cam] = ...
    estimateWorldCameraPose(imagePoints, worldPoints, camParams);
  T_world_cam = [[R_world_cam', t_world_cam']; [0,0,0,1]];
  T_cam_world = inv(T_world_cam);
end