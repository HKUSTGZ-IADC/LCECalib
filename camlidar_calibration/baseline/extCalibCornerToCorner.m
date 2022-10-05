function TOptm = extCalibCornerToCorner(all_cam_board_corners, ...
  all_cam_board_plane_coeff, ...
  all_lidar_board_corners, ...
  all_lidar_board_plane_coeff, ...
  reidx)
  
  sub_cam_corners = {};
  sub_cam_board_coeff = {};
  sub_lidar_corners = {};
  sub_lidar_board_coeff = {};
  for idx = 1:length(reidx)
    sub_cam_corners{end+1} = all_cam_board_corners{reidx(idx)};
    sub_cam_board_coeff{end+1} = all_cam_board_plane_coeff{reidx(idx)};
    sub_lidar_corners{end+1} = all_lidar_board_corners{reidx(idx)};
    sub_lidar_board_coeff{end+1} = all_lidar_board_plane_coeff{reidx(idx)};
  end
  TInit = plane_init(sub_lidar_board_coeff, sub_cam_board_coeff, ...
    sub_lidar_corners,sub_cam_corners);
  TOptm = corner_optm(sub_cam_corners, sub_lidar_corners, TInit);
end