clc; clear; close all;

add_path_lcecalib;

format short

% data_type = 'simu_data_bias';
% data_type = 'simu_data';
data_type = 'real_data';
% data_type = 'fp_data';

visualization_flag = 0;
debug_flag = 0;
save_result_flag = 0;
plot_result_flag = 0;

for data_option = 4:4
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'img/params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  

  all_cam_board_corners = {};
  all_lidar_board_corners = {};
  all_lidar_board_plane_coeff = {};
  all_cam_board_plane_coeff = {};
  all_img_undist = {};
  all_lidar_array = {};
  for idx = 1:size(pcd_list, 1)
      img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      pc_array = pc_raw.Location()';

      % extract img features
      [img_undist, camParams] = undistort_image(img_raw, K, D);
      [im_corners, cam_plane_coeff, pro_err] = imgbor_ext(img_raw, K, D, pattern_size, borW, borH, visualization_flag);
      if (isempty(im_corners))
        continue;
      end
      img_file

      % extract point cloud features
      [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH, data_type);  %% extract board points
      if (isempty(pts_bor))
        continue;
      end
      pc_corners = borcorner_ext(pts_bor, borW, borH, visualization_flag);  %% extract board corners
      pcd_file

      % save data
      all_lidar_board_corners{end+1} = pc_corners;
      all_lidar_board_plane_coeff{end+1} = bor_coeff;
      all_cam_board_corners{end+1} = im_corners;
      all_cam_board_plane_coeff{end+1} = cam_plane_coeff;
      all_img_undist{end+1} = img_undist;
      all_lidar_array{end+1} = pc_array;
  end
  
  %% Corner-Corner Extrinsic Calibration
  T_est = extCalibCornerToCorner(...
    all_cam_board_corners(1:end), ...
    all_cam_board_plane_coeff(1:end), ...
    all_lidar_board_corners(1:end), ...
    all_lidar_board_plane_coeff(1:end), ...
    1:length(all_cam_board_plane_coeff(1:end)));
  
  %% Visualize GT results
  figure; 
  hold on;
  pl = zeros(1, 2);
  for i = 1:length(all_cam_board_corners)
    img_list(i).name
    % project 3D camera corners
    cam_board_corners = all_cam_board_corners{i};
    project_cam_board_corners = zeros(2, 0);
    for j = 1:size(cam_board_corners, 2)
      project_cam_board_corners(:, end + 1) = ...
        worldpts_to_cam(cam_board_corners(:, j), eye(3, 3), zeros(3, 1), K);
    end
    % project 3D lidar corners
    lidar_board_corners = all_lidar_board_corners{i};
    project_lidar_board_corners = zeros(2, 0);
    for j = 1:size(lidar_board_corners, 2)
      project_lidar_board_corners(:, end + 1) = ...
        worldpts_to_cam(lidar_board_corners(:, j), T_est(1:3, 1:3), T_est(1:3, 4), K);
    end
    if (i == 1)
%       hold off;
      pl(1) = plot(project_cam_board_corners(1, :), project_cam_board_corners(2, :), 'bo', 'MarkerSize', 10);
%       hold on;
      pl(2) = plot(project_lidar_board_corners(1, :), project_lidar_board_corners(2, :), 'r*', 'MarkerSize', 10);
%       hold off;
    else
%       hold off;
      plot(project_cam_board_corners(1, :), project_cam_board_corners(2, :), 'bo', 'MarkerSize', 10);
%       hold on;
      plot(project_lidar_board_corners(1, :), project_lidar_board_corners(2, :), 'r*', 'MarkerSize', 10);      
%       hold off;
    end
  end
  legend(pl, 'Camera Pts', 'Projected Pts', 'Location', 'northeastOutside', 'FontSize', 25);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2);
  
  %%
  figure; 
  for idx = 1:length(all_lidar_array)
%   for idx = 1:10
    figure;
    imshow(projectPointOnImage(T_est, K, all_lidar_array{idx}, all_img_undist{idx}));
  end
end



























