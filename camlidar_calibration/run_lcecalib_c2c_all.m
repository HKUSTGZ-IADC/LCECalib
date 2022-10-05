clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

data_type = 'simu_data_bias';
% data_type = 'simu_data';
% data_type = 'real_data';
% data_type = 'fp_data';

visualization_flag = 0;
save_result_flag = 1;
debug_flag = 0;

%% load data and extract features
for data_option = 10:10
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
  all_cam_board_plane_coeff = {};
  all_cam_board_centers_on_plane = {};
  all_lidar_board_pts = {};
  all_lidar_board_corners = {};
  all_lidar_board_plane_coeff = {};
  all_img_undist = {};
  all_lidar_pc_array = {};
  for idx = 1:min(length(pcd_list), min(num_data, 60))
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      if ~(contains(img_list(idx).name, '.png') ...
        || contains(img_list(idx).name, '.jpg'))
        continue;
      end      
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      lidar_pc_array = pc_raw.Location()';
      
      %% image: feature extraction
      [img_undist, camParams] = undistort_image(img_raw, K, D);
      [imagePoints, boardSize] = detectCheckerboardPoints(img_undist);
      if debug_flag
        img_undist_checkerboard_pts = img_undist;
        img_undist_checkerboard_pts = insertMarker(...
          img_undist_checkerboard_pts, imagePoints(1:5, :), ...
          'o', 'Color', 'red', 'Size', 10);
        fig = figure; 
        imshow(img_undist_checkerboard_pts);
        close(fig);
      end
      if (boardSize(1) == 0 || boardSize(2) == 0)
        continue;
      end
      if (boardSize(1) ~= numH || boardSize(2) ~= numW)
        continue;
      end
      worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
      worldPoints = [worldPoints, zeros(size(worldPoints,1),1)];
      T = estimate2DBoardPoseMatlab(imagePoints, worldPoints, camParams);
      
      % estimate checkerboard corners
      cam_board_corners = mean(worldPoints, 1)' + ...
        [[borW/2,borH/2,0]', [borW/2,-borH/2,0]',...
        [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
      cam_board_corners = T(1:3, 1:3) * cam_board_corners + T(1:3, 4);
      n = T(1:3,1:3) * [0,0,1]';
      d = -(n' * T(1:3,4));
      cam_board_plane_coeff = [n; d];
      if cam_board_plane_coeff(4) < 0
          cam_board_plane_coeff = -cam_board_plane_coeff;
      end 
      if debug_flag
        worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
        pc_corner_px = worldpts_to_cam(cam_board_corners, eye(3, 3), zeros(3, 1), K);
        pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);       
        figure; 
        subplot(121); imshow(img_raw);
        subplot(122); imshow(img_undist);
        hold on;
        plot(worldPx(1,:),worldPx(2,:),'.r');
        plot(pc_corner_px(1,:), pc_corner_px(2,:),'-ob');
        hold off;
        sgtitle('Projected 3D corners and patterns');
      end
     
      %% lidar: feature extraction      
      [lidar_board_pts, lidar_board_plane_coeff, err] = ...
        boardpts_ext(lidar_pc_array, borW, borH, data_type);
      if (isempty(lidar_board_pts))
        continue;
      end      
      debug_flag = 0;
      lidar_board_corners = borcorner_ext(lidar_board_pts, borW, borH, debug_flag);
      if (debug_flag)
        figure; hold on;
        pcshow(pointCloud(lidar_board_pts(1:3, :)', 'Intensity', lidar_board_pts(4, :)'));
        pcshow(lidar_board_corners', [1,0,0], 'MarkerSize', 1000);
        hold off;
        sgtitle('Fitting planar points');
      end
      
      %% save feature extraction results      
      all_cam_board_corners{end + 1} = cam_board_corners;
      all_cam_board_plane_coeff{end + 1} = cam_board_plane_coeff;     
      all_cam_board_centers_on_plane{end + 1} = mean(cam_board_corners, 2)';
      
      all_lidar_board_pts{end + 1} = lidar_board_pts(1:3, :);
      all_lidar_board_corners{end + 1} = lidar_board_corners;
      all_lidar_board_plane_coeff{end + 1} = lidar_board_plane_coeff;           
      
      all_img_undist{end + 1} = img_undist;
      all_lidar_pc_array{end + 1} = lidar_pc_array;      
  end

  %% Corner-Corner Extrinsic Calibration
  all_iterations = 5;
  all_t_err = zeros(all_iterations, length(all_cam_board_plane_coeff) - 1);
  all_r_err = zeros(all_iterations, length(all_cam_board_plane_coeff) - 1);
  all_mp_err = zeros(all_iterations, length(all_cam_board_plane_coeff) - 1);
  T_est_best = eye(4, 4);
  min_error = 1000;
  start_frame = 5;
  end_frame = length(all_cam_board_plane_coeff);
  for PoseNum = start_frame : end_frame
      r_errs = zeros(1, all_iterations); 
      t_errs = zeros(1, all_iterations);
      mp_errs = zeros(1, all_iterations);
      for iter = 1:all_iterations    
        reidx = randperm(size(all_cam_board_corners, 2));
        sub_cam_corners = {};
        sub_cam_board_coeff = {};
        sub_lidar_corners = {};
        sub_lidar_board_coeff = {};
        T_est = extCalibCornerToCorner(all_cam_board_corners, ...
          all_cam_board_plane_coeff, ...
          all_lidar_board_corners, ...
          all_lidar_board_plane_coeff, ...
          reidx(1:PoseNum));
        
        [r_err, t_err] = evaluateTFError(TGt, T_est);
        r_errs(iter) = r_err;
        t_errs(iter) = t_err;
        
        p_err = zeros(2, length(all_cam_board_centers_on_plane));
        for i = 1:length(all_cam_board_plane_coeff)
          [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est, ...
            all_cam_board_plane_coeff{i}, ...
            all_lidar_board_pts{i});
        end
        mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
        if (mp_err < min_error)
          min_error = mp_err;
          T_est_best = T_est;
        end
        mp_errs(iter) = mp_err;
      end
      all_t_err(:, PoseNum) = t_errs';
      all_r_err(:, PoseNum) = r_errs';
      all_mp_err(:, PoseNum) = mp_errs';
%       sprintf('PoseNum: %d', PoseNum)
  end
  
  %%
  if save_result_flag
    save(fullfile(data_path, 'result_lcecalib_corner_corner.mat'), ...
      'all_t_err', 'all_r_err', 'all_mp_err', ...
      'TGt', 'T_est_best', ...
      'all_cam_board_corners', ...
      'all_cam_board_centers_on_plane', ...
      'all_cam_board_plane_coeff', ...
      'all_lidar_board_pts', ...
      'all_lidar_board_corners', ...
      'all_lidar_board_plane_coeff');

%     save(fullfile(data_path, 'result_lcecalib_corner_corner_sensor_data.mat'), ...
%       'all_t_err', 'all_r_err', 'all_mp_err', ...
%       'T_est_best', ...
%       'all_cam_board_corners', ...
%       'all_cam_board_centers_on_plane', ...      
%       'all_cam_board_plane_coeff', ...
%       'all_lidar_board_pts', ...
%       'all_lidar_board_corners', ...
%       'all_lidar_board_plane_coeff', ...
%       'all_img_undist', ...
%       'all_lidar_pc_array');
  end   
end




