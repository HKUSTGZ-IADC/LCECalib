clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

% data_type = 'simu_data_bias';
% data_type = 'simu_data';
data_type = 'real_data';
% data_type = 'fp_data';

visualization_flag = 0;
save_result_flag = 1;
debug_flag = 0;

%% load data and extract features
for data_option = 1:9
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  params = load(fullfile(data_path, 'img/params.mat'));
  load(fullfile(data_path, 'result_lcecalib_qpep.mat'));
  
  all_lidar_board_corners = {};
  for idx = 1:length(all_lidar_board_pts_raw)
      lidar_board_corners = borcorner_ext(all_lidar_board_pts_raw{idx}, params.borW, params.borH, debug_flag);
      all_lidar_board_corners{end + 1} = lidar_board_corners;
  end

  %% Corner-Corner Extrinsic Calibration
  all_iterations = 100;
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
  
  [r_err, t_err] = evaluateTFError(TGt, T_est_best);
  p_err = zeros(2, length(all_cam_board_centers_on_plane));
  for i = 1:length(all_cam_board_plane_coeff)
  [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est_best, ...
    all_cam_board_plane_coeff{i}, ...
    all_lidar_board_pts{i});
  end
  tmp_mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  
  sprintf('r_err: %f, t_err: %f, tmp_mp_err: %f', r_err, t_err, tmp_mp_err)   
  
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




