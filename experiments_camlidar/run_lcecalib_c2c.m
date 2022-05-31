clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

% data_type = 'simu_data_bias';
% data_type = 'simu_data';
data_type = 'real_data';
% data_type = 'fp_data';

visualization_flag = 0;
debug_flag = 0;
save_result_flag = 1;

%% load data and extract features
for data_option = 1:3
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  if (~exist(data_path)) 
    continue;
  end
  
  params = load(fullfile(data_path, 'img/params.mat'));
  load(fullfile(data_path, 'result_lcecalib_qpep.mat'));
  
  all_lidar_board_corners = {};
  for idx = 1:length(all_lidar_board_pts_raw)
      lidar_board_corners = borcorner_ext(all_lidar_board_pts_raw{idx}, params.borW, params.borH, debug_flag);
      all_lidar_board_corners{end + 1} = lidar_board_corners;
  end

  %% Corner-Corner Extrinsic Calibration
  all_iterations = 100;
  start_frame = 1;
  end_frame = length(all_cam_board_plane_coeff);
  T_est_best = eye(4, 4);
  min_error = 1000;
  
  all_t_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_r_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_mp_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_me_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   select_t_err = zeros(1, length(all_cam_board_plane_coeff));
%   select_r_err = zeros(1, length(all_cam_board_plane_coeff));
%   select_mp_err = zeros(1, length(all_cam_board_plane_coeff));  
%   all_eulerx = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   all_eulery = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   all_eulerz = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   all_tx = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   all_ty = zeros(all_iterations, length(all_cam_board_plane_coeff));
%   all_tz = zeros(all_iterations, length(all_cam_board_plane_coeff));
  
  for frame_num = start_frame : end_frame
    r_errs = zeros(1, all_iterations); 
    t_errs = zeros(1, all_iterations);
    mp_errs = zeros(1, all_iterations);
    me_errs = zeros(1, all_iterations);
%     all_eulers = zeros(3, all_iterations);
%     all_tsl = zeros(3, all_iterations);  

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
        reidx(1:frame_num));

      % Result Evaluation and Visualization
%       all_eulers(:, iter) = rotm2eul(T_est(1:3, 1:3), 'ZYX');
%       all_tsl(:, iter) = T_est(1:3, 4);
      
      [r_err, t_err] = evaluateTFError(TGt, T_est);
      r_errs(iter) = r_err;
      t_errs(iter) = t_err;
      p_err = zeros(4, length(all_cam_board_centers_on_plane));
      for i = 1:length(all_cam_board_plane_coeff)
        [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est, ...
          all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
        [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(T_est, ...
          all_cam_board_corners{i}, all_lidar_board_edge_pts{i});               
      end
      mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
      me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean planar error
      if (mp_err + me_err < min_error)
        min_error = mp_err + me_err;
        T_est_best = T_est;
      end          
      mp_errs(iter) = mp_err;
      me_errs(iter) = me_err;
    end
    all_t_err(:, frame_num) = t_errs';
    all_r_err(:, frame_num) = r_errs';
    all_mp_err(:, frame_num) = mp_errs';
    all_me_err(:, frame_num) = me_errs';
    
%     all_eulerz(:, frame_num) = all_eulers(1, :) / pi * 180;
%     all_eulery(:, frame_num) = all_eulers(2, :) / pi * 180;
%     all_eulerx(:, frame_num) = all_eulers(3, :) / pi * 180;
%     all_tx(:, frame_num) = all_tsl(1, :);
%     all_ty(:, frame_num) = all_tsl(2, :);
%     all_tz(:, frame_num) = all_tsl(3, :);
%     quat = quaternion(all_eulers' / pi * 180, 'eulerd', 'ZYX', 'frame');
%     quatAverage = meanrot(quat);
%     tslAverage = mean(all_tsl, 2);
%     T_est = eye(4, 4);
%     T_est(1:3, 1:3) = quat2rotm(quatAverage);
%     T_est(1:3, 4) = tslAverage;
%     
%     % compute tf error and mean planar error    
%     [r_err, t_err] = evaluateTFError(TGt, T_est);
%     p_err = zeros(4, length(all_cam_board_centers_on_plane));
%     for i = 1:length(all_cam_board_centers_on_plane)
%       [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est, ...
%         all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
%       [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(T_est, ...
%         all_cam_board_corners{i}, all_lidar_board_edge_pts{i});
%     end
%     mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
%     me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean edge error
%     if (mp_err + me_err < min_error)
%       min_error = mp_err + me_err;
%       T_est_best = T_est;
%     end    
%     select_r_err(1, frame_num) = r_err;
%     select_t_err(1, frame_num) = t_err;
%     select_mp_err(1, frame_num) = mp_err;
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
      'all_t_err', 'all_r_err', 'all_mp_err', 'all_me_err', ...
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




