clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

data_type = 'real_data';
% data_type = 'simu_data';
% data_type = 'fp_data';

visualization_flag = 0;
debug_flag = 0;
save_result_flag = 1;
plot_result_flag = 0;

%% load data and extract features
for data_option = 2:3
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'img/params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  use_edge_flag = params.use_edge_flag;
  use_planar_flag = params.use_planar_flag;  

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  
  save('tmp_lcecalib_dataset.mat', ...
    'data_path', 'data_type', ...
    'img_list', 'pcd_list', ...
    'borW', 'borH', 'numW', 'numH', 'pattern_size', ...
    'K', 'D', 'TGt', 'num_data', ...
    'use_edge_flag', 'use_planar_flag', ...
    'visualization_flag', 'debug_flag', ...
    'save_result_flag', 'plot_result_flag');
  run_lcecalib_fe();
  
  %% QPEP-pTop based extrinsic calibration
  all_iterations = 10;
  edge_iterations = 3;
  run_lcecalib_opt(all_iterations, edge_iterations); 
  load('tmp_lcecalib_opt.mat');
  
  %%
  if save_result_flag
    save(fullfile(data_path, 'result_lcecalib_qpep.mat'), ...
      'all_t_err', 'all_r_err', ...
      'all_eulerx', 'all_eulery', 'all_eulerz', ...
      'all_tx', 'all_ty', 'all_tz', ...
      'T_est_best', ...
      'all_cam_board_corners', ...
      'all_cam_board_centers', ...
      'all_cam_board_centers_on_plane', ...
      'all_cam_board_plane_coeff', ...
      'all_cam_board_plane_coeff_cov', ...
      'all_lidar_board_pts_raw', ...
      'all_lidar_board_pts', ...
      'all_lidar_board_edge_pts');

    save(fullfile(data_path, 'result_lcecalib_qpep_sensor_data.mat'), ...
      'all_t_err', 'all_r_err', ...
      'all_eulerx', 'all_eulery', 'all_eulerz', ...
      'all_tx', 'all_ty', 'all_tz', ...
      'T_est_best', ...
      'all_cam_board_corners', ...
      'all_cam_board_centers', ...
      'all_cam_board_centers_on_plane', ...
      'all_cam_board_plane_coeff', ...
      'all_cam_board_plane_coeff_cov', ...
      'all_lidar_board_pts_raw', ...
      'all_lidar_board_pts', ...
      'all_lidar_board_edge_pts', ...
      'all_img_undist', ...
      'all_lidar_pc_array');
  end
  
  %% Plot results
  plot_result_flag = 1;
  if plot_result_flag
    figure; 
    subplot(211); boxplot(all_r_err(:, 5:end));
    xlabel("Number of Poses"); ylabel("Rotation Error [deg]");
    grid on;
    ax = gca;
    ax.GridLineStyle = '--';
    ax.GridAlpha = 0.3;
    set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
    box on;
    
    subplot(212); boxplot(all_t_err(:, 5:end));
    xlabel("Number of Poses"); ylabel("Translation Error [m]");
    grid on;
    ax = gca;
    ax.GridLineStyle = '--';
    ax.GridAlpha = 0.3;
    set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
    box on;
    
    sgtitle('Mean and Median Rotation and Trnslation Error', 'FontSize', 25, ...
      'FontName', 'Times', 'FontWeight', 'normal');
  end
  plot_result_flag = 0;
end




