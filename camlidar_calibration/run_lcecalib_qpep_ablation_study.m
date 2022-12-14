visualization_flag = 0;
debug_flag = 0;
save_result_flag = 1;
plot_result_flag = 0;

%% load data and extract features
for data_option = 1:9
  close all;
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  if (~exist(data_path)) 
    continue;
  end 
  
  params = load(fullfile(data_path, 'img/params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  use_edge_flag = params.use_edge_flag;
  use_planar_flag = params.use_planar_flag;  
  edge_weight = params.edge_weight;
  planar_weight = params.planar_weight;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  
  pcd_raw_list = dir(fullfile(data_path, 'raw_pcd'));
  pcd_raw_list = pcd_raw_list(3:end);
  save('data_tmp/tmp_lcecalib_dataset.mat', ...
    'data_path', 'data_type', 'data_option', ...
    'img_list', 'pcd_list', 'pcd_raw_list', ...
    'borW', 'borH', 'numW', 'numH', 'pattern_size', ...
    'K', 'D', 'TGt', 'num_data', ...
    'use_edge_flag', 'use_planar_flag', ...
    'edge_weight', 'planar_weight', ...
    'visualization_flag', 'debug_flag', ...
    'save_result_flag', 'plot_result_flag');
  run_lcecalib_fe();

  %% QPEP-pTop based extrinsic calibration
  all_iterations = 100;
  edge_iterations = 3;
  start_frame = 1;
  end_frame = num_data;
  run_lcecalib_opt(all_iterations, edge_iterations, start_frame, end_frame); 
  load('data_tmp/tmp_lcecalib_opt.mat');
  
  %% Save calibration results
  % output result: result_lcecalib_qpep_sensor_data_\
  %                 (flg_point_projection)_\
  %                 (flg_outlier_rejection)_\
  %                 (flg_use_iter_num)_\
  %                 (flg_use_ptp)_\
  %                 (flg_use_ptl).mat  
  if save_result_flag
    resultname = sprintf('result_lcecalib_qpep_%d_%d_%d_%d_%d.mat', ...
      flg_point_projection, flg_outlier_rejection, flg_use_iter_num, ...
      flg_use_ptp, flg_use_ptl);
    save(fullfile(data_path, resultname), ...
      'data_type', 'data_option', ...
      'all_t_err', 'all_r_err', 'all_mp_err', 'all_me_err', ...
      'select_t_err', 'select_r_err', 'select_mp_err', 'select_me_err', ...
      'all_eulerx', 'all_eulery', 'all_eulerz', ...
      'all_tx', 'all_ty', 'all_tz', ...
      'TGt', 'T_ini_best', 'T_est_best', ...
      'start_frame', ...
      'all_cam_board_corners', ...
      'all_cam_board_centers_on_plane', ...
      'all_cam_board_plane_coeff', ...
      'all_cam_board_plane_coeff_cov', ...
      'all_lidar_board_pts_raw', ...
      'all_lidar_board_pts', ...
      'all_lidar_board_edge_pts', ...
      'all_lidar_board_plane_coeff');

    resultname = sprintf('result_lcecalib_qpep_sensor_data_%d_%d_%d_%d_%d.mat', ...
      flg_point_projection, flg_outlier_rejection, flg_use_iter_num, ...
      flg_use_ptp, flg_use_ptl);
    save(fullfile(data_path, resultname), ...
      'data_type', 'data_option', ...  
      'all_t_err', 'all_r_err', 'all_mp_err', 'all_me_err', ...
      'select_t_err', 'select_r_err', 'select_mp_err', 'select_me_err', ...
      'all_eulerx', 'all_eulery', 'all_eulerz', ...
      'all_tx', 'all_ty', 'all_tz', ...
      'TGt', 'T_ini_best', 'T_est_best', ...
      'start_frame', ...
      'all_cam_board_corners', ...
      'all_cam_board_centers_on_plane', ...
      'all_cam_board_plane_coeff', ...
      'all_cam_board_plane_coeff_cov', ...
      'all_lidar_board_pts_raw', ...
      'all_lidar_board_pts', ...
      'all_lidar_board_edge_pts', ...
      'all_lidar_board_plane_coeff', ...
      'all_img_undist', ...
      'all_lidar_pc_array', ...
      'all_lidar_pc_array_raw');
  end
  
  %% Plot results
  plot_result_flag = 1; 
  reidx = randperm(length(all_cam_board_plane_coeff));
  idx = reidx(1);
  if plot_result_flag
    figure; 
    subplot(311); plot(all_r_err(start_frame:end), 'r-o', ...
      'MarkerSize', 15, 'LineWidth', 3);
    ylabel("Rotation Error [deg]");
    grid on;
    ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3;
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 1.5, 'YScale', 'log');
    box on;
    
    subplot(312); plot(all_t_err(start_frame:end), 'r-o', ...
      'MarkerSize', 15, 'LineWidth', 3);
%     xlabel("Number of Poses"); 
    ylabel("Translation Error [m]");
    grid on;
    ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3;    
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 1.5, 'YScale', 'log');
    box on;
    
    subplot(313); plot(all_mp_err(start_frame:end), 'b-d', ...
      'MarkerSize', 15, 'LineWidth', 3);
    xlabel("Number of Poses"); ylabel("Planar Error [m]");
    grid on;
    ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3;
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 1.5, 'YScale', 'log');
    box on;    

    sgtitle('Mean Rotation, Translation, and Planar Error', 'FontSize', 30, ...
      'FontName', 'Times', 'FontWeight', 'normal');
  end
  
  if plot_result_flag
    figure;
    subplot(121);
    imshow(projectPointOnImage(T_est_best, K, ...
      all_lidar_pc_array_raw{idx}, all_img_undist{idx}));
    title('Projected points with Test', 'FontSize', 25);
    subplot(122);
    imshow(projectPointOnImage(TGt, K, ...
      all_lidar_pc_array_raw{idx}, all_img_undist{idx}));
    title('Projected points with TGt', 'FontSize', 25);
%     cloud_rgb = colorizePointFromImage(T_est_best, K, ...
%       all_lidar_pc_array_raw{reidx(1)}, all_img_undist{reidx(1)});    
%     pcwrite(cloud_rgb, '/tmp/cloud_rgb.pcd');
  end
  plot_result_flag = 0;
  
  %% tmp
  plot_result_flag = 1;
  reidx = randperm(length(all_cam_board_plane_coeff));
  if plot_result_flag
    figure; hold on;
    for idx = 1:8
      lbpts = all_lidar_board_pts{idx};
      lepts = all_lidar_board_edge_pts{idx};
      cbcorner = all_cam_board_corners{idx};
      [cbedge, cbedge_dir] = generateBoardPtsFromCorner(cbcorner);
      lbpts_cam = T_est_best(1:3, 1:3) * lbpts + T_est_best(1:3, 4);  % 3xN
      lepts_cam = T_est_best(1:3, 1:3) * lepts + T_est_best(1:3, 4);  % 3xN
      plot3(cbedge(1, :), cbedge(2, :), cbedge(3, :), 'g.', 'MarkerSize', 6); 
      plot3(lbpts_cam(1, :), lbpts_cam(2, :), lbpts_cam(3, :), 'r*', 'MarkerSize', 2.5);
      plot3(lepts_cam(1, :), lepts_cam(2, :), lepts_cam(3, :), 'bo', 'MarkerSize', 12, 'LineWidth', 2);
%       legend('cam edge pts', 'lidar board pts', 'lidar edge pts');
    end
    hold off; axis equal; view(40, 10);
    legend({'Camera Edge Points', 'LiDAR Planar Points', 'LiDAR Edge Points'}, 'Location', 'northeast', 'FontSize', 20);
    grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
    set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2);
    xlabel("X [m]"); ylabel("Y [m]"); zlabel("Z [m]");
  end  
  plot_result_flag = 0;
end



