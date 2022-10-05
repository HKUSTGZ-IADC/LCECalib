clc; clear; close all;

add_path_lcecalib;
load('color_list.mat');

format short

%%
data_type = 'simu_data_bias';
% data_type = 'simu_data';
% data_type = 'real_data';
% data_type = 'fp_data';

method = {'Ours', 'Zhou-Matlab'};
result_mat = {'result_lcecalib_qpep.mat', 'result_baseline_zhou.mat'};
% method = {'Ours', 'C2C', 'Zhou-Matlab'};
% result_mat = {'result_lcecalib_qpep.mat', 'result_lcecalib_corner_corner.mat', 'result_baseline_zhou.mat'};
% method = {'Ours', 'C2C'};
% result_mat = {'result_lcecalib_qpep.mat', 'result_lcecalib_corner_corner.mat'};
label_title = {'Simulated Data', 'Real-World Data'};

%% load data and extract features
r_err_noise = zeros(length(result_mat), 10);
t_err_noise = zeros(length(result_mat), 10);
mp_err_noise = zeros(length(result_mat), 10);
me_err_noise = zeros(length(result_mat), 10);
for data_option = 1:10
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  result_lcecalib_qpep = load(fullfile(data_path, 'result_lcecalib_qpep.mat'));  
  params = load(fullfile(data_path, 'img', 'params.mat'));  
  TGt = params.TGt;
  
  for j = 1:length(result_mat)
    result_data = load(fullfile(data_path, result_mat{j}));  
    [r_err_noise(j, data_option), t_err_noise(j, data_option), ...
     mp_err_noise(j, data_option), me_err_noise(j, data_option)] = evaluateTFPlaneEdgeError(...
      TGt, ...
      result_data.T_est_best, ...
      result_lcecalib_qpep.all_cam_board_centers_on_plane, ...
      result_lcecalib_qpep.all_cam_board_plane_coeff, ...
      result_lcecalib_qpep.all_lidar_board_pts, ...
      result_lcecalib_qpep.all_cam_board_corners, ...
      result_lcecalib_qpep.all_lidar_board_edge_pts);
    sprintf('%d: %s: r_err: %f, t_err: %f, mp_err: %f, me_err: %f, total_err: %f', ...
      data_option, method{j}, ...
      r_err_noise(j, data_option), t_err_noise(j, data_option), ...
      mp_err_noise(j, data_option), me_err_noise(j, data_option), ...
      mp_err_noise(j, data_option)+me_err_noise(j, data_option))   
  end 
end

%% Evaluate feature extraction performance
for t = 1:1
if strcmp(data_type, 'simu_data_bias')
  n_err_noise = zeros(length(result_mat), 10);
  result = load('data/simu_data_bias/simu_data_bias_1/result_lcecalib_qpep.mat');  
  gt_plane_coeff = result.all_lidar_board_plane_coeff;
  for i = 1:length(gt_plane_coeff)
    tmp_plane_coeff = gt_plane_coeff{i};
    if (tmp_plane_coeff(1) < 0)
      tmp_plane_coeff(1:3) = tmp_plane_coeff(1:3) * (-1);
      gt_plane_coeff{i} = tmp_plane_coeff;
    end
  end
  
  for data_option = 1:10
    if (~strcmp(data_type, 'simu_data_bias'))
      continue;
    end
    data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option))); 
    for j = 1:length(result_mat)
      result_data = load(fullfile(data_path, result_mat{j}));  
      est_plane_coeff = result_data.all_lidar_board_plane_coeff;
      n_err = 0;
      cnt = 0;
  
      for i = 1:length(est_plane_coeff)
        tmp_plane_coeff = est_plane_coeff{i};
        if (isempty(tmp_plane_coeff))
          continue;
        end
        if (tmp_plane_coeff(1) < 0)
          tmp_plane_coeff(1:3) = tmp_plane_coeff(1:3) * (-1);
          est_plane_coeff{i} = tmp_plane_coeff;
        end
        tmp_gt_plane_coeff = gt_plane_coeff{i};
        tmp_est_plane_coeff = est_plane_coeff{i};
  %       n_err = n_err + norm(tmp_gt_plane_coeff(1:3)-tmp_est_plane_coeff(1:3));
        angle = acos(tmp_gt_plane_coeff(1:3)' * tmp_est_plane_coeff(1:3) ...
          / (norm(tmp_gt_plane_coeff(1:3)) * norm(tmp_est_plane_coeff(1:3)))) / pi * 180.0;
        n_err = n_err + abs(angle);
        cnt = cnt + 1;
      end
      n_err = n_err / cnt;
      sprintf('%d, %s, n_err: %f', ...
        data_option, method{j}, ...
        n_err)
      n_err_noise(j, data_option) = n_err;
    end 
  end
end
end

%% Plot calibration error results
for t = 1:1
if strcmp(data_type, 'simu_data_bias')
  hf = figure; 
  subplot(411); hold on;
  plot(r_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(r_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(r_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  legend(method, 'Location', 'northeastOutside', 'FontSize', 26);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log', 'xticklabel', []);
  ylabel("Rotation [deg]", 'FontSize', 25);
  
  subplot(412); hold on;
  plot(t_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(t_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(t_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  legend(method, 'Location', 'northeastOutside', 'FontSize', 26);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log', 'xticklabel', []);
  ylabel("Translation [m]", 'FontSize', 25);  
  
  subplot(413); hold on;
  plot(mp_err_noise(1, :)+me_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(mp_err_noise(2, :)+me_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(mp_err_noise(3, :)+me_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  % xlabel("Noise Level");  
  legend(method, 'Location', 'northeastOutside', 'FontSize', 26);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log', 'xticklabel', []);
  ylabel("MGE [m]", 'FontSize', 25);
  sgtitle(sprintf('Calibration Error On %s', label_title{1}), 'FontSize', 30, 'FontName', 'Times', 'FontWeight', 'normal');
  
  subplot(414); hold on;
  plot(n_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(n_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
  legend({method{1}, method{2}}, 'Location', 'northeastOutside', 'FontSize', 26);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 28, 'LineWidth', 2);
  xlabel("Noise Level", 'FontSize', 30);  
  ylabel("Normal [deg]", 'FontSize', 25);
  % title(sprintf('Normal Vector Error On %s', label_title{1}), 'FontSize', 30, 'FontName', 'Times', 'FontWeight', 'normal');
end
end

for t = 1:1
if strcmp(data_type, 'real_data')
  hf = figure; 
  subplot(311); hold on;
  plot(r_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(r_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(r_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  legend(method, 'Location', 'northeastOutside', 'FontSize', 20);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log');
  ylabel("Rot. Error [deg]", 'FontSize', 20);
  
  subplot(312); hold on;
  plot(t_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(t_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(t_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  legend(method, 'Location', 'northeastOutside', 'FontSize', 20);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log');
  ylabel("Trans. Error [m]", 'FontSize', 20);  
  
  subplot(313); hold on;
  plot(mp_err_noise(1, :)+me_err_noise(1, :), 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  plot(mp_err_noise(2, :)+me_err_noise(2, :), 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(mp_err_noise(3, :)+me_err_noise(3, :), 'Color', color_list(3, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  xlabel("Dataset");  
  legend(method, 'Location', 'northeastOutside', 'FontSize', 20);
  grid on; ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3; box on;
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2, 'YScale', 'log');
  ylabel("MGE [m]", 'FontSize', 20);
  
  sgtitle(sprintf('Calibration Error On %s', label_title{2}), 'FontSize', 30, 'FontName', 'Times', 'FontWeight', 'normal');
end
end

%%
filename = fullfile('figure', 'tmech_version2', 'err_simu_data_2_methods'); print(filename, '-depsc'); saveas(hf, strcat(filename, '.fig'));

%% Plot back-projection result
for data_option = 8:9
data_path = sprintf('data/real_data/real_data_%d/', data_option);
params = load(fullfile(data_path, 'img/params.mat'));
result_qpep = load(fullfile(data_path, 'result_lcecalib_qpep_sensor_data.mat'));
result_zhou = load(fullfile(data_path, 'result_baseline_zhou.mat'));

% 16 41 51
img = imread(fullfile(data_path, 'img_before_select/00016.png'));
pcd = pcread(fullfile(data_path, 'pcd_before_select/00016.pcd'));
pcd_raw = pcread(fullfile(data_path, 'raw_pcd_before_select/00016.pcd'));
[img_undist, camParams] = undistort_image(img, params.K, params.D);
[lidar_board_pts, ~, ~] = boardpts_ext(pcd.Location', params.borW, params.borH, data_type);
lidar_board_edge_pts_idx = find_pts_ring_edges(lidar_board_pts);
lidar_board_edge_pts = lidar_board_pts(:, lidar_board_edge_pts_idx);  

p = zeros(2, 1);
figure; imshow(img_undist); hold on;
lidar_pc_array_raw = lidar_board_pts(1:3, :);
cam_pts = zeros(2, 0);
for i = 1:size(lidar_pc_array_raw, 2)
  if lidar_pc_array_raw(1, i) <= 0
    continue;
  end
  cam_pts(:, end+1) = worldpts_to_cam(lidar_pc_array_raw(:, i), result_zhou.T_est_best(1:3,1:3), result_qpep.T_est_best(1:3,4), params.K);
  if (cam_pts(1, end) <= 0 || cam_pts(1, end) > size(img, 2) || ...
      cam_pts(2, end) <= 0 || cam_pts(2, end) > size(img, 1))
    cam_pts(:, end) = [0;0];
    continue;
  end  
end
p(1) = plot(cam_pts(1, :), cam_pts(2, :), 'g*', 'LineStyle', 'none', 'MarkerSize', 3);

lidar_pc_array_raw = pcd_raw.Location';
cam_pts = zeros(2, 0);
for i = 1:size(lidar_pc_array_raw, 2)
  if lidar_pc_array_raw(1, i) <= 0
    continue;
  end
  cam_pts(:, end+1) = worldpts_to_cam(lidar_pc_array_raw(:, i), result_qpep.T_est_best(1:3,1:3), result_qpep.T_est_best(1:3,4), params.K);
  if (cam_pts(1, end) <= 0 || cam_pts(1, end) > size(img, 2) || ...
      cam_pts(2, end) <= 0 || cam_pts(2, end) > size(img, 1))
    cam_pts(:, end) = [0;0];
    continue;
  end
end
p(2) = plot(cam_pts(1, :), cam_pts(2, :), 'r*', 'LineStyle', 'none', 'MarkerSize', 3);

lidar_pc_array_raw = lidar_board_edge_pts(1:3, :);
cam_pts = zeros(2, 0);
for i = 1:size(lidar_pc_array_raw, 2)
  if lidar_pc_array_raw(1, i) <= 0
    continue;
  end
  cam_pts(:, end+1) = worldpts_to_cam(lidar_pc_array_raw(:, i), result_zhou.T_est_best(1:3,1:3), result_qpep.T_est_best(1:3,4), params.K);
  if (cam_pts(1, end) <= 0 || cam_pts(1, end) > size(img, 2) || ...
      cam_pts(2, end) <= 0 || cam_pts(2, end) > size(img, 1))
    cam_pts(:, end) = [0;0];
    continue;
  end  
end
plot(cam_pts(1, :), cam_pts(2, :), 'go', 'LineStyle', 'none', 'MarkerSize', 12, 'LineWidth', 3);
cam_pts = zeros(2, 0);
for i = 1:size(lidar_pc_array_raw, 2)
  if lidar_pc_array_raw(1, i) <= 0
    continue;
  end
  cam_pts(:, end+1) = worldpts_to_cam(lidar_pc_array_raw(:, i), result_qpep.T_est_best(1:3,1:3), result_qpep.T_est_best(1:3,4), params.K);
  if (cam_pts(1, end) <= 0 || cam_pts(1, end) > size(img, 2) || ...
      cam_pts(2, end) <= 0 || cam_pts(2, end) > size(img, 1))
    cam_pts(:, end) = [0;0];
    continue;
  end
end
plot(cam_pts(1, :), cam_pts(2, :), 'ro', 'LineStyle', 'none', 'MarkerSize', 12, 'LineWidth', 3);
set(gca, 'FontName', 'Times');
hold off;
end

%%
filename = fullfile('figure', 'tmech_version2', 'rlfs01'); print(filename, '-depsc'); saveas(hf, strcat(filename, '.fig'));

%% Print calibration results (TF, error)
% data_type = 'simu_data_bias';
% data_type = 'simu_data';
data_type = 'real_data';
% data_type = 'fp_data';

method = {'Ours', 'Zhou-Matlab'};
result_mat = {'result_lcecalib_qpep.mat', 'result_baseline_zhou.mat'};
for data_option = 7:9
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  result_lcecalib_qpep = load(fullfile(data_path, 'result_lcecalib_qpep.mat'));  
  TGt = result_lcecalib_qpep.TGt;
  
  disp(num2str(rotm2quat(TGt(1:3, 1:3)), '$%.3f$ & '))
  disp(num2str(TGt(1:3, 4)', '$%.3f$ & '))
  for j = 1:length(result_mat)
    result_data = load(fullfile(data_path, result_mat{j}));  
    [r_err_noise(j, data_option), t_err_noise(j, data_option), ...
     mp_err_noise(j, data_option), me_err_noise(j, data_option)] = evaluateTFPlaneEdgeError(...
      TGt, ...
      result_data.T_est_best, ...
      result_lcecalib_qpep.all_cam_board_centers_on_plane, ...
      result_lcecalib_qpep.all_cam_board_plane_coeff, ...
      result_lcecalib_qpep.all_lidar_board_pts, ...
      result_lcecalib_qpep.all_cam_board_corners, ...
      result_lcecalib_qpep.all_lidar_board_edge_pts);
    method{j}
    disp(num2str(rotm2quat(result_data.T_est_best(1:3, 1:3)), '$%.3f$ & '))
    disp(num2str(result_data.T_est_best(1:3, 4)', '$%.3f$ & '))
    str = sprintf('$%.3f$ & $%.3f$ & $%.4f$\n\n', ...
      r_err_noise(j, data_option), t_err_noise(j, data_option), ...
      mp_err_noise(j, data_option)+me_err_noise(j, data_option));
    disp(str)
  end 
end

%% Plot Error of RLFS01 - RLFS03
data_type = 'real_data';
method = {'Ours', 'Zhou-Matlab'};
result_mat = {'result_lcecalib_qpep_sensor_data.mat', 'result_baseline_zhou.mat'};
marker = {'r-o', 'b-d'};
p = zeros(3, 3*length(method));
for data_option = 1:3
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  method_r_err = {};
  method_t_err = {};
  method_mg_err = {};
  label = {};
  data_num = 0;
  TGt = eye(4, 4);
  for j = 1:length(method)
    result_data = load(fullfile(data_path, result_mat{j}));  
    if (j == 1)
      data_num = size(result_data.all_r_err, 2);
      TGt = result_data.TGt;
    end
    tmp = zeros(100, data_num);
    tmp(1:size(result_data.all_r_err, 1), 1:size(result_data.all_r_err, 2)) = result_data.all_r_err;
    method_r_err{end+1} = tmp;
    tmp(1:size(result_data.all_t_err, 1), 1:size(result_data.all_t_err, 2)) = result_data.all_t_err;
    method_t_err{end+1} = tmp;
    tmp(1:size(result_data.all_mp_err, 1), 1:size(result_data.all_mp_err, 2)) = result_data.all_mp_err + result_data.all_me_err;
    method_mg_err{end+1} = tmp;
    label{end+1} = 1:size(tmp, 2);
    result_data.T_est_best
  end 

  for j = 1:length(method)
    figure(1); hold on;
    p(data_option, (j-1)*3+j) = plot(median(method_r_err{j}), marker{j}, 'MarkerSize', 15, 'Color', color_list(data_option, :), 'LineWidth', 1.5);
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2, 'YScale', 'log');
    ylabel('Rotation Error [deg]', 'FontSize', 25);  
    xlabel('Number of Frames', 'FontSize', 25);
    grid on;
    
    figure(2); hold on;
    p(data_option, (j-1)*3+j) = plot(median(method_t_err{j}), marker{j}, 'MarkerSize', 15, 'Color', color_list(data_option, :), 'LineWidth', 1.5);
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2, 'YScale', 'log');
    ylabel('Translation Error [m]', 'FontSize', 25);
    xlabel('Number of Frames', 'FontSize', 25);
    grid on;
    
    figure(3); hold on;
    p(data_option, (j-1)*3+j) = plot(median(method_mg_err{j}), marker{j}, 'MarkerSize', 15, 'Color', color_list(data_option, :), 'LineWidth', 1.5);
    set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2, 'YScale', 'log');
    ylabel('MGE [m]', 'FontSize', 25);
    grid on;
    xlabel('Number of Frames', 'FontSize', 25);
  end
end
figure(1); legend('Ours (RLFS01)', 'Zhou-MATLAB (RLFS01)', 'Ours (RLFS02)', 'Zhou-MATLAB (RLFS02)', 'Ours (RLFS03)', 'Zhou-MATLAB (RLFS03)');
axis([0 26 0 10]);
figure(2); legend('Ours (RLFS01)', 'Zhou-MATLAB (RLFS01)', 'Ours (RLFS02)', 'Zhou-MATLAB (RLFS02)', 'Ours (RLFS03)', 'Zhou-MATLAB (RLFS03)');
axis([0 26 0 1.0]);
figure(3); legend('Ours (RLFS01)', 'Zhou-MATLAB (RLFS01)', 'Ours (RLFS02)', 'Zhou-MATLAB (RLFS02)', 'Ours (RLFS03)', 'Zhou-MATLAB (RLFS03)');
axis([0 26 0 0.3]);

%%
% filename = fullfile('figure', 'tmech_version2', 'rlfs01'); print(filename, '-depsc'); saveas(hf, strcat(filename, '.fig'));

%% Plot Error of RLES01 - RLES03
data_type = 'real_data';
method = {'Ours', 'Zhou-Matlab'};
result_mat = {'result_lcecalib_qpep_sensor_data.mat', 'result_baseline_zhou.mat'};
marker = {'r-o', 'b-d'};
p = zeros(3, 3*length(method));
cnt = 1;
for data_option = 4:9
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  method_r_err = {};
  method_t_err = {};
  method_mg_err = {};
  label = {};
  data_num = 0;
  TGt = eye(4, 4);
  j = 1;
  result_data = load(fullfile(data_path, result_mat{j}));  
  data_num = size(result_data.all_r_err, 2);
  TGt = result_data.TGt;
  tmp = zeros(100, data_num);
  tmp(1:size(result_data.all_r_err, 1), 1:size(result_data.all_r_err, 2)) = result_data.all_r_err;
  method_r_err{end+1} = tmp;
  tmp(1:size(result_data.all_t_err, 1), 1:size(result_data.all_t_err, 2)) = result_data.all_t_err;
  method_t_err{end+1} = tmp;
  tmp(1:size(result_data.all_mp_err, 1), 1:size(result_data.all_mp_err, 2)) = result_data.all_mp_err + result_data.all_me_err;
  method_mg_err{end+1} = tmp;
  label{end+1} = 1:size(tmp, 2);
  result_data.T_est_best

  figure(1); hold on;
  p(data_option, (j-1)*3+j) = plot(median(method_r_err{j}), marker{mod(data_option, 2)+1}, 'MarkerSize', 15, 'Color', color_list(floor(cnt), :), 'LineWidth', 2);
  set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2);
  ylabel('Rotation Error [deg]', 'FontSize', 25);  
  xlabel('Number of Frames', 'FontSize', 25);
  grid on;

  figure(2); hold on;
  p(data_option, (j-1)*3+j) = plot(median(method_t_err{j}), marker{mod(data_option, 2)+1}, 'MarkerSize', 15, 'Color', color_list(floor(cnt), :), 'LineWidth', 2);
  set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2);
  ylabel('Translation Error [m]', 'FontSize', 25);
  xlabel('Number of Frames', 'FontSize', 25);
  grid on;

  figure(3); hold on;
  p(data_option, (j-1)*3+j) = plot(median(method_mg_err{j}), marker{mod(data_option, 2)+1}, 'MarkerSize', 15, 'Color', color_list(floor(cnt), :), 'LineWidth', 2);
  set(gca, 'FontName', 'Times', 'FontSize', 20, 'LineWidth', 2);
  ylabel('MGE [m]', 'FontSize', 25);
  grid on;
  xlabel('Number of Frames', 'FontSize', 25);
  cnt = cnt + 0.5;
end
figure(1); 
% legend('Ours (RLES01-LF)', 'Ours (RLES01-LE)', 'Ours (RLES02-LF)', 'Ours (RLES02-LE)', 'Ours (RLES03-LF)', 'Ours (RLES03-LE)');
% axis([0 35 0 10]);
figure(2); 
% legend('Ours (RLES01-LF)', 'Ours (RLES01-LE)', 'Ours (RLES02-LF)', 'Ours (RLES02-LE)', 'Ours (RLES03-LF)', 'Ours (RLES03-LE)');
% axis([0 35 0 1.0]);
figure(3); 
legend('Ours (RLES01-LF)', 'Ours (RLES01-LE)', 'Ours (RLES02-LF)', 'Ours (RLES02-LE)', 'Ours (RLES03-LF)', 'Ours (RLES03-LE)');
% axis([0 35 0 0.3]);

%%
% filename = fullfile('figure', 'tmech_version2', 'rlfs01'); print(filename, '-depsc'); saveas(hf, strcat(filename, '.fig'));

%% load ablation study result (Section VI.D)
eval_result.r_err = zeros(9, 7);
eval_result.t_err = zeros(9, 7);
eval_result.mp_err = zeros(9, 7);
eval_result.me_err = zeros(9, 7);
eval_result.mg_err = zeros(9, 7);
data_type = 'real_data';
for data_option = 1:9
  result_folder = fullfile('data', data_type, sprintf('real_data_%d', data_option));
  %%%%%%%%%%%%%%%%%%% setup 1
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_1_100_1_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 1) = r_err;
  eval_result.t_err(data_option, 1) = t_err;
  eval_result.mp_err(data_option, 1) = mp_err;
  eval_result.me_err(data_option, 1) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 2
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_0_1_100_1_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 2) = r_err;
  eval_result.t_err(data_option, 2) = t_err;
  eval_result.mp_err(data_option, 2) = mp_err;
  eval_result.me_err(data_option, 2) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 3
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_0_100_1_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 3) = r_err;
  eval_result.t_err(data_option, 3) = t_err;
  eval_result.mp_err(data_option, 3) = mp_err;
  eval_result.me_err(data_option, 3) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 4
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_1_100_0_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 4) = r_err;
  eval_result.t_err(data_option, 4) = t_err;
  eval_result.mp_err(data_option, 4) = mp_err;
  eval_result.me_err(data_option, 4) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 5
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_1_100_1_0.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 5) = r_err;
  eval_result.t_err(data_option, 5) = t_err;
  eval_result.mp_err(data_option, 5) = mp_err;
  eval_result.me_err(data_option, 5) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 6
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_1_1_1_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 6) = r_err;
  eval_result.t_err(data_option, 6) = t_err;
  eval_result.mp_err(data_option, 6) = mp_err;
  eval_result.me_err(data_option, 6) = me_err;
  %%%%%%%%%%%%%%%%%%% setup 7
  result_filename = fullfile(result_folder, ...
    'result_lcecalib_qpep_sensor_data_1_1_5_1_1.mat');
  result_data = load(result_filename);
  [r_err, t_err, mp_err, me_err] = evaluateTFPlaneEdgeError(...
    result_data.TGt, ...
    result_data.T_est_best, ...
    result_data.all_cam_board_centers_on_plane, ...
    result_data.all_cam_board_plane_coeff, ...
    result_data.all_lidar_board_pts, ...
    result_data.all_cam_board_corners, ...
    result_data.all_lidar_board_edge_pts);
  eval_result.r_err(data_option, 7) = r_err;
  eval_result.t_err(data_option, 7) = t_err;
  eval_result.mp_err(data_option, 7) = mp_err;
  eval_result.me_err(data_option, 7) = me_err;
  %%%%%%%%%%%%%%%%%%% 
  eval_result.mg_err(data_option, :) = ...
    eval_result.mp_err(data_option, :) + eval_result.me_err(data_option, :);
end
disp(num2str(eval_result.r_err(1:3, :)', '$%.3f$ & '));
disp('-----------------------------------')
disp('-----------------------------------')
disp('-----------------------------------')
disp(num2str(eval_result.t_err(1:3, :)', '$%.3f$ & '));
disp('-----------------------------------')
disp('-----------------------------------')
disp('-----------------------------------')
disp(num2str(eval_result.mg_err(1:3, :)', '$%.4f$ & '));

































