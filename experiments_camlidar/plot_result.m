clc, clear;
close all;

load('color_list.mat');
hf_list = [];

% data_type = 'simu_data';
% data_fold = {'noise-free', 'noise-0.015', 'noise-0.03'};
% str_legend = {' (\sigma=0)', ' (\sigma=0.015)', ' (\sigma=0.03)'};
% len = 1.3049;

% data_type = 'real_data';
% data_fold = {'real_data_1', 'real_data_2', 'real_data_3'};
% str_legend = {' (RLFS01)', ' (RLFS02)', ' (RLFS03)'};
% len = 1.3049;

data_type = 'real_data';
data_fold = {'real_data_4', 'real_data_5', 'real_data_6', 'real_data_7', 'real_data_8', 'real_data_9'};
str_legend = {' (RLES01-LF)', ' (RLES01-LE)', ' (RLES02-LF)', ' (RLES02-LE)', ' (RLES03-LF)', ' (RLES03-LE)'};
len = 1.3049;

%% Plot calibration error
for option = 1:6
%   sl{(option - 1) * 4 + 1} = strcat('Our mean error', str_legend{option});
%   sl{(option - 1) * 2 + 1} = strcat('Median error (Proposed)', str_legend{option});
%   sl{(option - 1) * 4 + 3} = strcat('Zhou et al. mean error', str_legend{option});
%   sl{(option - 1) * 2 + 2} = strcat('Median error (Baseline)', str_legend{option});
  sl{option} = strcat('Median error (Proposed)', str_legend{option});

  %
  figure(1), hold on;
  result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
  result = load(result_proposed_filename); 
%   plot(mean(result.aver_r_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
%   plot(median(result.aver_r_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  if (mod(option, 2) == 0)
    plot(median(result.aver_r_err(:, 1:end), 1), 'Color', color_list(floor((option + 1) / 2), :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
  else
    plot(median(result.aver_r_err(:, 1:end), 1), 'Color', color_list(floor((option + 1) / 2), :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  end
  
%   result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
%   result = load(result_baseline_filename); 
%   plot(mean(result.aver_r_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(median(result.aver_r_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
   
  xlabel("Number of Frames"); ylabel("Rotation Error [deg]");
  grid on;
  ax = gca;
  ax.GridLineStyle = '--';
  ax.GridAlpha = 0.3;
%   set(gca, 'FontName', 'Times', 'FontSize', 30, 'LineWidth', 2, 'YScale', 'log');
  set(gca, 'FontName', 'Times', 'FontSize', 30, 'LineWidth', 2);  
%   title('Mean and Median Rotation Error', 'FontSize', 35, 'FontWeight', 'normal');
%   xlim([1, length(mean(result.aver_r_err(:, 1:end))) - 2]);
  box on;
  legend(sl, 'Location', 'northeast', 'FontSize', 20);
  axis([2, 56, 0.3, 2]);

  %
  figure(2), hold on;
  result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
  result = load(result_proposed_filename); 
%   plot(mean(result.aver_t_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
%   plot(median(result.aver_t_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  if (mod(option, 2) == 0)
    plot(median(result.aver_t_err(:, 1:end), 1), 'Color', color_list(floor((option + 1) / 2), :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
  else
    plot(median(result.aver_t_err(:, 1:end), 1), 'Color', color_list(floor((option + 1) / 2), :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
  end
  
%   result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
%   result = load(result_baseline_filename); 
%   plot(mean(result.aver_t_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
%   plot(median(result.aver_t_err(:, 1:end), 1), 'Color', color_list(option, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);
   
  xlabel("Number of Frames"); ylabel("Translation Error [m]");
  grid on;
  ax = gca;
  ax.GridLineStyle = '--';
  ax.GridAlpha = 0.3;
  set(gca, 'FontName', 'Times', 'FontSize', 30, 'LineWidth', 2, 'YScale', 'log');
%   set(gca, 'FontName', 'Times', 'FontSize', 35, 'LineWidth', 2);
%   title('Mean and Median Translation Error', 'FontSize', 35, 'FontWeight', 'normal');
%   xlim([1, length(mean(result.aver_r_err(:, 1:end))) - 2]);
  box on;
  legend(sl, 'Location', 'northeast', 'FontSize', 20);
  axis([2, 561 0.04, 0.1]);
end

%% Save plots
for option = 1:length(data_fold)
  hf = figure(1);
  filename = fullfile(data_type, data_fold(option), 'plot_mean_rot');
  print(filename{1}, '-depsc');
  saveas(hf, strcat(filename{1}, '.fig'));
  
  hf = figure(2);
  filename = fullfile(data_type, data_fold(option), 'plot_mean_tsl');
  print(filename{1}, '-depsc');  
  saveas(hf, strcat(filename{1}, '.fig'));
end

%% Save LiDAR point back-projection results
idx = 20;
for option = 1:length(data_fold)
  params = load(fullfile(data_type, data_fold{option}, 'params.mat'));
  K = params.K;
  D = params.D;
  
  pcd_path = fullfile(data_type, data_fold{option}, 'raw_pcd');
  img_path = fullfile(data_type, data_fold{option}, 'img');
  img_list = dir(img_path);
  pcd_list = dir(pcd_path);
  pcd_list = pcd_list(3:end);
  img_list = img_list(3:end);
  
  img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
  pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
  img_raw = imread(img_file);
  pc_raw = pcread(pcd_file);
  pc_array = pc_raw.Location()';  
  pc_filter = pc_array(:, pc_array(1, :) > 0);
  [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, params.borW, params.borH);

  figure(1), hold on;
  img_undist = myundistortImage(img_raw, K, D);
  if (size(size(img_undist), 2) == 2)
    img_undist = cat(3, img_undist, img_undist, img_undist);
  end
  
  result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
  result = load(result_proposed_filename);
  p_camera = K * (result.TOptm(1:3, 1:3) * pc_filter + result.TOptm(1:3, 4));
  u_camera = round(p_camera(1:2, :) ./ p_camera(3, :));
  for i = 1:size(u_camera, 2)
    u = u_camera(:, i);
    if (~insideImage(u, params.imageHeight, params.imageWidth, 3))
      continue;
    end
    for j = u(2) : u(2)
      for k = u(1) : u(1)
        img_undist(j, k, :) = [255, 0, 0];
      end
    end
  end
    
  result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
  result = load(result_baseline_filename);
  p_camera = K * (result.TOptm(1:3, 1:3) * pts_bor + result.TOptm(1:3, 4));
  u_camera = round(p_camera(1:2, :) ./ p_camera(3, :));
  for i = 1:size(u_camera, 2)
    u = u_camera(:, i);
    if (~insideImage(u, params.imageHeight, params.imageWidth, 3))
      continue;
    end
    for j = u(2) : u(2)
      for k = u(1) : u(1)
        img_undist(j, k, :) = [0, 255, 0];
      end
    end
  end 
  imshow(img_undist); hold on;
%   plot(1, 1, 'r.', 'MarkerSize', 20);
%   plot(1, 1, 'g.', 'MarkerSize', 20);
%   legend('Proposed', 'Baseline'); 
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2); hold off;
   
%   hf = figure(1);
%   filename = fullfile(data_type, data_fold(option), 'calibrate_image');
%   print(filename{1}, '-depsc');  
%   saveas(hf, strcat(filename{1}, '.fig'));  
%   
  figure(2), pcshow(pointCloud(pc_array'));
  set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 2); hold off;
end

%% Print calibrated extrinsics
rotm2eul()
for option = 1:length(data_fold)
  sprintf("Data: %d", option)

  result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
  result = load(result_proposed_filename); 
  sprintf("(Proposed) rot error: %.3f, tsl error: %.3f", ...
    median(result.aver_r_err(:, 1:end), 1), median(result.aver_t_err(:, 1:end), 1)) 
  
  result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
  result = load(result_baseline_filename); 
  sprintf("(Baseline) rot error: %.3f, tsl error: %.3f", ...
    median(result.aver_r_err(:, 1:end), 1), median(result.aver_t_err(:, 1:end), 1))
end

%% Computer planar error
if (strcmp(data_type, 'real_data'))
  for option = 1:length(data_fold)
    option
    params = load(fullfile(data_type, data_fold{option}, 'params.mat'));
    broad_pts = load(fullfile(data_type, data_fold{option}, 'result_proposed.mat'));
    
    result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
    result_proposed = load(result_proposed_filename); 
    all_err_proposed = computePlanarError(result_proposed, params, result_proposed.TOptm, broad_pts.broad_pts);
    sprintf('(Proposed) mean PE at %d: %.3f', option, mean(all_err_proposed))

    result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
    result_baseline = load(result_baseline_filename); 
    all_err_baseline = computePlanarError(result_proposed, params, result_baseline.TOptm, broad_pts.broad_pts);
    sprintf('(Baseline) mean PE at %d: %.3f', option, mean(all_err_baseline))

    all_err_gt = computePlanarError(result_proposed, params, params.TGt, broad_pts.broad_pts);
    sprintf('(GT) mean PE at %d: %.3f', option, mean(all_err_gt))
    
%     figure; hold on;
%     plot(all_err_proposed, 'Color', color_list(1, :), 'LineStyle', '-', 'LineWidth', 2, 'Marker', 'd', 'MarkerSize', 10);
%     plot(all_err_baseline, 'Color', color_list(2, :), 'LineStyle', '--', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 10);    
%     sl = {'Proposed', 'Zhou et al.'};
%     xlabel("Number of Frames"); ylabel("Error [m]");
%     grid on;
%     ax = gca;
%     ax.GridLineStyle = '--';
%     ax.GridAlpha = 0.3;
%     set(gca, 'FontName', 'Times', 'FontSize', 35, 'LineWidth', 2);
%     title('Mean Planar Error', 'FontSize', 45, 'FontWeight', 'normal');
% %     xlim([0, length(all_err_proposed) - 2]);
%     box on;
%     legend(sl);
  end
end

%% Print GT and estimiate extrinsics
for option = 1:length(data_fold) 
  option
  params = load(fullfile(data_type, data_fold{option}, 'params.mat'));
  q = rotm2quat(params.TGt(1:3, 1:3));
  t = params.TGt(1:3, 4)';
  sprintf('GT: & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$', ...
    q(1), q(2), q(3), q(4), t(1), t(2), t(3))

  result_baseline_filename = fullfile(data_type, data_fold{option}, 'result_baseline.mat');
  result = load(result_baseline_filename); 
  q = rotm2quat(result.TOptm(1:3, 1:3));
  t = result.TOptm(1:3, 4)';
  sprintf('Baseline: & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$', ...
    q(1), q(2), q(3), q(4), t(1), t(2), t(3), ...
    median(result.aver_r_err(:, end)), median(result.aver_t_err(:, end)))

  result_proposed_filename = fullfile(data_type, data_fold{option}, 'result_proposed.mat');
  result = load(result_proposed_filename); 
  q = rotm2quat(result.TOptm(1:3, 1:3));
  t = result.TOptm(1:3, 4)';
  sprintf('Proposed: & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$ & $%.3f$', ...
    q(1), q(2), q(3), q(4), t(1), t(2), t(3), ...
    median(result.aver_r_err(:, end)), median(result.aver_t_err(:, end)))
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
function all_err = computePlanarError(result, params, TOptm, broad_pts)
  Test = TOptm;  % tform from camera to LiDAR
  K = params.K;
  
  all_err = [];
  for i = 1:length(result.cam_bors_coeff)
    pts_bor = broad_pts{i};
    pts_bor_cam = Test(1:3, 1:3) * pts_bor + Test(1:3, 4);
    cam_bors_coeff = result.cam_bors_coeff{i};
    planar_err = cam_bors_coeff(1:3)' * pts_bor_cam + cam_bors_coeff(4);
    mean_planar_err = mean(abs(planar_err));
    all_err = [all_err, mean_planar_err];
  end
end

%%
function all_err = computeCornerError(result, params, TOptm)
  Test = TOptm;  % tform from camera to LiDAR
  K = params.K;
  imageHeight = params.imageHeight;
  imageWidth = params.imageWidth;

  all_err = [];
  for j = 1:length(result.pcd_corner3D)
    all_pcd_corner3D = result.pcd_corner3D{j};
    all_img_corner = result.img_corner3D{j};
    total_dis = 0;      
    for k = 1:4
      P_cam = all_img_corner(:, k);
      min_d = 10000;
      min_idx = 1;
      for l = 1:4
        P_lidar = Test(1:3, 1:3) * all_pcd_corner3D(:, l) + Test(1:3, 4);
        dis = norm(P_cam - P_lidar);
        if (dis < min_d)
          min_d = dis;
          min_idx = l;
        end
      end
      P_lidar = Test(1:3, 1:3) * all_pcd_corner3D(:, min_idx) + Test(1:3, 4);
      total_dis = total_dis + norm(P_cam - P_lidar);
    end
    total_dis = total_dis / 4;
    all_err = [all_err, total_dis];
  end 
end

%%
function flag = insideImage(u, height, width, margin)
  if (u(2) <= margin || u(2) > height - margin || u(1) <= margin || u(1) > width - margin)
    flag = false;
  else
    flag = true;
  end  
end















