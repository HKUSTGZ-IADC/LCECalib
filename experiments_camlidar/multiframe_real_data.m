clc; 
clear;
close all;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

% data_type = 'real_data';
% data_fold = {'real_data_1', 'real_data_2', 'real_data_3'};

data_type = 'real_data';
data_fold = {'real_data_4', 'real_data_5', 'real_data_6', 'real_data_7', 'real_data_8', 'real_data_9'};
for data_option = 1:6
  sprintf('data_option: %d', data_option)
  %% parameters
  params = load(fullfile(data_type, data_fold{data_option}, 'params.mat'));
  borW = params.borW;
  borH = params.borH;
  numW = params.numW;
  numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K;
  D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  is_display = 0;
  all_iterations = 100;

  %% Extract features
  pcd_path = fullfile(data_type, data_fold{data_option}, 'pcd');
  img_path = fullfile(data_type, data_fold{data_option}, 'img');
  img_list = dir(img_path);
  pcd_list = dir(pcd_path);
  pcd_list = pcd_list(3:end);
  img_list = img_list(3:end);

  board_pts = {};
  img_corner3D = {};
  pcd_corner3D = {};
  pc_bors_ceoff = {};
  cam_bors_coeff = {};

  cnt = 0;
  for idx = 1:min(60, num_data)
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      pc_array = pc_raw.Location()';

      [imagePoints, boardSize] = detectCheckerboardPoints(img_raw);
      if (boardSize(1) == 0 || boardSize(2) == 0)
        continue;
      end
      if (boardSize(1) < numH || boardSize(2) < numW)
        continue;
      end      
      [im_corners, cam_plane_coeff, pro_err] = imgbor_ext(img_raw, K, D, pattern_size, borW, borH, is_display);

      [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
      if (isempty(pts_bor))
        continue;
      end      
%       figure, pcshow(pointCloud(pts_bor'))
      pc_corners = borcorner_ext(pts_bor, borW, borH, is_display);
  
      % save data
      cnt = cnt + 1;
      broad_pts{cnt} = pts_bor;
      pcd_corner3D{cnt} = pc_corners;
      pc_bors_ceoff{cnt} = bor_coeff;
      img_corner3D{cnt} = im_corners;
      cam_bors_coeff{cnt} = cam_plane_coeff;
  end

  %% Calibration
  aver_t_err = [];
  aver_r_err = [];
  TOptm_best = eye(4, 4);
  min_t = 1000;
  for PoseNum = 1:length(cam_bors_coeff) - 1
      multi_theta_errs=[];
      t_errs = [];    
      for iter = 1:all_iterations    
        reidx = randperm(size(img_corner3D,2));
        sub_cam_corners3D = {};
        sub_lidar_corners3D = {};
        sub_pc_bors_coeff = {};
        sub_cam_bors_coeff = {};
        for idx = 1:PoseNum
            sub_cam_corners3D{idx} = img_corner3D{reidx(idx)};
            sub_lidar_corners3D{idx} = pcd_corner3D{reidx(idx)};
            sub_pc_bors_coeff{idx} = pc_bors_ceoff{reidx(idx)};
            sub_cam_bors_coeff{idx} = cam_bors_coeff{reidx(idx)};
        end

        TInit = plane_init(sub_pc_bors_coeff,sub_cam_bors_coeff,sub_lidar_corners3D,sub_cam_corners3D);
        TOptm = corner_optm(sub_cam_corners3D,sub_lidar_corners3D,TInit);
%         TGt
%         TOptm
        deltaT = inv(TGt) * TOptm;
        deltaQ = rotm2quat(deltaT(1:3,1:3));
        angle_err = abs(2*acosd(deltaQ(1)));
        multi_theta_errs = [multi_theta_errs, angle_err];
        t_errs = [t_errs, norm(deltaT(1:3,4))];
  %       imshow(pt_project_depth2image(TOptm,K,pc_array,myundistortImage(img_raw,K,D)));
        if (t_errs < min_t)
          min_t = t_errs;
          TOptm_best = TOptm;
        end
      end
      aver_t_err = [aver_t_err, t_errs'];
      aver_r_err = [aver_r_err, multi_theta_errs'];
      sprintf('PoseNum: %d', PoseNum)
  end
  TOptm = TOptm_best;
  save(fullfile(data_type, data_fold{data_option}, 'result_proposed.mat'), 'aver_r_err', 'aver_t_err', 'TOptm', ...
    'broad_pts', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff');

  %% Plot results
%   figure; boxplot(aver_r_err(:, 3: end));
%   xlabel("Number of Poses"); title("Rotation Error [deg]");
%   grid on;
%   ax = gca;
%   ax.GridLineStyle = '--';
%   ax.GridAlpha = 0.3;
%   set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
%   title('Mean and Median Rotation Error', 'FontSize', 30, 'FontWeight', 'normal');
%   box on;
% 
%   figure; boxplot(aver_t_err(:, 3: end));
%   xlabel("Number of Poses"); ylabel("Translation Error [m]");
%   grid on;
%   ax = gca;
%   ax.GridLineStyle = '--';
%   ax.GridAlpha = 0.3;
%   set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
%   title('Mean and Median Translation Error', 'FontSize', 30, 'FontWeight', 'normal');
%   box on;
end




