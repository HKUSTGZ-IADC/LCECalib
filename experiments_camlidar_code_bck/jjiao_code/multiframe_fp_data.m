% function [] = multiframe_handheld_data(preprocess_roi_flag, visualization_flag)
% clc; clear; close all;
close all;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

preprocess_roi_flag = false;
visualization_flag = false;
TInit = [0 -1 0 0.12; 0 0 -1 -0.05; 1 0 0 -0.03];

%%
data_type = 'data/fp_data';
data_fold = {'fp_data_1'};
sensor_type = {'frame_img', 'event_img'};

for data_option = 1:1
  sprintf('data_option: %d', data_option)
  for sensor_option = 1:length(sensor_type)
    %% parameters
    params = load(fullfile(data_type, data_fold{data_option}, ...
      sensor_type{sensor_option}, 'params.mat'));
    borW = params.borW;
    borH = params.borH;
    numW = params.numW;
    numH = params.numH;
    pattern_size = params.pattern_size;
    K = params.K;
    D = params.D;
    TGt = params.TGt;
    num_data = params.num_data;
    is_display = false;
    all_iterations = 1;

    %% Extract ROI point cloud
    if (preprocess_roi_flag)
      pcd_path = fullfile(data_type, data_fold{data_option}, 'pcd_refined');
      pcd_list = dir(pcd_path);
      pcd_list = pcd_list(3:end);    
      for idx = 1:num_data
        pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
        pc_raw = pcread(pcd_file);
        roi = [0.7, 5, -2.0, 2, -0.5, 1.2];
        indices = findPointsInROI(pc_raw, roi);
        pc_roi = select(pc_raw, indices);
        
        pcd_file_save = strcat(fullfile(data_type, data_fold{data_option}, 'pcd_roi'), '/', pcd_list(idx).name);
        pcwrite(pc_roi, pcd_file_save);

        pc_array = [pc_raw.Location()'; pc_raw.Intensity'];
        [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
        pcd_file_save = strcat(fullfile(data_type, data_fold{data_option}, 'pcd_board'), '/', pcd_list(idx).name);
        pcwrite(pointCloud(pts_bor(1:3, :)', 'Intensity', pts_bor(4, :)'), pcd_file_save);
      end
    end
    
    %% Extract features
    pcd_path = fullfile(data_type, data_fold{data_option}, 'pcd_roi');
    img_path = fullfile(data_type, data_fold{data_option}, sensor_type{sensor_option});
    pcd_list = dir(pcd_path);
    pcd_list = pcd_list(3:end);

    img_calib = {};
    board_pts = {};
    img_corner3D = {};
    pcd_corner3D = {};
    pc_bors_ceoff = {};
    cam_bors_coeff = {};

    cnt = 0;
    for idx = 1:min(length(pcd_list), num_data)
        pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
        if (~exist(pcd_file)) 
          continue;
        end
        ss = split(pcd_list(idx).name, '.');
        img_file = strcat(img_path, '/', ss{1}, '.png');
        if (~exist(img_file)) 
          continue;
        end        
                
        img_raw = imread(img_file);
        pc_raw = pcread(pcd_file);
        pc_array = [pc_raw.Location()'; pc_raw.Intensity'];

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
        pc_corners = borcorner_ext(pts_bor, borW, borH, is_display);
        if (visualization_flag)
          figure; hold on;
          pcshow(pointCloud(pts_bor(1:3, :)', 'Intensity', pts_bor(4, :)'));
          pcshow(pc_corners', [1,0,0], 'MarkerSize', 1000);
          hold off;
        end
        sprintf('%s', img_file)

        % save data
        cnt = cnt + 1;
        img_calib{cnt} = img_raw;
        pts_calib{cnt} = pc_array(1:3, :);
        broad_pts{cnt} = pts_bor(1:3, :);
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
    for PoseNum = length(cam_bors_coeff):length(cam_bors_coeff)
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

%           TInit = plane_init(sub_pc_bors_coeff,sub_cam_bors_coeff,sub_lidar_corners3D,sub_cam_corners3D);
          TOptm = corner_optm(sub_cam_corners3D,sub_lidar_corners3D,TInit);
          % TInit is already given
          TInit
          TOptm
          deltaT = inv(TGt) * TOptm;
          deltaQ = rotm2quat(deltaT(1:3,1:3));
          angle_err = abs(2*acosd(deltaQ(1)));
          multi_theta_errs = [multi_theta_errs, angle_err];
          t_errs = [t_errs, norm(deltaT(1:3,4))];
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
    if (visualization_flag)
      close all;
      for i = 1:length(img_calib)
        figure, imshow(pt_project_depth2image(TOptm,K,pts_calib{i},myundistortImage(img_calib{i},K,D)));
%       figure, imshow(pt_project_depth2image(TOptm,K,broad_pts{i},myundistortImage(img_calib{i},K,D)));
      end
    end   

    T_lidar_cam = TOptm^(-1);
    t_lidar_cam = T_lidar_cam(1:3, 4);
    q_lidar_cam = rotm2quat(T_lidar_cam(1:3, 1:3));    
    save(fullfile(data_type, data_fold{data_option}, sensor_type{sensor_option}, 'result_proposed.mat'), ...
      'aver_r_err', 'aver_t_err', 'TOptm', 'img_calib', 'pts_calib', ...
      'broad_pts', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff', ...
      't_lidar_cam', 'q_lidar_cam');
  end
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
    close all;
    pcd_path = fullfile(data_type, data_fold{data_option}, 'pcd_raw');
    img_path = fullfile(data_type, data_fold{data_option}, sensor_type{sensor_option});
    pcd_list = dir(pcd_path);
    pcd_list = pcd_list(3:end);
    for idx = 20:30
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      if (~exist(pcd_file)) 
        continue;
      end
      ss = split(pcd_list(idx).name, '.');
      img_file = strcat(img_path, '/', ss{1}, '.png');
      if (~exist(img_file)) 
        continue;
      end
      cloud = pcread(pcd_file);
      pts = cloud.Location';
      img = imread(img_file);
      if (visualization_flag)
        figure, imshow(pt_project_depth2image(TOptm,K,pts, myundistortImage(img,K,D)));
      end   
    end
end





