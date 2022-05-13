clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

data_type = 'real_data';
visualization_flag = 1;
debug_flag = 1;

%% load data and extract features
for data_option = 1:6
  sprintf('data_option: %d', data_option)
  data_path = fullfile(data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K;
  D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  all_iterations = 1;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  

  board_pts = {}; 
  img_corner3D = {}; 
  pcd_corner3D = {};
  pc_bors_ceoff = {}; 
  cam_bors_coeff = {};
  for idx = 1:min(25, num_data)
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      pc_array = pc_raw.Location()';
      
      %% image: feature extraction
      [img_undist, camParams] = undistort_image(img_raw, K, D);
      [imagePoints, boardSize] = detectCheckerboardPoints(img_undist);
      if (boardSize(1) == 0 || boardSize(2) == 0)
        continue;
      end
      if (boardSize(1) < numH || boardSize(2) < numW)
        continue;
      end     
      
      worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
      worldPoints = [worldPoints, zeros(size(worldPoints,1),1)];

%       Compared with MATLAB baseline
%       [worldOrientation, worldLocation] = ...
%         estimateWorldCameraPose(imagePoints, worldPoints, camParams);
%       T= [[worldOrientation;worldLocation],[0,0,0,1]'];
%       T = T';
%       T = inv(T);
%       T
%       worldPx=T(1:3,1:3)*worldPoints'+T(1:3,4);
%       worldPx = K * worldPx;
%       worldPx = worldPx(1:2,:)./worldPx(3,:);
%       imagePoints= imagePoints';

      [R_cam_board, t_cam_board, ~, ~] = ...
        qpep_pnp(imagePoints, worldPoints, K', false, false);
      R_cam_board = R_cam_board'; 
      T = [R_cam_board, t_cam_board; 0 0 0 1];
      
      % estimate checkerboard corners
      center = mean(worldPoints,1);
      pts_corner = [[borW/2,borH/2,0]', [borW/2,-borH/2,0]',...
                    [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
      pts_corner = center' + pts_corner;
      pts_corner = T(1:3, 1:3) * pts_corner + T(1:3, 4);
      pc_corner_px = worldpts_to_cam(pts_corner, eye(3, 3), zeros(3, 1), K);
      pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);
      plane_coeff = T(1:3,1:3) * [0,0,1]';
      d = -plane_coeff'*T(1:3,4);
      plane_coeff = [plane_coeff;d];
      if d<0
          plane_coeff =-plane_coeff;
      end     
      worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
      if debug_flag
        figure; 
        subplot(121); imshow(img_raw);
        subplot(122); imshow(img_undist);
        hold on;
        plot(worldPx(1,:),worldPx(2,:),'.r');
        plot(pc_corner_px(1,:), pc_corner_px(2,:),'-ob');
        hold off;
      end
      
      im_corners = pts_corner;
      cam_plane_coeff = plane_coeff;
      
%       [im_corners, cam_plane_coeff, pro_err] = ...
%         imgbor_ext(img_raw, K, D, pattern_size, borW, borH, visualization_flag);
%       im_corners
%       cam_plane_coeff'

      [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
      if (isempty(pts_bor))
        continue;
      end      
      pc_corners = borcorner_ext(pts_bor, borW, borH, visualization_flag);

      if (visualization_flag)
        figure; hold on;
        pcshow(pointCloud(pts_bor(1:3, :)', 'Intensity', pts_bor(4, :)'));
        pcshow(pc_corners', [1,0,0], 'MarkerSize', 1000);
        hold off;
      end
  
      board_pts{end + 1} = pts_bor;
      pcd_corner3D{end + 1} = pc_corners;
      pc_bors_ceoff{end + 1} = bor_coeff;
      img_corner3D{end + 1} = im_corners;
      cam_bors_coeff{end + 1} = cam_plane_coeff;
  end

  %% Calibration
  aver_t_err = [];
  aver_r_err = [];
  TOptm_best = eye(4, 4);
  min_t = 1000;
%   for PoseNum = 1:length(cam_bors_coeff) - 1
  for PoseNum = length(cam_bors_coeff) - 1
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
  save(fullfile(data_path, 'result_proposed.mat'), 'aver_r_err', 'aver_t_err', 'TOptm', ...
    'board_pts', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff');

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




