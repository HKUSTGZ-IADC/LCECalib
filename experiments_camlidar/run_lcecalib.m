clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

data_type = 'real_data';
visualization_flag = 0;
debug_flag = 1;

%% load data and extract features
for data_option = 3:6
  sprintf('data_option: %d', data_option)
  data_path = fullfile(data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  all_iterations = 1;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  

  imgs_undist = {}; pcs_array = {};
  board_pts = {}; 
  img_corner3D = {}; cam_bors_coeff = {}; cam_bors_center = {};
  pcd_corner3D = {}; pc_bors_ceoff = {}; 
  for idx = 2:min(25, num_data)
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      pc_array = pc_raw.Location()';
      
      %% image: feature extraction
      % LCE-CALIB version 2
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
%       if (boardSize(1) ~= numH || boardSize(2) ~= numW)
%         continue;
%       end
      worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
      worldPoints = [worldPoints, zeros(size(worldPoints,1),1)];

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % option 1: MATLAB baseline
      [R_world_cam, t_world_cam] = ...
        estimateWorldCameraPose(imagePoints, worldPoints, camParams);
      T_world_cam = [[R_world_cam', t_world_cam']; [0,0,0,1]];
      T_matlab_pnp = inv(T_world_cam);
      disp('T_matlab_pnp')
      disp(num2str(T_matlab_pnp, '%5f '))      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % option 2: QPEP
      [R_cam_world, t_cam_world, ~, ~] = ...
        qpep_pnp(imagePoints, worldPoints, K', false, false);
      R_cam_world = R_cam_world'; 
      T_qpep_pnp = [R_cam_world, t_cam_world; 0 0 0 1];
      %%%% check if QPEP has another PNP solution
      if t_cam_world(3) > 0
        T_qpep_pnp = [R_cam_world, t_cam_world; 0 0 0 1];
      else
        eul = rotm2eul(R_cam_world, 'ZYX');
        R = eul2rotm([eul(1) - pi, -eul(2), -eul(3)], 'ZYX');
        T_qpep_pnp = [R, -t_cam_world; 0 0 0 1];
      end
      disp('T_qpep_pnp')
      disp(num2str(T_qpep_pnp, '%5f '))
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      T = T_qpep_pnp;
      
      % estimate checkerboard corners
      center = mean(worldPoints,1);
      pts_corner = [[borW/2,borH/2,0]', [borW/2,-borH/2,0]',...
                    [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
      pts_corner = center' + pts_corner;
      pts_corner = T(1:3, 1:3) * pts_corner + T(1:3, 4);
      pc_corner_px = worldpts_to_cam(pts_corner, eye(3, 3), zeros(3, 1), K);
      pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);
      n = T(1:3,1:3) * [0,0,1]';
      d = -(n' * T(1:3,4));
      plane_coeff = [n; d];
%       if plane_coeff(3) < 0
%           plane_coeff = -plane_coeff;
%       end 
      worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
      if debug_flag
        figure; 
        subplot(121); imshow(img_raw);
        subplot(122); imshow(img_undist);
        hold on;
        plot(worldPx(1,:),worldPx(2,:),'.r');
        plot(pc_corner_px(1,:), pc_corner_px(2,:),'-ob');
        hold off;
        sgtitle('Projected 3D corners and patterns');
      end
      im_corners = pts_corner;
      cam_plane_coeff = plane_coeff;
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      % LCE-CALIB version 1
%       [imagePoints, boardSize] = detectCheckerboardPoints(img_raw);
%       if (boardSize(1) == 0 || boardSize(2) == 0)
%         continue;
%       end
%       if (boardSize(1) < numH || boardSize(2) < numW)
%         continue;
%       end      
%       [im_corners, cam_plane_coeff, pro_err] = ...
%         imgbor_ext(img_raw, K, D, pattern_size, borW, borH, debug_flag);
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      
      %% lidar: feature extraction      
%       [pts_bor, bor_coeff, err] = boardpts_ext(pc_array, borW, borH);
%       if (isempty(pts_bor))
%         continue;
%       end      
% %       pc_corners = borcorner_ext(pts_bor, borW, borH, debug_flag);
%       if (debug_flag)
%         figure; hold on;
%         pcshow(pointCloud(pts_bor(1:3, :)', 'Intensity', pts_bor(4, :)'));
% %         pcshow(pc_corners', [1,0,0], 'MarkerSize', 1000);
%         hold off;
%         sgtitle('Fitting planar points');
%       end
      
%       %% save feature extraction results
%       board_pts{end + 1} = pts_bor(1:3, :);
%       pcd_corner3D{end + 1} = pc_corners;
%       pc_bors_ceoff{end + 1} = bor_coeff;
%       img_corner3D{end + 1} = im_corners;
%       cam_bors_coeff{end + 1} = cam_plane_coeff;     

%       imgs_undist{end + 1} = img_undist;
%       pcs_array{end + 1} = pc_array;
%       
%       board_pts{end + 1} = pts_bor(1:3, :);
%       cam_bors_coeff{end + 1} = cam_plane_coeff;     
%       cam_bors_center{end + 1} = mean(pts_corner, 2)';
  end
  
  %% QPEP-pTop based extrinsic calibration
  aver_t_err = zeros(all_iterations, length(cam_bors_coeff));
  aver_r_err = zeros(all_iterations, length(cam_bors_coeff));
  Test_best = eye(4, 4);
  min_t = 1000;
  for frame_num = length(cam_bors_coeff)
    r_errs = zeros(1, all_iterations); 
    t_errs = zeros(1, all_iterations);
    for iter = 1:all_iterations    
      reidx = randperm(length(cam_bors_coeff));
      ref_pts = zeros(0, 3);
      ref_normals = zeros(0, 3);
      target_pts = zeros(0, 3);
      target_normals = zeros(0, 3);
      for idx = 1:frame_num
        bpts = board_pts{reidx(idx)};
        cbcoeff = cam_bors_coeff{reidx(idx)};
        cbcenter = cam_bors_center{reidx(idx)};
        for j = 1:length(bpts)
          ref_pts = [ref_pts; cbcenter];
          ref_normals = [ref_normals; cbcoeff(1:3)'];
          target_pts = [target_pts; bpts(:, j)'];          
          target_normals = [target_normals; [0 0 0]];
        end
      end
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Initialization
      % TODO:
      [R_cam_lidar, t_cam_lidar, ~, ~] = qpep_pTop(...
        target_pts, target_normals, ref_pts, ref_normals, false, false);
      Test = [R_cam_lidar, t_cam_lidar; 0 0 0 1];
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Refinement
      % TODO:
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      [r_err, t_err] = evaluateTFError(TGt, Test);
      r_errs(iter) = r_err;
      t_errs(iter) = t_err;
      if (t_err < min_t)
        min_t = t_err;
        Test_best = Test;
      end
      
      disp('TGt')
      disp(num2str(TGt, '%5f '))
      disp('Test')
      disp(num2str(Test, '%5f '))     
      debug_flag = 1;
      if debug_flag
        imshow(pt_project_depth2image(...
          Test, K, pcs_array{reidx(1)}, imgs_undist{reidx(1)}));
      end         
    end
    aver_t_err(:, frame_num) = t_errs';
    aver_r_err(:, frame_num) = r_errs';
    sprintf('Number of frames used for calibration: %d', frame_num)    
  end
%   save(fullfile(data_path, 'result_proposed.mat'), ...
%     'aver_r_err', 'aver_t_err', 'TOptm', ...
%     'board_pts', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff');

  %% Calibration
%   aver_t_err = zeros(all_iterations, length(cam_bors_coeff) - 1);
%   aver_r_err = zeros(all_iterations, length(cam_bors_coeff) - 1);
%   TOptm_best = eye(4, 4);
%   min_t = 1000;
% %   for PoseNum = 1:length(cam_bors_coeff) - 1
%   for PoseNum = length(cam_bors_coeff) - 1
%       r_errs = zeros(1, all_iterations); 
%       t_errs = zeros(1, all_iterations);
%       for iter = 1:all_iterations    
%         reidx = randperm(size(img_corner3D, 2));
%         sub_cam_corners3D = {};
%         sub_lidar_corners3D = {};
%         sub_pc_bors_coeff = {};
%         sub_cam_bors_coeff = {};
%         for idx = 1:PoseNum
%             sub_cam_corners3D{idx} = img_corner3D{reidx(idx)};
%             sub_lidar_corners3D{idx} = pcd_corner3D{reidx(idx)};
%             sub_pc_bors_coeff{idx} = pc_bors_ceoff{reidx(idx)};
%             sub_cam_bors_coeff{idx} = cam_bors_coeff{reidx(idx)};
%         end
% 
%         TInit = plane_init(sub_pc_bors_coeff, sub_cam_bors_coeff, ...
%           sub_lidar_corners3D,sub_cam_corners3D);
%         TOptm = corner_optm(sub_cam_corners3D, sub_lidar_corners3D, TInit);
%         
%         [r_err, t_err] = evaluateTFError(TGt, TOptm);
%         r_errs(iter) = r_err;
%         t_errs(iter) = t_err;
%         if (t_err < min_t)
%           min_t = t_err;
%           TOptm_best = TOptm;
%         end
%         
%         if debug_flag
%           [img_undist, camParams] = undistort_image(img_raw, K, D);        
%           imshow(pt_project_depth2image(TOptm, K, pc_array, img_undist));
%         end       
%       end
%       aver_t_err(:, PoseNum) = t_errs';
%       aver_r_err(:, PoseNum) = r_errs';
%       sprintf('PoseNum: %d', PoseNum)
%   end
%   TOptm = TOptm_best;
%   save(fullfile(data_path, 'result_proposed.mat'), ...
%     'aver_r_err', 'aver_t_err', 'TOptm', ...
%     'board_pts', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff');

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




