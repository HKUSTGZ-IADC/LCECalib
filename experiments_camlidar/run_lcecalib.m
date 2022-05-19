clc; clear; close all;

add_path_lcecalib;
add_path_qpep;

format short

data_type = 'real_data';
% data_type = 'simu_data';

visualization_flag = 0;
debug_flag = 0;
save_result_flag = 0;
plot_result_flag = 0;

%% load data and extract features
for data_option = 1:1
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  all_iterations = 10;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  

  all_cam_board_corners = {}; all_cam_board_centers = {};
  all_cam_board_plane_coeff = {}; all_cam_board_plane_coeff_cov = {}; 
  all_lidar_board_pts = {}; all_lidar_board_pts_raw = {};
  all_lidar_board_edge_pts = {};
  all_lidar_board_corners = {}; all_lidar_board_plane_coeff = {};
  all_img_undist = {}; all_lidar_pc_array = {};
  for idx = 1:min(60, num_data)
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      lidar_pc_array = pc_raw.Location()';
      
      %% image: feature extraction
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
      if (boardSize(1) == 0 || boardSize(2) == 0 || ...
          boardSize(1) ~= numH || boardSize(2) ~= numW)
        continue;
      end
      worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
      worldPoints = [worldPoints, zeros(size(worldPoints,1),1)];

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % QPEP-PnP
      [R_cam_world, t_cam_world, covR, covt] = ...
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
      if debug_flag
        disp('T_qpep_pnp')
        disp(num2str(T_qpep_pnp, '%5f '))
      end
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      T = T_qpep_pnp;
      
      % estimate checkerboard corners
      cam_board_corners = mean(worldPoints, 1)' + ...
        [[borW/2,borH/2,0]', [borW/2,-borH/2,0]',...
        [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
      cam_board_corners = T(1:3, 1:3) * cam_board_corners + T(1:3, 4);
      n = T(1:3,1:3) * [0,0,1]';
      d = -(n' * T(1:3,4));
      covn = 4 *...
        (R_cam_world * skew([0,0,1]')) * covR(2:4, 2:4) * (R_cam_world * skew([0,0,1]'))';
      cam_board_plane_coeff = [n; d];
      if cam_board_plane_coeff(4) < 0
          cam_board_plane_coeff = -cam_board_plane_coeff;
      end 
      if debug_flag
        worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
        pc_corner_px = worldpts_to_cam(cam_board_corners, eye(3, 3), zeros(3, 1), K);
        pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);       
        figure; 
        subplot(121); imshow(img_raw);
        subplot(122); imshow(img_undist);
        hold on;
        plot(worldPx(1,:), worldPx(2,:), '.r');
        plot(pc_corner_px(1,:), pc_corner_px(2,:), '-ob');
        hold off;
        sgtitle('Projected 3D corners and patterns');
      end
     
      %% lidar: feature extraction     
      % extract board points
      [lidar_board_pts, lidar_board_plane_coeff, err] = ...
        boardpts_ext(lidar_pc_array, borW, borH, data_type);
      lidar_board_pts_raw = lidar_board_pts;
      if (isempty(lidar_board_pts))
        continue;
      end
      % extract edge points
      lidar_board_edge_pts_idx = find_pts_ring_edges(lidar_board_pts);
      lidar_board_edge_pts = lidar_board_pts(:, lidar_board_edge_pts_idx);
      
      % project planar points and edge points onto plane to reduce noise
      n = lidar_board_plane_coeff(1:3);
      d = lidar_board_plane_coeff(4);
      pts_on_plane = lidar_board_edge_pts(1:3, 1);
      pts_on_plane(3) = -((d + n(1:2)' * pts_on_plane(1:2)) / n(3));
      
      for ptid = 1 : size(lidar_board_pts, 2)
        pt = lidar_board_pts(1:3, ptid);
        v = pt - pts_on_plane;
        pt_onboard = pts_on_plane + v - (v' * n * n);
        lidar_board_pts(1:3, ptid) = pt_onboard;
      end
      
      for ptid = 1 : size(lidar_board_edge_pts, 2)
        pt = lidar_board_edge_pts(1:3, ptid);
        v = pt - pts_on_plane;
        pt_onboard = pts_on_plane + v - (v' * n * n);
        lidar_board_edge_pts(1:3, ptid) = pt_onboard;
      end

      %% save feature extraction results
      all_cam_board_corners{end + 1} = cam_board_corners;
      all_cam_board_centers{end + 1} = mean(cam_board_corners, 2);
      all_cam_board_plane_coeff{end + 1} = cam_board_plane_coeff;     
      all_cam_board_plane_coeff_cov{end + 1} = covn;
      
      all_lidar_board_pts_raw{end + 1} = lidar_board_pts_raw(1:3, :);
      all_lidar_board_pts{end + 1} = lidar_board_pts(1:3, :);
      all_lidar_board_edge_pts{end + 1} = lidar_board_edge_pts(1:3, :);
      all_img_undist{end + 1} = img_undist;
      all_lidar_pc_array{end + 1} = lidar_pc_array;   
  end
  
  %% QPEP-pTop based extrinsic calibration
  aver_t_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  aver_r_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  T_est_best = eye(4, 4);
  min_t = 1000;
%   for frame_num = length(all_cam_board_plane_coeff)
  for frame_num = 9:length(all_cam_board_plane_coeff)
    r_errs = zeros(1, all_iterations); 
    t_errs = zeros(1, all_iterations);
    for iter = 1:all_iterations  % multiple iterations
      reidx = randperm(length(all_cam_board_plane_coeff));
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Initialization
      reference_pts = zeros(0, 3);
      reference_normals = zeros(0, 3);
      target_pts = zeros(0, 3);
      target_normals = zeros(0, 3);
      weights = zeros(0, 1);
      for idx = 1:frame_num
        lbpts = all_lidar_board_pts{reidx(idx)};
        cbcoeff = all_cam_board_plane_coeff{reidx(idx)};
        cbcenter = all_cam_board_centers{reidx(idx)};
        for j = 1:length(lbpts)
          reference_pts = [reference_pts; cbcenter'];
          reference_normals = [reference_normals; cbcoeff(1:3)'];
          target_pts = [target_pts; lbpts(:, j)'];          
          target_normals = [target_normals; [0 0 0]];
          weights = [weights; 1.0 / length(lbpts)];
        end
      end     
      % QPEP-PTop using only planar constraints
      [R_cam_lidar, t_cam_lidar, ~, ~] = qpep_pTop(...
        target_pts, target_normals, ...
        reference_pts, reference_normals, ...
        false, false, weights);
      T_ini_qpep = [R_cam_lidar, t_cam_lidar; 0 0 0 1];
      disp('T_ini_qpep')
      disp(num2str(T_ini_qpep, '%5f '))
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Refinement
      T_ref_qpep = T_ini_qpep;
      if num_data >= 15
        use_edge_flag = 0;
      else
        use_edge_flag = 1;
      end
      use_planar_flag = 1;
      for iter_ref = 1:5
        reference_pts = zeros(0, 3);
        reference_normals = zeros(0, 3);
        target_pts = zeros(0, 3);
        target_normals = zeros(0, 3);        
        weights = zeros(0, 1);
        for idx = 1:frame_num
          lbpts = all_lidar_board_pts{reidx(idx)};
          lepts = all_lidar_board_edge_pts{reidx(idx)};
          cbcorner = all_cam_board_corners{reidx(idx)};
          cbcoeff = all_cam_board_plane_coeff{reidx(idx)};
          cbcenter = all_cam_board_centers{reidx(idx)};          

          % set weights of edge and planar residuals
          r1 = 1.0 / size(lepts, 2);  % weights for edge residuals            
%           r2 = 30 * 1.0 / length(lbpts);   % weights for planar residuals
          r2 = 1.0 / length(lbpts);   % weights for planar residuals
          
          %%%%% add edge residuals
          [cbedge, cbedge_dir] = generateBoardPtsFromCorner(cbcorner);
          lepts_cam = T_ref_qpep(1:3, 1:3) * lepts + T_ref_qpep(1:3, 4);  % 3xN
          corre_idx = knnsearch(cbedge(1:3, :)', lepts_cam', 'K', 1);
          corre_cbedge = cbedge(:, corre_idx); % 4xN
          if use_edge_flag
            for j = 1:size(lepts_cam, 2)
              p = lepts_cam(:, j);
              p1 = cbedge_dir(4:6, corre_cbedge(4, j));
              p2 = p1 + cbedge_dir(1:3, corre_cbedge(4, j));
              p1p2 = p1 - p2;
              p1p = p1 - p;
              p2p = p2 - p;
              n1 = cross(p1p, p2p);
              n1 = n1 / norm(n1);
              n2 = cross(n1, p1p2);
              n2 = n2 / norm(n2);
              reference_pts = [reference_pts; cbedge(1:3, corre_idx(j))'];
              reference_normals = [reference_normals; n2'];
              target_pts = [target_pts; lepts(:, j)'];
              target_normals = [target_normals; [0 0 0]];
              weights = [weights; r1];
            end   
            debug_flag = 0;
            if debug_flag
              fig = figure; hold on;
              plot3(cbedge(1, :), cbedge(2, :), cbedge(3, :), 'g*'); 
              corr_pts = cbedge(1:3, corre_idx)';
              plot3(corr_pts(:, 1), corr_pts(:, 2), corr_pts(:, 3), 'ro', 'MarkerSize', 12);
              plot3(lepts_cam(1, :), lepts_cam(2, :), lepts_cam(3, :), 'bo', 'MarkerSize', 12);
              legend('cam edge pts', 'cam corre edge pts', 'lidar edge pts');
              hold off;
              axis equal;
              view(40, 10);
              close(fig);
            end
          end
          %%%%% add planar residuals
          if use_planar_flag
            for j = 1:length(lbpts)
              reference_pts = [reference_pts; cbcenter'];
              reference_normals = [reference_normals; cbcoeff(1:3)'];
              target_pts = [target_pts; lbpts(:, j)'];          
              target_normals = [target_normals; [0 0 0]];
              weights = [weights; r2];
            end      
          end        
        end
        % QPEP-PTop using both edge and planar constraints
        [R_cam_lidar, t_cam_lidar, ~, ~] = qpep_pTop(...
          target_pts, target_normals, ...
          reference_pts, reference_normals, ...
          false, false, weights);
        T_ref_qpep = [R_cam_lidar, t_cam_lidar; 0 0 0 1];
        disp('T_ref_qpep')
        disp(num2str(T_ref_qpep, '%5f '))
        disp('TGt')
        disp(num2str(TGt, '%5f '))
      end
      % After multiple iterations of refinement
      T_est = T_ref_qpep;
      disp('T_est')
      disp(num2str(T_est, '%5f ')) 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      [r_err, t_err] = evaluateTFError(TGt, T_est);
      r_errs(iter) = r_err; t_errs(iter) = t_err;
      if (t_err < min_t)
        min_t = t_err;
        T_est_best = T_est;
      end
      
      debug_flag = 1;
      if debug_flag
        figure;
        imshow(pt_project_depth2image(T_est, K, ...
          all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
        title('Projected points with Test', 'FontSize', 25);
        figure;
        imshow(pt_project_depth2image(TGt, K, ...
          all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
        title('Projected points with TGt', 'FontSize', 25);
      end         
      
      % TODO: add point-to-plane registration visualization
      debug_flag = 1;
      if debug_flag
        fig = figure; hold on;
        lbpts_cam = T_est(1:3, 1:3) * lbpts + T_est(1:3, 4);  % 3xN
        lepts_cam = T_est(1:3, 1:3) * lepts + T_est(1:3, 4);  % 3xN
        plot3(cbedge(1, :), cbedge(2, :), cbedge(3, :), 'g*'); 
        plot3(lbpts_cam(1, :), lbpts_cam(2, :), lbpts_cam(3, :), 'r.', 'MarkerSize', 6);
        plot3(lepts_cam(1, :), lepts_cam(2, :), lepts_cam(3, :), 'bo', 'MarkerSize', 12);
        legend('cam edge pts', 'cam corre edge pts', 'lidar edge pts');
        hold off;
        axis equal;
        view(40, 10);
        reclose(fig);
      end              
    end
    aver_t_err(:, frame_num) = t_errs';
    aver_r_err(:, frame_num) = r_errs';
    sprintf('Number of frames used for calibration: %d', frame_num)    
    disp('T_est_best')
    disp(num2str(T_est_best, '%5f '))    
    disp('TGt')
    disp(num2str(TGt, '%5f '))    
  end
  
  if save_result_flag
    save(fullfile(data_path, 'result_lcecalib_qpep.mat'), ...
      'aver_r_err', 'aver_t_err', 'T_est_best', ...
      'all_cam_board_corners', 'all_cam_board_plane_coeff', ...
      'all_cam_board_centers', ...
      'all_lidar_board_pts', 'all_lidar_board_edge_pts');

    save(fullfile(data_path, 'result_lcecalib_qpep_sensor_data.mat'), ...
      'aver_r_err', 'aver_t_err', 'T_est_best', ...
      'all_cam_board_corners', 'all_cam_board_plane_coeff', ...
      'all_cam_board_centers', ...
      'all_lidar_board_pts', 'all_lidar_board_edge_pts', ...
      'all_img_undist', 'all_lidar_pc_array');
  end

  %% Plot results
  plot_result_flag = 1;
  if plot_result_flag
    figure; 
    subplot(211); boxplot(aver_r_err(:, 5: end));
    xlabel("Number of Poses"); ylabel("Rotation Error [deg]");
    grid on;
    ax = gca;
    ax.GridLineStyle = '--';
    ax.GridAlpha = 0.3;
    set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
    box on;

    subplot(212); boxplot(aver_t_err(:, 5: end));
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
end




