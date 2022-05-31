function run_lcecalib_opt(all_iterations, edge_iterations, start_frame, end_frame)
  load('tmp_lcecalib_fe.mat');

  all_t_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_r_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_mp_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_me_err = zeros(all_iterations, length(all_cam_board_plane_coeff));
  select_t_err = zeros(1, length(all_cam_board_plane_coeff));
  select_r_err = zeros(1, length(all_cam_board_plane_coeff));
  select_mp_err = zeros(1, length(all_cam_board_plane_coeff));  
  select_me_err = zeros(1, length(all_cam_board_plane_coeff));  
  all_eulerx = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_eulery = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_eulerz = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_tx = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_ty = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_tz = zeros(all_iterations, length(all_cam_board_plane_coeff));

  %%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Initialization
  T_ini_best = eye(4, 4);
  min_error = 1e5;
  if strcmp(data_type, 'simu_data_bias')
    r = 1.0;
  else
    r = 0.6;
  end
  for frame_num = floor(length(all_cam_board_plane_coeff) * r) ...
      :min(length(all_cam_board_plane_coeff), end_frame)  
    for iter = 1:5  % multiple iterations to try different combinations
      reidx = randperm(length(all_cam_board_plane_coeff));
      reference_pts = zeros(0, 3);
      reference_normals = zeros(0, 3);
      target_pts = zeros(0, 3);
      target_normals = zeros(0, 3);
      point_cnt = 0;
      for idx = 1:frame_num
        lbpts = all_lidar_board_pts{reidx(idx)};
        cbcoeff = all_cam_board_plane_coeff{reidx(idx)};
        cbcenter_on_plane = all_cam_board_centers_on_plane{reidx(idx)};
        for j = 1:length(lbpts)
          reference_pts = [reference_pts; cbcenter_on_plane'];
          reference_normals = [reference_normals; cbcoeff(1:3)'];
          target_pts = [target_pts; lbpts(:, j)'];          
          target_normals = [target_normals; [0 0 0]];
          point_cnt = point_cnt + 1;
        end
      end
      weights = ones(point_cnt, 1) / point_cnt;
      
      % QPEP-PTop using only planar constraints
      [R_cam_lidar, t_cam_lidar, ~, ~, ~] = qpep_pTop(...
        target_pts, target_normals, ...
        reference_pts, reference_normals, ...
        false, false, weights);
      T_ini_qpep = [R_cam_lidar, t_cam_lidar; 0 0 0 1];
      
      p_err = zeros(4, length(all_cam_board_centers_on_plane));
      for i = 1:length(all_cam_board_centers_on_plane)
        [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_ini_qpep, ...
          all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
      end
      mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
      if (mp_err < min_error)
        min_error = mp_err;
        T_ini_best = T_ini_qpep;
      end
    end
  end
  disp('T_ini_best')
  disp(num2str(T_ini_best, '%5f '))  
  [r_err, t_err] = evaluateTFError(T_ini_best, TGt);
  p_err = zeros(4, length(all_cam_board_centers_on_plane));
  for i = 1:length(all_cam_board_centers_on_plane)
    [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_ini_best, ...
      all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
    [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(T_ini_best, ...
      all_cam_board_corners{i}, all_lidar_board_edge_pts{i});        
  end
  mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
  me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean edge error      
  sprintf('r_err: %f, t_err: %f, mp_err: %f, me_err: %f, total_err: %f of initial estimated TF', ...
    r_err, t_err, mp_err, me_err, mp_err + me_err)  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Refinement
  T_est_best = T_ini_best;
  min_error = 1e5;
  for frame_num = start_frame : min(length(all_cam_board_plane_coeff), end_frame)
    r_errs = zeros(1, all_iterations); 
    t_errs = zeros(1, all_iterations);
    mp_errs = zeros(1, all_iterations);
    me_errs = zeros(1, all_iterations);
    
    all_eulers = zeros(3, all_iterations);
    all_tsl = zeros(3, all_iterations);
    for iter = 1:all_iterations  % multiple iterations to try different combinations
      reidx = randperm(length(all_cam_board_plane_coeff));
      T_ref_qpep = T_ini_best;
      for iter_edge = 1:edge_iterations
        reference_pts = zeros(0, 3);
        reference_normals = zeros(0, 3);
        target_pts = zeros(0, 3);
        target_normals = zeros(0, 3);        
        stds = zeros(0, 1);
        weights = zeros(1, 0);
        point_cnt = 0;
        for idx = 1:frame_num
          lbpts = all_lidar_board_pts{reidx(idx)};
          lepts = all_lidar_board_edge_pts{reidx(idx)};
          cbcorner = all_cam_board_corners{reidx(idx)};
          cbcoeff = all_cam_board_plane_coeff{reidx(idx)};
          cbcenter_on_plane = all_cam_board_centers_on_plane{reidx(idx)};     
          cbcoeff_cov = all_cam_board_plane_coeff_cov{reidx(idx)};

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
              
%               reference_pts = [reference_pts; corre_cbedge(1:3, j)'];
%               reference_normals = [reference_normals; n1'];
%               target_pts = [target_pts; lepts(:, j)'];
%               target_normals = [target_normals; [0 0 0]];
              
              reference_pts = [reference_pts; corre_cbedge(1:3, j)'];
              reference_normals = [reference_normals; n2'];
              target_pts = [target_pts; lepts(:, j)'];
              target_normals = [target_normals; [0 0 0]];
              
              stds = [stds; 0.0];
              point_cnt = point_cnt + 1;
              if (strcmp(data_type, 'simu_data_bias'))
                weights = [weights, edge_weight];
              else
                weights = [weights, edge_weight / size(lepts, 2)];
              end              
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
              reference_pts = [reference_pts; cbcenter_on_plane'];
              reference_normals = [reference_normals; cbcoeff(1:3)'];
              target_pts = [target_pts; lbpts(:, j)'];          
              target_normals = [target_normals; [0 0 0]];
              % compute the covariance of the point-to-plane residual
              p12 = T_ref_qpep(1:3, 1:3) * lbpts(:, j) + T_ref_qpep(1:3, 4) - cbcenter_on_plane;
              std = (p12' * cbcoeff_cov * p12)^(0.5);
              stds = [stds; std];
              point_cnt = point_cnt + 1;
              if (strcmp(data_type, 'simu_data_bias'))
                weights = [weights, planar_weight];
              else
                weights = [weights, planar_weight / length(lbpts)];
              end
            end      
          end               
        end
        if (strcmp(data_type, 'simu_data_bias'))
          weights = weights / point_cnt;
        end
        % remove outlier data
        stds_sort = sort(stds, 'ascend');
        std_threshold = stds_sort(floor(length(stds_sort) * 0.9));
        weights(stds >= std_threshold) = 0;

        % QPEP-PTop using both edge and planar constraints
        [R_cam_lidar, t_cam_lidar, ~, ~, ~] = qpep_pTop(...
          target_pts, target_normals, ...
          reference_pts, reference_normals, ...
          false, false, weights);
        T_ref_qpep = [R_cam_lidar, t_cam_lidar; 0 0 0 1];      
      end
      % After multiple iterations of refinement
      T_est = T_ref_qpep;
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Result Evaluation and Visualization
      all_eulers(:, iter) = rotm2eul(T_est(1:3, 1:3), 'ZYX');
      all_tsl(:, iter) = T_est(1:3, 4);

      [r_errs(iter), t_errs(iter)] = evaluateTFError(TGt, T_est);
      p_err = zeros(4, length(all_cam_board_centers_on_plane));
      for i = 1:length(all_cam_board_centers_on_plane)
        [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est, ...
          all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
        [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(T_est, ...
          all_cam_board_corners{i}, all_lidar_board_edge_pts{i});        
      end
      mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
      me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean edge error
      mp_errs(iter) = mp_err;      
      me_errs(iter) = me_err;      
      
%       debug_flag = 0;
%       if debug_flag
%         figure;
%         subplot(121);
%         imshow(projectPointOnImage(T_est, K, ...
%           all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
%         title('Projected points with Test', 'FontSize', 25);
%         subplot(122);
%         imshow(projectPointOnImage(TGt, K, ...
%           all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
%         title('Projected points with TGt', 'FontSize', 25);
%         figure;
%         cloud_rgb = colorizePointFromImage(T_est, K, ...
%           all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)});
%       end    
%       debug_flag = 0;
%       
%       % add point-to-plane registration visualization
%       debug_flag = 0;
%       if debug_flag
%         fig = figure; hold on;
%         lbpts_cam = T_est(1:3, 1:3) * lbpts + T_est(1:3, 4);  % 3xN
%         lepts_cam = T_est(1:3, 1:3) * lepts + T_est(1:3, 4);  % 3xN
%         plot3(cbedge(1, :), cbedge(2, :), cbedge(3, :), 'g.'); 
%         plot3(corre_cbedge(1, :), corre_cbedge(2, :), corre_cbedge(3, :), 'bo', 'MarkerSize', 12);
%         plot3(lbpts_cam(1, :), lbpts_cam(2, :), lbpts_cam(3, :), 'r.', 'MarkerSize', 6);
%         plot3(lepts_cam(1, :), lepts_cam(2, :), lepts_cam(3, :), 'ro', 'MarkerSize', 12);
%         legend('cam edge pts', 'cam corre edge pts', 'lidar board pts', 'lidar edge pts');
%         hold off;
%         axis equal;
%         view(40, 10);
%         close(fig);
%       end           
%       debug_flag = 0;
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    end
    all_r_err(:, frame_num) = r_errs';
    all_t_err(:, frame_num) = t_errs';
    all_mp_err(:, frame_num) = mp_errs';    
    all_me_err(:, frame_num) = me_errs';    
    
    all_eulerz(:, frame_num) = all_eulers(1, :) / pi * 180;
    all_eulery(:, frame_num) = all_eulers(2, :) / pi * 180;
    all_eulerx(:, frame_num) = all_eulers(3, :) / pi * 180;
    all_tx(:, frame_num) = all_tsl(1, :);
    all_ty(:, frame_num) = all_tsl(2, :);
    all_tz(:, frame_num) = all_tsl(3, :);
    quat = quaternion(all_eulers' / pi * 180, 'eulerd', 'ZYX', 'frame');
    quatAverage = meanrot(quat);
    tslAverage = mean(all_tsl, 2);
    T_est = eye(4, 4);
    T_est(1:3, 1:3) = quat2rotm(quatAverage);
    T_est(1:3, 4) = tslAverage;
    
    % compute tf error and mean planar error
    [r_err, t_err] = evaluateTFError(TGt, T_est);
    p_err = zeros(4, length(all_cam_board_centers_on_plane));
    for i = 1:length(all_cam_board_centers_on_plane)
      [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(T_est, ...
        all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
      [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(T_est, ...
        all_cam_board_corners{i}, all_lidar_board_edge_pts{i});
    end
    mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
    me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean edge error
    if (mp_err + me_err < min_error)
      min_error = mp_err + me_err;
      T_est_best = T_est;
    end
    sprintf('r_err: %f, t_err: %f, mp_err: %f, me_err: %f, total_err: %f', ...
      r_err, t_err, mp_err, me_err, mp_err + me_err)  
    sprintf('Number of frames used for calibration: %d', frame_num) 
    select_r_err(1, frame_num) = r_err;
    select_t_err(1, frame_num) = t_err;
    select_mp_err(1, frame_num) = mp_err;
    select_me_err(1, frame_num) = me_err;
  end
  disp('T_est_best')
  disp(num2str(T_est_best, '%5f '))    
  disp('TGt')
  disp(num2str(TGt, '%5f '))    
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  % output calibration errors
  [r_err, t_err] = evaluateTFError(TGt, T_est_best);
  tmp_p_err = zeros(4, length(all_cam_board_centers_on_plane));
  for i = 1:length(all_cam_board_centers_on_plane)
    [tmp_p_err(1, i), tmp_p_err(2, i)] = evaluateTotalPlanarError(T_est_best, ...
      all_cam_board_plane_coeff{i}, all_lidar_board_pts{i});
    [tmp_p_err(3, i), tmp_p_err(4, i)] = evaluateTotalEdgeError(T_est, ...
      all_cam_board_corners{i}, all_lidar_board_edge_pts{i});    
  end
  tmp_mp_err = sum(tmp_p_err(1, :)) / sum(tmp_p_err(2, :));  % mean edge error
  tmp_me_err = sum(tmp_p_err(3, :)) / sum(tmp_p_err(4, :));  % mean planar error  
  sprintf('r_err: %f, t_err: %f, mp_err: %f, me_err: %f, total_err: %f of final estimated TF', ...
    r_err, t_err, tmp_mp_err, tmp_me_err, tmp_mp_err + tmp_me_err)  
  save('tmp_lcecalib_opt.mat');
end




