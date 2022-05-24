function run_lcecalib_opt(all_iterations, edge_iterations, start_frame, end_frame)
  load('tmp_lcecalib_fe.mat');

  all_t_err = zeros(1, length(all_cam_board_plane_coeff));
  all_r_err = zeros(1, length(all_cam_board_plane_coeff));
  all_mp_err = zeros(1, length(all_cam_board_plane_coeff));
  all_eulerx = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_eulery = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_eulerz = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_tx = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_ty = zeros(all_iterations, length(all_cam_board_plane_coeff));
  all_tz = zeros(all_iterations, length(all_cam_board_plane_coeff));
  T_est_best = eye(4, 4);
  min_error = 1000;
  
  for frame_num = start_frame :min(length(all_cam_board_plane_coeff), end_frame)
    r_errs = zeros(1, all_iterations); 
    t_errs = zeros(1, all_iterations);
    all_eulers = zeros(3, all_iterations);
    all_tsl = zeros(3, all_iterations);
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
        cbcenter_on_plane = all_cam_board_centers_on_plane{reidx(idx)};
        for j = 1:length(lbpts)
          reference_pts = [reference_pts; cbcenter_on_plane'];
          reference_normals = [reference_normals; cbcoeff(1:3)'];
          target_pts = [target_pts; lbpts(:, j)'];          
          target_normals = [target_normals; [0 0 0]];
          weights = [weights; 1.0 / length(lbpts)];
        end
      end     
      % QPEP-PTop using only planar constraints
      [R_cam_lidar, t_cam_lidar, ~, ~, ~] = qpep_pTop(...
        target_pts, target_normals, ...
        reference_pts, reference_normals, ...
        false, false, weights);
      T_ini_qpep = [R_cam_lidar, t_cam_lidar; 0 0 0 1];
%       disp('T_ini_qpep')
%       disp(num2str(T_ini_qpep, '%5f '))
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Refinement
      T_ref_qpep = T_ini_qpep;
      for iter_ref = 1:edge_iterations
        reference_pts = zeros(0, 3);
        reference_normals = zeros(0, 3);
        target_pts = zeros(0, 3);
        target_normals = zeros(0, 3);        
        weights = zeros(0, 1);
        stds = zeros(0, 1);
        for idx = 1:frame_num
          lbpts = all_lidar_board_pts{reidx(idx)};
          lepts = all_lidar_board_edge_pts{reidx(idx)};
          cbcorner = all_cam_board_corners{reidx(idx)};
          cbcoeff = all_cam_board_plane_coeff{reidx(idx)};
          cbcenter_on_plane = all_cam_board_centers_on_plane{reidx(idx)};     
          cbcoeff_cov = all_cam_board_plane_coeff_cov{reidx(idx)};

          % set weights of edge and planar residuals
          r1 = 2.0 / size(lepts, 2);  % weights for edge residuals            
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
              stds = [stds; 0.0];
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
              weights = [weights; r2];              
              % compute the covariance of the point-to-plane residual
              p12 = T_ref_qpep(1:3, 1:3) * lbpts(:, j) + T_ref_qpep(1:3, 4) - cbcenter_on_plane;
              std = (p12' * cbcoeff_cov * p12)^(0.5);
              stds = [stds; std];
            end      
          end        
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
%       disp('T_est')
%       disp(num2str(T_est, '%5f ')) 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % Result Evaluation and Visualization
      all_eulers(:, iter) = rotm2eul(T_est(1:3, 1:3), 'ZYX');
      all_tsl(:, iter) = T_est(1:3, 4);

      debug_flag = 0;
      if debug_flag
        figure;
        subplot(121);
        imshow(projectPointOnImage(T_est, K, ...
          all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
        title('Projected points with Test', 'FontSize', 25);
        subplot(122);
        imshow(projectPointOnImage(TGt, K, ...
          all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)}));
        title('Projected points with TGt', 'FontSize', 25);
        figure;
        cloud_rgb = colorizePointFromImage(T_est, K, ...
          all_lidar_pc_array{reidx(1)}, all_img_undist{reidx(1)});
      end    
      debug_flag = 0;
      
      % add point-to-plane registration visualization
      debug_flag = 0;
      if debug_flag
        fig = figure; hold on;
        lbpts_cam = T_est(1:3, 1:3) * lbpts + T_est(1:3, 4);  % 3xN
        lepts_cam = T_est(1:3, 1:3) * lepts + T_est(1:3, 4);  % 3xN
        plot3(cbedge(1, :), cbedge(2, :), cbedge(3, :), 'g.'); 
        plot3(corre_cbedge(1, :), corre_cbedge(2, :), corre_cbedge(3, :), 'bo', 'MarkerSize', 12);
        plot3(lbpts_cam(1, :), lbpts_cam(2, :), lbpts_cam(3, :), 'r.', 'MarkerSize', 6);
        plot3(lepts_cam(1, :), lepts_cam(2, :), lepts_cam(3, :), 'ro', 'MarkerSize', 12);
        legend('cam edge pts', 'cam corre edge pts', 'lidar board pts', 'lidar edge pts');
        hold off;
        axis equal;
        view(40, 10);
        close(fig);
      end           
      debug_flag = 0;
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
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
    all_r_err(1, frame_num) = r_err;
    all_t_err(1, frame_num) = t_err;
    
    mp_err = zeros(1, length(all_cam_board_centers_on_plane));
    for i = 1:length(all_cam_board_centers_on_plane)
      [mp_err(i)] = evaluateMeanPlanarError(T_est, ...
        all_cam_board_plane_coeff{i}, ...
        all_lidar_board_pts{i});
    end
    if (mean(mp_err) < min_error)
      min_error = mean(mp_err);
      T_est_best = T_est;
    end
    mp_err
    all_mp_err(1, frame_num) = mean(mp_err);
    
%     if isequal(TGt, eye(4, 4))
%       [r_err, ~] = evaluateTFError(TGt, T_est);
%       if (r_err < min_error)
%         min_error = r_err;
%         T_est_best = T_est;
%       end 
%     else
%       all_mp_err = zeros(1, length(all_cam_board_centers_on_plane));
%       for i = 1:length(all_cam_board_centers_on_plane)
%         [mp_err] = evaluateMeanPlanarError(T_est, ...
%           all_cam_board_plane_coeff{i}, ...
%           all_lidar_board_pts{i});
%         all_mp_err(i) = mp_err;
%       end
%       if (mean(all_mp_err) < min_error)
%         min_error = mean(all_mp_err);
%         T_est_best = T_est;
%       end
%     end
    sprintf('Number of frames used for calibration: %d', frame_num)    
  end
  disp('T_est_best')
  disp(num2str(T_est_best, '%5f '))    
  disp('TGt')
  disp(num2str(TGt, '%5f '))    

  % output calibration errors
  [r_err, t_err] = evaluateTFError(TGt, T_est_best);
  sprintf('r_err: %f', r_err)
  sprintf('t_err: %f', t_err)    

  tmp_mp_err = zeros(1, length(all_cam_board_centers_on_plane));
  for i = 1:length(all_cam_board_centers_on_plane)
    [tmp_mp_err(i)] = evaluateMeanPlanarError(T_est_best, ...
      all_cam_board_plane_coeff{i}, ...
      all_lidar_board_pts{i});
  end
  sprintf('p_err: %f', mean(tmp_mp_err))
  
  save('tmp_lcecalib_opt.mat');
end




