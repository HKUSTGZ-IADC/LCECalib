function run_lcecalib_fe()
  load('tmp_lcecalib_dataset.mat');
  
  all_cam_board_corners = {}; 
  all_cam_board_centers = {}; all_cam_board_centers_on_plane = {};
  all_cam_board_plane_coeff = {}; all_cam_board_plane_coeff_cov = {}; 
  all_lidar_board_pts = {}; all_lidar_board_pts_raw = {};
  all_lidar_board_edge_pts = {};
  all_lidar_board_corners = {}; all_lidar_board_plane_coeff = {};
  all_img_undist = {}; 
  all_lidar_pc_array = {}; all_lidar_pc_array_raw = {};
  for idx = 1:min(length(pcd_list), min(num_data, 60))
    if strcmp(data_type, 'fp_data')
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      if (~exist(pcd_file)) 
        continue;
      end
      ss = split(pcd_list(idx).name, '.pcd');
      img_file1 = strcat(fullfile(data_path, 'img'), '/', ss{1}, '.png');
      img_file2 = strcat(fullfile(data_path, 'img'), '/', ss{1}, '.jpg');
      img_file = img_file1;
      if (~exist(img_file))
        img_file = img_file2;
      end
      if (~exist(img_file))
        continue;
      end     
    else
      if ~(contains(img_list(idx).name, '.png') ...
        || contains(img_list(idx).name, '.jpg'))
        continue;
      end
      img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
      pcd_raw_file = strcat(pcd_raw_list(idx).folder, '/', pcd_raw_list(idx).name);
    end
      
    img_raw = imread(img_file);
    pc = pcread(pcd_file);
    lidar_pc_array = pc.Location()';
    pc_raw = pcread(pcd_raw_file);
    lidar_pc_array_raw = pc_raw.Location()';

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
      qpep_pnp(imagePoints, worldPoints, K', true, false);
    R_cam_world = R_cam_world'; 
    T_cam_world = [R_cam_world, t_cam_world; 0 0 0 1];
    %%%% check if QPEP has another PNP solution
    if t_cam_world(3) > 0
      T_cam_world = [R_cam_world, t_cam_world; 0 0 0 1];
    else
      eul = rotm2eul(R_cam_world, 'ZYX');
      R = eul2rotm([eul(1) - pi, -eul(2), -eul(3)], 'ZYX');
      T_cam_world = [R, -t_cam_world; 0 0 0 1];
    end
    T_qpep_pnp = T_cam_world;
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
    H = R_cam_world * skew([0 0 1]');
    covn = 4 * H * covR(2:4, 2:4) * H';
    cam_board_plane_coeff = [n; d];
    if cam_board_plane_coeff(4) < 0
        cam_board_plane_coeff = -cam_board_plane_coeff;
    end 

    % project camera checkerboard center onto the checkerboar plane
    cam_board_center = mean(cam_board_corners, 2);
    cam_board_center_on_plane = cam_board_center;
    cam_board_center_on_plane(3) = -((d + n(1:2)' * cam_board_center(1:2)) / n(3));

    debug_flag = 0;
    if debug_flag
      worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
      pc_corner_px = worldpts_to_cam(cam_board_corners, eye(3, 3), zeros(3, 1), K);
      pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);       
      fig = figure(1); 
      subplot(121); imshow(img_raw);
      subplot(122); imshow(img_undist);
      hold on;
      plot(worldPx(1,:), worldPx(2,:), '.r');
      plot(pc_corner_px(1,:), pc_corner_px(2,:), '-ob');
      hold off;
%       subplot(133); 
%       tmp_corners = cam_board_corners;
%       tmp_corners(:, size(cam_board_corners, 2) + 1) = cam_board_corners(:, 1);
%       plot3(tmp_corners(1, :), tmp_corners(2, :), tmp_corners(3, :), '-ob');
      sgtitle('Projected 3D corners and patterns');
%       close(fig);
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
    all_cam_board_centers{end + 1} = cam_board_center;
    all_cam_board_centers_on_plane{end + 1} = cam_board_center_on_plane;
    all_cam_board_plane_coeff{end + 1} = cam_board_plane_coeff;     
    all_cam_board_plane_coeff_cov{end + 1} = covn;

    all_lidar_board_pts_raw{end + 1} = lidar_board_pts_raw(1:3, :);
    all_lidar_board_pts{end + 1} = lidar_board_pts(1:3, :);
    all_lidar_board_edge_pts{end + 1} = lidar_board_edge_pts(1:3, :);
    all_img_undist{end + 1} = img_undist;
    all_lidar_pc_array{end + 1} = lidar_pc_array;   
    all_lidar_pc_array_raw{end + 1} = lidar_pc_array_raw;
  end  
  save('tmp_lcecalib_fe.mat');
%   save('tmp_lcecalib_fe.mat', ...
%     'all_cam_board_corners', ...  
%     'all_cam_board_centers', ...
%     'all_cam_board_centers_on_plane', ... 
%     'all_cam_board_plane_coeff', ...  
%     'all_cam_board_plane_coeff_cov', ...
%     'all_lidar_board_pts', ... 
%     'all_lidar_board_pts_raw', ... 
%     'all_lidar_board_edge_pts', ... 
%     'all_lidar_board_corners', ...  
%     'all_lidar_board_plane_coeff', ... 
%     'all_img_undist', ...
%     'all_lidar_pc_array');
end