clc; clear; close all;

add_path_lcecalib;
add_path_qpep;
load('color_list.mat');
format short

data_path = 'data_bag_process/LE_calib_20211126_RI';
data_type = {'e2calib_1_1/e2calib', 'e2calib_1_5/e2calib', ...
  'e2calib_1_10/e2calib', 'e2calib_1_20/e2calib', ...
  'e2calib_1_30/e2calib', 'e2calib_1_40/e2calib', ...
  'e2calib_1_50/e2calib', 'e2calib_1_75/e2calib', ...
  'e2calib_1_100/e2calib', 'e2calib_1_150/e2calib', ...
  'e2calib_1_200/e2calib'};
label = {'1000ms', '200ms', '100ms', '50ms', '33.3ms', '25ms', '20ms', ...
  '13.3ms', '10ms', '6.7ms', '5ms'};

params = load(fullfile(data_path, 'params.mat'));
borW = params.borW; borH = params.borH; 
numW = params.numW; numH = params.numH;
pattern_size = params.pattern_size;
K = params.K; D = params.D;

% data_type_1   x 
% data_type_2   x
% data_type_3   x
%              img_1 img_2 img3
all_R = cell(length(data_type), 111);
all_t = cell(length(data_type), 111);

%% detect checkboard points
for data_option = 1:length(data_type)
  sprintf('data_option: %d', data_option)
  path = fullfile(data_path, data_type{data_option});
  if (~exist(path)) 
    continue;
  end 
  
  img_list = dir(path); 
  for idx = 1:111
    img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
    if ~(contains(img_list(idx).name, '.png') ...
      || contains(img_list(idx).name, '.jpg'))
      continue;
    end
    img_raw = imread(img_file);
    
    %% image: feature extraction
    [img_undist, camParams] = undistort_image(img_raw, K, D);
    [imagePoints, boardSize] = detectCheckerboardPoints(img_undist);
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    T = T_qpep_pnp;
    all_R{data_option, idx} = T(1:3, 1:3);
    all_t{data_option, idx} = T(1:3, 4);
    % estimate checkerboard corners
    cam_board_corners = mean(worldPoints, 1)' + ...
      [[borW/2,borH/2,0]', [borW/2,-borH/2,0]',...
      [-borW/2,-borH/2,0]', [-borW/2,borH/2,0]'];
    cam_board_corners = T(1:3, 1:3) * cam_board_corners + T(1:3, 4);
    
    debug_flag = 1;
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
      sgtitle('Projected 3D corners and patterns');
      close(fig);
    end
    
    save_flag = 1;
    if save_flag
      worldPx = worldpts_to_cam(worldPoints', T(1:3, 1:3), T(1:3, 4), K);
      pc_corner_px = worldpts_to_cam(cam_board_corners, eye(3, 3), zeros(3, 1), K);
      pc_corner_px(:, size(pc_corner_px, 2) + 1) = pc_corner_px(:, 1);       
      fig = figure(1); 
      imshow(img_undist);
      hold on;
      plot(worldPx(1,:), worldPx(2,:), '.r', 'MarkerSize', 20);
      plot(pc_corner_px(1,:), pc_corner_px(2,:), '-ob', 'MarkerSize', 12);
      hold off;
      img_filename = fullfile(img_list(idx).folder, '../', ...
        sprintf('%d_%s.png', idx, label{data_option}));
      saveas(fig, img_filename);
      close(fig);
    end
  end
end

%% result counting
total_img = 111;
all_detect_flag = cell(length(data_type), 111);
data_detect_cnt = zeros(length(data_type), 1);
for j = 1:111
  for i = 1:length(data_type)
    if (isempty(all_t{i, j}))
      all_detect_flag{i, j} = 0;
    else
      all_detect_flag{i, j} = 1;
      data_detect_cnt(i) = data_detect_cnt(i) + 1;
    end
  end
end
figure, 
plot(data_detect_cnt / total_img * 100, 'ro-', 'Color', color_list(1, :), ...
  'LineWidth', 2, 'MarkerSize', 15);
xticks(1:length(data_type));
xticklabels(label);
xlabel("Time Interval"); ylabel("Checkboard Detection Ratio [%]");
xlim([0.5, 11.5]); ylim([40, 85]);
grid on;
ax = gca; ax.GridLineStyle = '--'; ax.GridAlpha = 0.3;
set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5, 'YScale', 'log');
box on;    

sgtitle('Checkboard Detection Ratio With Different Time Interval', 'FontSize', 30, ...
  'FontName', 'Times', 'FontWeight', 'normal');    

%%
save(fullfile(data_path, 'result_test_event_img_quality.mat'), ...
  'all_R', 'all_t', 'all_detect_flag', 'data_detect_cnt', ...
  'params');




