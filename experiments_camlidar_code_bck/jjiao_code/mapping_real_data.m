clc; 
clear;
close all;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

data_type = 'real_data';
data_fold = {'pose_data_1', 'pose_data_2'};
for data_option = 1:1
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
  all_pose = zeros(3, 0);
  
  %% Extract features
  pcd_path = fullfile(data_type, data_fold{data_option}, 'pcd');
  img_path = fullfile(data_type, data_fold{data_option}, 'img');
  img_list = dir(img_path);
  pcd_list = dir(pcd_path);
  pcd_list = pcd_list(3:end);
  img_list = img_list(3:end);

  map_points = zeros(3, 0);
  freq = 5;
%   for idx = 1:length(img_list)
  for idx = 1:10
    if (mod(idx, freq) ~= 0)
      continue;
    end
    sprintf('idx: %d', idx)
    img_file = strcat(img_list(idx).folder, '/', img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder, '/', pcd_list(idx).name);
    img_raw = imread(img_file);
    pc_raw = pcread(pcd_file);
    
    [imagePoints, boardSize] = detectCheckerboardPoints(img_raw);
    if (boardSize(1) == 0 || boardSize(2) == 0)
      continue;
    end
    if (boardSize(1) < numH || boardSize(2) < numW)
      continue;
    end
    
    IntrinsicMatrix = K';
    radialDistortion = [D(1),D(2),D(5)];
    tangentialDist = [D(3),D(4)];
    principalPoint =[K(1,3),K(2,3)];
    cam_param = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',...
                                  radialDistortion,'TangentialDistortion',tangentialDist);
    imageSize = [size(img_raw,1), size(img_raw,2)];
    im = undistortImage(img_raw, cam_param);
    worldPoints = generateCheckerboardPoints(boardSize, pattern_size);
    worldPoints = [worldPoints,zeros(size(worldPoints,1),1)];
    [worldOrientation,worldLocation] = estimateWorldCameraPose(imagePoints, worldPoints, cam_param);
    T = [[worldOrientation; worldLocation],[0,0,0,1]'];
    T_world_cam = T';
    T_world_lidar = T_world_cam * TGt;
%     T_world_lidar
    all_pose = [all_pose, T_world_cam(1:3, 4)];
    
    xyzPoints = pc_raw.Location';
    xyzPoints_world = T_world_lidar(1:3, 1:3) * xyzPoints + T_world_lidar(1:3, 4);
    map_points = [map_points, xyzPoints_world];
  end
  
  cloud_map = pointCloud(map_points');  
  pcshow(cloud_map);
end













