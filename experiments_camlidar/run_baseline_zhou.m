clc, clear; close all;

addpath('evaluation');

all_data_type = {'simu_data_bias', 'real_data'};
% all_data_type = {'real_data'};
% all_data_type = {'simu_data_bias'};

removeground = true;
rng('default');

for i = 1:length(all_data_type)
data_type = all_data_type{i};
for data_option = 1:10
  sprintf('data_option: %d', data_option)
  %% parameters
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));  
  if (~exist(data_path)) 
    continue;
  end
  data_qpep = load(fullfile(data_path, 'result_lcecalib_qpep.mat'));
  
  intrinsic = load(fullfile('data', data_type, 'baseline_zhou_calibration.mat'));
  params = load(fullfile('data', data_type, 'baseline_zhou_params.mat'));
  squareSize = params.squareSize; 
  checkerboardPadding = params.checkerboardPadding;
  TGt = params.TGt;

  imagePath = fullfile(data_path, 'img');
  ptCloudPath = fullfile(data_path, 'pcd');
  imds = imageDatastore(imagePath); 
  pcds = fileDatastore(ptCloudPath, 'ReadFcn', @pcread); 
  for i = 1:min(length(imds.Files), params.num_data)
    imageFileNames{i} = imds.Files{i};
    ptCloudFileNames{i} = pcds.Files{i};
  end 
  
  %% Extract features
  % Extract Checkerboard corners from the images
  [imageCorners3d, checkerboardDimension, dataUsed] = ...
      estimateCheckerboardCorners3d(imageFileNames, intrinsic.cameraParams, squareSize);
%   helperShowImageCorners(imageCorners3d, imageFileNames, intrinsic.cameraParams)
  
  imageFileNames = imageFileNames(dataUsed); % Remove image files that are not used
  % Filter point cloud files corresponding to the detected images
  ptCloudFileNames = ptCloudFileNames(dataUsed);
  % Extract ROI from the detected image corners
  roi = helperComputeROI(imageCorners3d, 3);
  
  % Extract Checkerboard in lidar data
  checkerboard3DDimension = checkerboardDimension + checkerboardPadding;
  [lidarCheckerboardPlanes, framesUsed, indices] = detectRectangularPlanePoints(...
    ptCloudFileNames, checkerboard3DDimension, ...
    'RemoveGround', removeground, ...
    'ROI', roi, ...
    'DimensionTolerance', 0.4, ...
    'Verbose', true);
%   helperShowCheckerboardPlanes(ptCloudFileNames, indices)  

  %% Extract planes from lidar point cloud
  all_lidar_board_plane_coeff = cell(1, length(framesUsed));
  for i = 1:length(framesUsed)
    if (framesUsed(i) == 0)
      all_lidar_board_plane_coeff{i} = [];
    else
      plane = lidarCheckerboardPlanes(sum(framesUsed(1:i)));
    end
    planeModel = pcfitplane(plane, 0.1);
    all_lidar_board_plane_coeff{i} = planeModel.Parameters';
  end
  % Fit plane
  %   plane = lidarCheckerboardPlanes(1);
  %   rectModel = lidar.internal.calibration.fitRectangle3D(plane, 'O', 'YPR', 'Iterations', 30);
  %   cuboidCorners = rectModel.getCornerPoints();
  %   dim = rectModel.Dimensions;
  %   planeModel = pcfitplane(plane, 0.1);
  %   normal = correctNormal(planeModel.Normal, mean(plane.Location));
  %   corners(:, :, i) = get4Corners(cuboidCorners, normal, dim);  
  %   planeModel = pcfitplane(lidarCheckerboardPlanes(1), 0.1);    

  %% Calibration
  avail_ptCloudFileNames = ptCloudFileNames(framesUsed);
  avail_imageCorners3d = imageCorners3d(:, :, framesUsed);
  avail_imageFileNames = imageFileNames(framesUsed);
  avail_lidarCheckerPlanes = lidarCheckerboardPlanes;
  
  all_t_err = [];
  all_r_err = [];
  all_mp_err = [];
  all_me_err = [];    
  T_est_best = eye(4, 4);
  min_error = 1e5;
  all_iterations = 100;
  for frame_num = 1:length(avail_ptCloudFileNames)
    t_errs = [];
    r_errs = [];
    mp_errs = [];
    me_errs = [];
    for iter = 1:all_iterations
      reidx = randperm(length(avail_ptCloudFileNames));
      tmp_ptCloudFileNames = avail_ptCloudFileNames(reidx(1:frame_num));
      tmp_imageCorners3d = avail_imageCorners3d(:, :, reidx(1:frame_num));
      tmp_imageFileNames = avail_imageFileNames(reidx(1:frame_num));
      tmp_lidarCheckerPlanes = avail_lidarCheckerPlanes(reidx(1:frame_num));
      [tform, errors] = estimateLidarCameraTransform(tmp_lidarCheckerPlanes, ...
          tmp_imageCorners3d, 'CameraIntrinsic', intrinsic.cameraParams);
      % helperFuseLidarCamera(tmp_imageFileNames, tmp_ptCloudFileNames, indices, intrinsic.cameraParams, tform);
      
      % save MATLAB calibration results
      % matlab's tform format is the transpose of traditional format
      % MATLAB: p_trans = p * tform.Rotation + tform.Translation is a 1x3 vector. 
      % Ours: p_trans = tform.Rotation * p + tform.Translation is a 3x1 vector
      Test = eye(4, 4);
      Test(1:3, 1:3) = tform.Rotation';
      Test(1:3, 4) = tform.Translation';

      % evaluation
      [r_errs(end + 1), t_errs(end + 1)] = evaluateTFError(TGt, Test);
      p_err = zeros(4, length(data_qpep.all_cam_board_centers_on_plane));
      for i = 1:length(data_qpep.all_cam_board_centers_on_plane)
        [p_err(1, i), p_err(2, i)] = evaluateTotalPlanarError(Test, ...
          data_qpep.all_cam_board_plane_coeff{i}, data_qpep.all_lidar_board_pts{i});
        [p_err(3, i), p_err(4, i)] = evaluateTotalEdgeError(Test, ...
          data_qpep.all_cam_board_corners{i}, data_qpep.all_lidar_board_edge_pts{i});       
      end
      mp_err = sum(p_err(1, :)) / sum(p_err(2, :));  % mean planar error
      me_err = sum(p_err(3, :)) / sum(p_err(4, :));  % mean edge error
      if (mp_err + me_err < min_error)
        min_error = mp_err + me_err;
        T_est_best = Test;
      end  
      mp_errs(end + 1) = mp_err;
      me_errs(end + 1) = me_err;
    end
    all_r_err = [all_r_err, r_errs'];
    all_t_err = [all_t_err, t_errs'];    
    all_mp_err = [all_mp_err, mp_errs'];
    all_me_err = [all_me_err, me_errs'];
    sprintf('frame_num: %d', frame_num)
  end
  disp('T_est_best')
  disp(num2str(T_est_best, '%5f '))    
  disp('TGt')
  disp(num2str(TGt, '%5f '))    
  save(fullfile(data_path, 'result_baseline_zhou.mat'), ...
    'all_r_err', 'all_t_err', 'all_mp_err', 'all_me_err', ...
    'T_est_best', ...
    'all_lidar_board_plane_coeff');
  save(fullfile(data_path, 'result_baseline_zhou_sensor_data.mat'), ...
    'all_r_err', 'all_t_err', 'all_mp_err', 'all_me_err', ...
    'T_est_best', ...
    'all_lidar_board_plane_coeff', ...
    'avail_lidarCheckerPlanes');  
end
end



