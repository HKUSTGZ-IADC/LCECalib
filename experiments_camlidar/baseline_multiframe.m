clc, clear;
close all;

% data_type = 'simu_data';
% data_fold = {'noise-free', 'noise-0.015', 'noise-0.03'};
% removeground = true;

data_type = 'real_data';
data_fold = {'real_data_1', 'real_data_2', 'real_data_3'};
removeground = true;

data_type = 'real_data';
data_fold = {'real_data_4', 'real_data_5', 'real_data_6', 'real_data_7', 'real_data_8', 'real_data_9'};
removeground = false;

for data_option = 3:6
  sprintf('data_option: %d', data_option)
  %% parameters
  imagePath = fullfile(data_type, data_fold{data_option}, 'img');
  ptCloudPath = fullfile(data_type, data_fold{data_option}, 'pcd');
  imds = imageDatastore(imagePath); 
  pcds = fileDatastore(ptCloudPath, 'ReadFcn', @pcread); 
  
  intrinsic = load(fullfile(data_type, data_fold{data_option}, 'calibration.mat'));
  params = load(fullfile(data_type, data_fold{data_option}, 'params.mat'));
  squareSize = params.squareSize; 
  checkerboardPadding = params.checkerboardPadding;
  TGt = params.TGt;
  all_iterations = 100;

  for i = 1:params.num_data
    imageFileNames{i} = imds.Files{i};
    ptCloudFileNames{i} = pcds.Files{i};
  end

  rng('default');
  
  %% Extract features
  % Extract Checkerboard corners from the images
  [imageCorners3d, checkerboardDimension, dataUsed] = ...
      estimateCheckerboardCorners3d(imageFileNames, intrinsic.cameraParams, squareSize);
  % helperShowImageCorners(imageCorners3d, imageFileNames, intrinsic.cameraParams)
  
  imageFileNames = imageFileNames(dataUsed); % Remove image files that are not used
  % Filter point cloud files corresponding to the detected images
  ptCloudFileNames = ptCloudFileNames(dataUsed);
  % Extract ROI from the detected image corners
  roi = helperComputeROI(imageCorners3d, 3);
  
  %Extract Checkerboard in lidar data
  checkerboard3DDimension = checkerboardDimension + checkerboardPadding;
  [lidarCheckerboardPlanes, framesUsed, indices] = detectRectangularPlanePoints(...
    ptCloudFileNames, checkerboard3DDimension, ...
    'RemoveGround', removeground, 'ROI', roi, 'DimensionTolerance', 0.2);
  % helperShowCheckerboardPlanes(ptCloudFileNames, indices)  
  
  %% Calibration
  avail_ptCloudFileNames = ptCloudFileNames(framesUsed);
  avail_imageCorners3d = imageCorners3d(:, :, framesUsed);
  avail_imageFileNames = imageFileNames(framesUsed);
  avail_lidarCheckerPlanes = lidarCheckerboardPlanes;
  
  aver_t_err = [];
  aver_r_err = [];
  TOptm_best = eye(4, 4);
  min_t = 1000;
  for poseNum = 1:length(avail_ptCloudFileNames) - 1
    t_errs= [];
    multi_theta_errs=[];
    for iter = 1:all_iterations
      reidx = randperm(length(avail_ptCloudFileNames));
      reidx = reidx(1:poseNum);
      tmp_ptCloudFileNames = avail_ptCloudFileNames(reidx);
      tmp_imageCorners3d = avail_imageCorners3d(:, :, reidx);
      tmp_imageFileNames = avail_imageFileNames(reidx);
      tmp_lidarCheckerPlanes = avail_lidarCheckerPlanes(reidx);
  
      [tform, errors] = estimateLidarCameraTransform(tmp_lidarCheckerPlanes, ...
          tmp_imageCorners3d, 'CameraIntrinsic', intrinsic.cameraParams);
    %   helperFuseLidarCamera(tmp_imageFileNames, tmp_ptCloudFileNames, indices, intrinsic.cameraParams, tform);
      
      % save MATLAB calibration results
      % matlab's tform format is the transpose of traditional format
      % MATLAB: p_trans = p * tform.Rotation + tform.Translation is a 1x3 vector. 
      % Ours: p_trans = tform.Rotation * p + tform.Translation is a 3x1 vector
      TOptm = eye(4, 4);
      TOptm(1:3, 1:3) = tform.Rotation';
      TOptm(1:3, 4) = tform.Translation';
%       TOptm
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
    aver_t_err = [aver_t_err,t_errs'];
    aver_r_err = [aver_r_err,multi_theta_errs'];
    sprintf('poseNum: %d', poseNum)
  end
  TOptm = TOptm_best;
  save(fullfile(data_type, data_fold{data_option}, 'result_baseline.mat'), 'aver_r_err', 'aver_t_err', 'TOptm');
  
  %% plot results
%   figure; boxplot(aver_r_err(:, 3:end));
%   xlabel("Number of Poses"); title("Rotation Error [deg]");
%   grid on;
%   ax = gca;
%   ax.GridLineStyle = '--';
%   ax.GridAlpha = 0.3;
%   set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
%   title('Mean and Median Rotation Error', 'FontSize', 30, 'FontWeight', 'normal');
%   box on;
%   
%   figure; boxplot(aver_t_err(:, 3:end));
%   xlabel("Number of Poses"); ylabel("Translation Error [m]");
%   grid on;
%   ax = gca;
%   ax.GridLineStyle = '--';
%   ax.GridAlpha = 0.3;
%   set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
%   title('Mean and Median Translation Error', 'FontSize', 30, 'FontWeight', 'normal');
%   box on;
end

























