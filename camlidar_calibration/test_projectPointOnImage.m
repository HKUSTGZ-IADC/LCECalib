clc, clear; close all;

addpath('tools');
%%
data_path = fullfile('data', 'fp_data_20220424', strcat('fp_data_20220424', '_', num2str(1)));
params = load(fullfile(data_path, 'img/params.mat'));
result_qpep = load(fullfile(data_path, 'result_lcecalib_qpep_sensor_data.mat'));

data_path = fullfile('data', 'fp_data_20220424', strcat('fp_data_20220424', '_', num2str(3)));
img_filename = fullfile(data_path, 'img', '000000.png');
pcd_filename = fullfile(data_path, 'pcd', '000000_crop.pcd');
img = imread(img_filename);
[img_undist, ~] = undistort_image(img, params.K, params.D);
pc = pcread(pcd_filename);
ptCloudOut = pcdownsample(pc, 'random', 1.0);

figure;
imshow(projectPointOnImage(result_qpep.T_est_best, params.K, ...
  ptCloudOut.Location', img_undist));
title('Projected points with Test', 'FontSize', 25);

%%
% ptCloudOut = pcdownsample(pc, 'random', 0.5);