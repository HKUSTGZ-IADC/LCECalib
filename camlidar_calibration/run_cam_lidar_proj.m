clc, clear; close all;

add_path_lcecalib;
load('color_list.mat');
format short

%% NOTE(googjjh): Please modify parameters here
data_path = 'data_demo/cam_lidar_proj';
sequence_name = '20220216_garden_day';
img = imread(fullfile(data_path, sequence_name, '000000.png'));
cloud = pcread(fullfile(data_path, sequence_name, '000000.pcd'));

%%%%%% camera intrinsics
K = [ 6.05128601e+02, 0., 5.20453430e+02;
      0., 6.04974060e+02, 3.93878479e+02;
      0., 0., 1. ];
D = [ -9.19851288e-02, 8.66983905e-02, 2.48622790e-04, 7.21565739e-04 ];

%%%%%% camera-lidar extrinsics
quat_L_C = [0.5054, -0.4888, 0.5074, -0.4982];
t_L_C = [0.0419, 0.0739, -0.0399];
tf_L_C = [quat2rotm(quat_L_C), t_L_C'; 0 0 0 1];
tf_C_L = tf_L_C^(-1);

%% projection
max_range = 100.0;
xyz = cloud.Location;
xyz_filter = xyz(sqrt(xyz(:, 1).^2 + xyz(:, 2).^2 + xyz(:, 1).^2) < max_range, :);
[img_undist, camParams] = undistort_image(img, K, D);
figure;
imshow(projectPointOnImage(tf_C_L, K, xyz_filter(1:2:end, :)', img_undist));
title('Projected points with Extrinsics', 'FontSize', 25);
