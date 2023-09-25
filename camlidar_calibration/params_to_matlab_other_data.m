clc, clear; 
close all;

%% mini_hercules_data_20221205/mini_hercules_data_20221205_1 --- frame_cam00
% K = [607.7607         0  509.9821;
%      0  608.1210  389.9726;
%      0         0    1.0000];
% D = [-0.1026 0.1200 8.0769e-04 -8.1645e-04 -0.0435];
% borH = 0.495;  % m
% borW = 0.595;  % m
% numH = 7;
% numW = 9;
% pattern_size = 0.060;  % m
% squareSize = 60;       % mm
% checkerboardPadding = [115, 135]; %mm
% num_data = 54;
% imageWidth = 1024;
% imageHeight = 768;
% use_edge_flag = 1; 
% use_planar_flag = 1;
% edge_weight = 3.0;
% planar_weight = 1.0;
% TGt = eye(4, 4);
% save('data/mini_hercules_data_20221205/mini_hercules_data_20221205_1/img/params.mat', ...
%   'K', 'D', 'borH', 'borW', ...
%   'numH', 'numW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', ...
%   'imageWidth', 'imageHeight', ...
%   'use_edge_flag', 'use_planar_flag', ...
%   'edge_weight', 'planar_weight');

%% mini_hercules_data_20221205/mini_hercules_data_20221205_2 --- frame_cam01
% K = [ 2.91047882e+02, 0., 1.63948074e+02, 
%       0., 2.91041321e+02, 1.34054352e+02, 
%       0., 0., 1. ];
% D = [ -3.63157451e-01, 1.46164939e-01, -4.12534573e-04, -8.07056727e-04 ];
% borH = 0.495;  % m
% borW = 0.595;  % m
% numH = 6;
% numW = 8;
% pattern_size = 0.060;  % m
% squareSize = 60;       % mm
% checkerboardPadding = [115, 135]; %mm
% num_data = 30;
% imageWidth = 720;
% imageHeight = 540;
% use_edge_flag = 1; 
% use_planar_flag = 1;
% edge_weight = 3.0;
% planar_weight = 1.0;
% TGt = eye(4, 4);
% save('data/mini_hercules_data_20221205/mini_hercules_data_20221205_2/img/params.mat', ...
%   'K', 'D', 'borH', 'borW', ...
%   'numH', 'numW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', ...
%   'imageWidth', 'imageHeight', ...
%   'use_edge_flag', 'use_planar_flag', ...
%   'edge_weight', 'planar_weight');

%% rdmapping
K = [ 1413.30786, 0.0,       983.3288858;
      0.0,       1409.7222,  587.50149407;
      0.0,       0.0,           1.0];
D = [ -0.2173951, 0.1539990288, -0.001093152, 0.001120104, -0.04912207];
imageWidth = 1920;
imageHeight = 1200;
save('/Spy/dataset/rdmapping/20230901_w1_building_filter/params.mat', ...
  'K', 'D', 'imageWidth', 'imageHeight');

