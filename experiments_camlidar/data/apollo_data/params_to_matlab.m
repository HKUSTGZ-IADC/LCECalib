clc, clear; close all;

%% apollo_data_1
K = [3961.082370, 0.000000, 963.839055;
     0.000000, 3984.084420, 620.555243;
     0.000000, 0.000000, 1.000000];
D = [0 0 0 0 0 0];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
TGt = eye(4, 4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 36;
imageWidth = 1920;
imageHeight = 1080;
numH = 7;
numW = 10;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('apollo_data_1/img/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', ...
  'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% apollo_data_2
K = [1996.241246, 0.000000, 953.758716; 
     0.000000, 1993.372273, 500.599692;
     0.000000, 0.000000, 1.000000];
D = [0 0 0 0 0];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
TGt = eye(4, 4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 36;
imageWidth = 1920;
imageHeight = 1080;
numH = 7;
numW = 10;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('apollo_data_2/img/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', ...
  'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
