clc, clear; close all;

%% apollo_data_1
K = 1.0e+03 * ...
    [3.8662         0    0.9403;
          0    3.8656    0.4974;
          0         0    0.0010];
D = [1.4883  7.3876 -0.0010 0.0024 -7.9213];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
% TGt = [0.0122   -0.9999   -0.0010    0.1115
%       -0.0052    0.0009   -1.0000   -0.0943
%        0.9999    0.0122   -0.0052   -0.1082
%             0         0         0    1.0000];
TGt = eye(4, 4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 36;
imageWidth = 346;
imageHeight = 260;
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
K = 1.0e+03 * ...
    [1.9458         0    0.9297;
          0    1.9419    0.4944;
          0         0    0.0010];
D = [-0.4181    0.1718  1.0e-03 * 0.2959  1.0e-03 * -0.7913  -0.0685];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
TGt = eye(4, 4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 36;
imageWidth = 346;
imageHeight = 260;
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
