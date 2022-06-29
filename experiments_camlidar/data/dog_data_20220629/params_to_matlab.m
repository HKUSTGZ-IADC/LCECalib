clc, clear; close all;

%% fp_data_20220629 frame_cam00
K = [6.05128601e+02, 0., 5.21453430e+02;                                % TODO  
     0., 6.04974060e+02, 3.94878479e+02;
     0., 0., 1. ];
D = [-9.19851288e-02, 8.66983905e-02, 2.48622790e-04, 7.21565739e-04];  % TODO
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
TGt = eye(4, 4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 30;
imageWidth = 1024;   % TODO
imageHeight = 768;   % TODO
numH = 7;
numW = 10;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('dog_data_20220629_1/img/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', ...
  'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

