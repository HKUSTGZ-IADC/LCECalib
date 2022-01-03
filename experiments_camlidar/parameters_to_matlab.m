clc, clear; 
close all;

%% simu data
K = [640.5099, 0, 640.500;
     0, 640.5099, 360.500;
     0, 0, 1];
D = [0, 0, 0, 0, 0];
borH = 0.63;
borW = 0.77;
pattern_size = 0.07;
TGt = [-0.27068     -0.94215     -0.19768      0.41853
        0.15218      0.16088     -0.97517     -0.30504
        0.95056     -0.29404     0.099833     -0.14758
              0            0            0            1];
squareSize = 70;
checkerboardPadding = [140, 140];
num_data = 36;
imageWidth = 1280;
imageHeight = 720;
save('simu_data/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight');

%% feiyi data
K = [897.4566, 0, 635.4040;
     0, 896.7992, 375.3149;
     0, 0, 1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
borH = 0.626;
borW = 0.767;
pattern_size = 0.07;
TGt = [0.77764     -0.62867    0.0066632   -0.0083745
       0.0063093   -0.0027942  -0.99998    -0.069025
       0.62867      0.77767    0.0017936   -0.084349
               0            0          0           1];
squareSize = 70; 
checkerboardPadding = [137, 136];
num_data = 12;
imageWidth = 1280;
imageHeight = 720;
% save('real_data/real_data_1/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');
% save('real_data/real_data_2/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');
% save('real_data/real_data_3/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');


%% jjh_data_20211129
K = [349.57, 0, 172.49;
     0, 349.18, 126.24;
     0, 0, 1];
D = [-0.36503, 0.18361, 0.00070794, 0.00010983, 0.000000];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
TGt = [0.012483     -0.99991    0.0033947      0.10578
       -0.018495   -0.0036253     -0.99982   -0.054982
        0.99975     0.012418    -0.018539    -0.065022
              0            0            0            1];
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 20;
imageWidth = 346;
imageHeight = 260;
numH = 7;
numW = 10;
save('real_data/real_data_4/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');
% save('real_data/real_data_5/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');


%% jjh_data_20211228
K = [304.98, 0, 174.83;
     0, 305, 126.7;
     0, 0, 1];
D = [-0.38527, 0.19819, 0.00093786, -0.00075714, 0.000000];
borH = 0.6;
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
TGt = [0.012483     -0.99991    0.0033947      0.10578
       -0.018495   -0.0036253     -0.99982   -0.054982
        0.99975     0.012418    -0.018539    -0.065022
              0            0            0            1];
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 63;
imageWidth = 346;
imageHeight = 260;
save('real_data/real_data_9/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');
% save('real_data/real_data_5/params.mat', 'K', 'D', 'borH', 'borW', 'pattern_size', 'squareSize', ...
%   'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight');
             
             
             





