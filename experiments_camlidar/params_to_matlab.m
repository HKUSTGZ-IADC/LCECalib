clc, clear; 
close all;

%% simu_data_bias_1 - simu_data_bias_10
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
numH = 7;
numW = 9;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 10.0;
planar_weight = 1.0;
save('data/simu_data_bias/simu_data_bias_1/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_2/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_3/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_4/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_5/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_6/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_7/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_8/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_9/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data_bias/simu_data_bias_10/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% simu_data_1 - simu_data_2
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
numH = 7;
numW = 9;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 10.0;
planar_weight = 1.0;
save('data/simu_data/simu_data_1/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data/simu_data_2/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/simu_data/simu_data_3/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', 'checkerboardPadding', 'TGt', ...
  'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% real_data_1 - real-data_3
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
num_data = 40;
imageWidth = 1280;
imageHeight = 720;
numH = 7;
numW = 9;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('data/real_data/real_data_1/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_2/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_3/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% %% real_data_4 - real-data_5
% TMECH version1
K = [349.57, 0, 172.49;
     0, 349.18, 126.24;
     0, 0, 1];
D = [-0.36503, 0.18361, 0.00070794, 0.00010983, 0.000000];
% TMECH version2
% K = [346.1763         0  175.6199;
%             0  346.4199  131.0226;
%             0         0    1.0000];
% D = [-0.3697 0.1870 0.0005 -0.0013 0.1332];
borH = 0.6;
borW = 0.8;
pattern_size = 0.068;
% TMECH version1
% TGt = [0.012483     -0.99991    0.0033947      0.10578
%        -0.018495   -0.0036253     -0.99982   -0.054982
%         0.99975     0.012418    -0.018539    -0.065022
%               0            0            0            1];
% TMECH version2
TGt = [0.0175   -0.9998    0.0005    0.0939
      -0.0054   -0.0006   -1.0000   -0.0926
       0.9998    0.0175   -0.0054   -0.1082
            0         0         0    1.0000];
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 30;
imageWidth = 346;
imageHeight = 260;
numH = 7;
numW = 10;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 10.0;
planar_weight = 1.0;
save('data/real_data/real_data_4/img/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', ...
  'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_5/img/params.mat', 'K', 'D', 'borH', 'borW', 'numH', 'numW', ...
  'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% real_data_6 - real-data_9
% TMECH version1
% K = [304.98, 0, 174.83;
%      0, 305, 126.7;
%      0, 0, 1];
% D = [-0.38527, 0.19819, 0.00093786, -0.00075714, 0.000000];
% TMECH version2
K = [305.5716         0  175.2111;
            0  305.6182  125.4881;
            0         0    1.0000];
D = [-0.3879, 0.2354, 8.7038e-04, -7.5971e-04, -0.1027];
borH = 0.6; 
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
% TMECH version1
% TGt = [0.012483     -0.99991    0.0033947      0.10578
%        -0.018495   -0.0036253     -0.99982   -0.054982
%         0.99975     0.012418    -0.018539    -0.065022
%               0            0            0            1];
% TMECH version2
TGt = [ 0.0093   -0.9999    0.0113    0.1231
       -0.0115   -0.0114   -0.9999   -0.0648
        0.9999    0.0091   -0.0116   -0.1030
             0         0         0    1.0000];
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 30;
imageWidth = 346;
imageHeight = 260;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 10.0;
planar_weight = 1.0;
save('data/real_data/real_data_6/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_7/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_8/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
save('data/real_data/real_data_9/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', 'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% fp_data/fp_data_1/frame_img
K = [6.05128601e+02, 0, 5.21453430e+02;
     0, 6.04974060e+02, 3.94878479e+02;
     0, 0, 1];
D = [-9.19851288e-02, 8.66983905e-02, 2.48622790e-04, 7.21565739e-04];
borH = 0.6;
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
TGt = eye(4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 14;
imageWidth = 1024;
imageHeight = 768;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('data/fp_data/fp_data_1/frame_img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');
             
%% fp_data/fp_data_1/event_image
K = [2.91047882e+02, 0., 1.63948074e+02;
     0, 2.91041321e+02, 1.34054352e+02;
     0, 0, 1];
D = [-3.63157451e-01, 1.46164939e-01, -4.12534573e-04, -8.07056727e-04];
borH = 0.6;
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
TGt = eye(4);
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 35;
imageWidth = 346;
imageHeight = 260;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
save('data/fp_data/fp_data_1/event_img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% fp_data/fp_data_2 --- frame_cam00
K = [6.05128601e+02, 0, 5.21453430e+02;
     0, 6.04974060e+02, 3.94878479e+02;
     0, 0, 1];
D = [-9.19851288e-02, 8.66983905e-02, 2.48622790e-04, 7.21565739e-04];
borH = 0.6;
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 14;
imageWidth = 1024;
imageHeight = 768;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
TGt = [-0.0235   -0.9996   -0.0153    0.1107
        0.0098    0.0151   -0.9998   -0.0489
        0.9997   -0.0237    0.0095   -0.0477
             0         0         0    1.0000];
save('data/fp_data/fp_data_2/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%% fp_data/fp_data_4 --- event_cam00
K = [ 2.91047882e+02, 0., 1.63948074e+02, 
      0., 2.91041321e+02, 1.34054352e+02, 
      0., 0., 1. ];
D = [ -3.63157451e-01, 1.46164939e-01, -4.12534573e-04, -8.07056727e-04 ];
borH = 0.6;
borW = 0.8;
numH = 7;
numW = 10;
pattern_size = 0.068;
squareSize = 68;
checkerboardPadding = [120, 124];
num_data = 12;
imageWidth = 1024;
imageHeight = 768;
use_edge_flag = 1; 
use_planar_flag = 1;
edge_weight = 3.0;
planar_weight = 1.0;
TGt = eye(4, 4);
save('data/fp_data/fp_data_4/img/params.mat', 'K', 'D', 'borH', 'borW', ...
  'numH', 'numW', 'pattern_size', 'squareSize', ...
  'checkerboardPadding', 'TGt', 'num_data', ...
  'imageWidth', 'imageHeight', ...
  'use_edge_flag', 'use_planar_flag', ...
  'edge_weight', 'planar_weight');

%%




