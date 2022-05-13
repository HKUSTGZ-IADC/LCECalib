clc, clear;
close all;

warning('off');
format long g

libQPEP_dir = '../../LibQPEP/MATLAB';
addpath(fullfile(libQPEP_dir, 'func_files'));
addpath(fullfile(libQPEP_dir, 'solvers'));
addpath(fullfile(libQPEP_dir, 'utils'));
addpath(fullfile(libQPEP_dir, 'calib'));
addpath(fullfile(libQPEP_dir, 'sdpt3'));
run(fullfile(libQPEP_dir, 'sdpt3', 'install_sdpt3.m'))
addpath(fullfile(libQPEP_dir, 'sedumi'));
if(~verLessThan('matlab', '8.0.0'))  
  run(fullfile(libQPEP_dir, 'sedumi', 'install_sedumi.m'))
end
p = genpath(fullfile(libQPEP_dir, 'YALMIP'));
addpath(p);

addpath('qpep_api');
addpath('eval');

%% generate testing data
load realsense-calib
if(~verLessThan('matlab', '8.0.0'))
    K = cam_params_realsense.IntrinsicMatrix;
    camParams = cam_params_realsense;
end

width = 1280;
height = 720;

% load zed-M-calib-left
% cam_params_left = cam_params_zed_M_left;
% K = cam_params_left.IntrinsicMatrix;
% camParams = cam_params_left;
% width = 1280;
% height = 720;

len = 88;
max_iter = 10000;
found = false;
while(true)
    iter = 0;
    counter = 0;
    image_pt0 = zeros(len, 2);
    world_pt0 = zeros(len, 3);
    R0 = orthonormalize(randn(3, 3));
    q0 = dcm2quat(R0).';
    if(q0(1) < 0)
        q0 = - q0;
    end
    t0 = randn(3, 1);
    X0 = inv([R0, t0;
        zeros(1, 3), 1]);
    while(true)
        world_pt_ = randn(1, 3);
        [image_pt_, s] = generateProjectedPoints_(world_pt_, K, R0, t0);
        if(s > 0 && 0 < image_pt_(1) && image_pt_(1) < width && ...
                 0 < image_pt_(2) && image_pt_(2) < height)
            counter = counter + 1;
            image_pt0(counter, :) = image_pt_;
            world_pt0(counter, :) = world_pt_;
        end
    
        if(counter >= len)
            if(abs(J_pnp_loss(image_pt0, world_pt0, K, R0, t0)) < 1e-14)
                found = true;
            end
            break;
        end
        
        if(iter >= max_iter)
            break;
        end
        iter = iter + 1;
    end
    
    if(found)
        break;
    end
end

T_true = eye(4, 4);
T_true(1:3, 1:3) = R0;
T_true(1:3, 4) = t0;
disp('T_true:')
disp(num2str(T_true, '%.5f '))

J_pnp_loss(image_pt0, world_pt0, K, R0, t0)

%% QPEP-PNP
[Rest, test, covR, covt] = qpep_pnp(image_pt0, world_pt0, K, true);
Test = eye(4, 4);
Test(1:3, 1:3) = Rest;
Test(1:3, 4) = test;
disp('T_est:')
disp(num2str(Test, '%.5f '))












