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

%% generate real testing data
% qpep_data = load('qpep_api/qpep_data/qpep-data-pTop.mat');
% r0 = qpep_data.target_pts';
% nvr = qpep_data.target_normal(1:3, :)';
% b0 = qpep_data.ref_pts';
% nvb = qpep_data.ref_normal(1:3, :)';
% Rgt = qpep_data.Rgt;
% tgt = qpep_data.tgt;
% 
% len = length(r0);
% R0 = Rgt;
% t0 = tgt;
% q0 = dcm2quat(R0).';
% if(q0(1) < 0)
%     q0 = - q0;
% end

%% set the ground-truth rotation and translation
m = 10;
[X, Y] = meshgrid(linspace(-2, 2, m), linspace(-2, 2, m));
X = reshape(X, 1, []);
Y = reshape(Y, 1, []);
Z = sin(X) .* cos(Y) .* cos(X);
len = m^2;

R0 = orthonormalize(randn(3, 3));
q0 = dcm2quat(R0).';
if(q0(1) < 0)
    q0 = - q0;
end
t0 = randn(3, 1);
X0 = [R0, t0;
     zeros(1, 3), 1];
 
r0 = [X; Y; Z].';
b0 = zeros(len, 3);
noise = 5e-100;
for i = 1 : len
    b0(i, :) = (R0 * r0(i, :).' + t0 + noise * randn(3, 1)).' + noise * randn(1, 3);
end
nvr = pcnormals(pointCloud(r0));   % target point cloud
nvb = pcnormals(pointCloud(b0));   % reference point cloud (provide normal)

%% report dataset information
J1 = J_func_pTop(q0, t0, r0, b0, nvr);  
J2 = J_func_pTop(q0, t0, r0, b0, nvb);  % residual function

Ttrue = eye(4, 4);
Ttrue(1:3, 1:3) = R0;
Ttrue(1:3, 4) = t0;
disp('Ttrue:')
disp(num2str(Ttrue, '%.5f '))

%% QPEP-PTOP
[Rest, test, covR, covt] = qpep_pTop(r0, nvr, b0, nvb, true);
Test = eye(4, 4);
Test(1:3, 1:3) = Rest;
Test(1:3, 4) = test;
disp('Test:')
disp(num2str(Test, '%.5f '))

%% evaluation
[r_err, t_err] = evaluateTFError(Ttrue, Test);
disp('Rotation error, translation error:')
disp(num2str([r_err, t_err ], '%.6f '))
