clc;
clear;
addpath(genpath(pwd));

%% param init
Refdata = importdata('.\data\pair_1_0.txt');
Movdata = importdata('.\data\pair_1_1.txt');
Refdata = Refdata';
Movdata = Movdata';

Tf0 = eye(4);
Kernel = 'L2';
% Kernel = 'L1';
% Kernel = 'Welsch';

%% process
[R,T] = f_irls_icp(Refdata, Movdata, Tf0, Kernel);