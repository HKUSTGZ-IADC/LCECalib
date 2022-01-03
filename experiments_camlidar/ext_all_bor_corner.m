clc;
clear;

addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%% parameters
params = load('simu_data/params.mat');
borW = params.borW;
borH = params.borH;
pattern_size = params.pattern_size;
K = params.K;
D = params.D;
pcd_path = "/home/jjiao/Dropbox/calibration_result/simulated_data/noise-free/pcd";
img_path = "/home/jjiao/Dropbox/calibration_result/simulated_data/noise-free/img";

%%
img_list = dir(img_path);
pcd_list = dir(pcd_path);
pcd_list = pcd_list(3:end);
img_list = img_list(3:end);

img_corner3D = {};
pcd_corner3D = {};
pc_bors_ceoff = {};
cam_bors_coeff = {};
for idx = 1:size(pcd_list,1)
    % 输入数据
    img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
    img_raw = imread(img_file);
    pc_raw = pcread(pcd_file);
    pc_array = pc_raw.Location()';
    
    [pts_bor, bor_coeff, err] = boardpts_ext(pc_array,borW,borH);  %% extract board points
    pc_corners = borcorner_ext(pts_bor,borW,borH);  %% extract board corners
    [im_corners, cam_plane_coeff, pro_err] = imgbor_ext(img_raw,K,D,pattern_size,borW,borH);
    
    % save data
    pcd_corner3D{idx} = pc_corners;
    pc_bors_ceoff{idx} = bor_coeff;
    img_corner3D{idx} = im_corners;
    cam_bors_coeff{idx} = cam_plane_coeff;
end