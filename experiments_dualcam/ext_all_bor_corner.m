clc;
clear;

addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");


K1 =[640.5098521801531, 0.0, 640.5; 0.0, 640.5098521801531, 360.5; 0.0, 0.0, 1.0];
D1 = [0.0, 0.0, 0.0, 0.0, 0.0];
K2 =[640.5098521801531, 0.0, 640.5; 0.0, 640.5098521801531, 360.5; 0.0, 0.0, 1.0];
D2 = [0.0, 0.0, 0.0, 0.0, 0.0];
borW=0.77;
borH=0.63;
img_path1 = "/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-camera/noise-0.007/img1";
img_path2 = "/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-camera/noise-0.007/img2";
img_list1 = dir(img_path1);
img_list2 = dir(img_path2);
img_list1 = img_list1(3:end);
img_list2 = img_list2(3:end);

img_corner3D1 = {};
cam_bors_coeff1={};
img_corner3D2 = {};
cam_bors_coeff2={};
pro_err1s =[];
pro_err2s =[];
for idx = 1:size(img_list1,1)
    % 输入数据
    img_file1 = strcat(img_list1(idx).folder,'/',img_list1(idx).name);
    img_raw1 = imread(img_file1);
    img_file2 = strcat(img_list2(idx).folder,'/',img_list2(idx).name);
    img_raw2 = imread(img_file2);
    im_corners1_iter=[];
    im_corners2_iter=[];
    cam_plane_coeff1_iter=[];
    cam_plane_coeff2_iter=[];
    [im_corners1,cam_plane_coeff1,pro_err1] = imgbor_ext(img_raw1,K1,D1,0.07,borW,borH);
    [im_corners2,cam_plane_coeff2,pro_err2] = imgbor_ext(img_raw2,K2,D2,0.07,borW,borH);
    % save data
    img_corner3D1{idx} = im_corners1;
    cam_bors_coeff1{idx} = cam_plane_coeff1;
    img_corner3D2{idx} = im_corners2;
    cam_bors_coeff2{idx} = cam_plane_coeff2;
    pro_err1s=[pro_err1s,pro_err1];
    pro_err2s=[pro_err2s,pro_err2];
    
end