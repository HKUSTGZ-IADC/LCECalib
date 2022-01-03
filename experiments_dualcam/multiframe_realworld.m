clc;
clear;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%parameters
K1= [ 825.6334,0,639.9610;0,824.9260,384.6734;0,0,1.0000];
D1=[-0.3371,0.1315,-6.3185e-06,-3.6323e-04,0];

K2 = [897.4566,0,635.4040;0,896.7992,375.3149;0,0,1];
D2 = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
borW=0.767;
borH=0.626;
img_path1 = "/home/ramlab/Documents/publication/unifiedCali/data/real-world/dual-camera/img1";
img_path2 = "/home/ramlab/Documents/publication/unifiedCali/data/real-world/dual-camera/img2";
img_list1 = dir(img_path1);
img_list2 = dir(img_path2);
img_list1 = img_list1(3:end);
img_list2 = img_list2(3:end);
thetas = [];
pro_error = [];
t_errs= [];
multi_theta_errs=[];

cam_corners3D1={};
cam_bors_coeff1={};
cam_corners3D2={};
cam_bors_coeff2={};
for idx = 1:size(img_list1,1)
    img_file1 = strcat(img_list1(idx).folder,'/',img_list1(idx).name);
    img_raw1 = imread(img_file1);
    img_file2 = strcat(img_list2(idx).folder,'/',img_list2(idx).name);
    img_raw2 = imread(img_file2);
    
    [im_corners1,cam_plane_coeff1,pro_err1] = imgbor_ext(img_raw1,K1,D1,0.07,borW,borH);
    [im_corners2,cam_plane_coeff2,pro_err2] = imgbor_ext(img_raw2,K2,D2,0.07,borW,borH);
    cam_corners3D1{idx} = im_corners1;
    cam_corners3D2{idx} = im_corners2;
    cam_bors_coeff1{idx} = cam_plane_coeff1;
    cam_bors_coeff2{idx} = cam_plane_coeff2;
end
% optimize

TInit = plane_init(cam_bors_coeff2,cam_bors_coeff1,cam_corners3D2,cam_corners3D1);

TOptm = corner_optm(cam_corners3D1,cam_corners3D2,TInit);

figure;
img_un1 = myundistortImage(img_raw1,K1,D1);
imshow(img_un1);
hold on;
cam_corner1_all =[];
cam_corner2_all =[];
for idx=1:size(cam_corners3D1,2)
    cam_corner1_all = [cam_corner1_all,cam_corners3D1{idx}];
    cam_corner2_all = [cam_corner2_all,cam_corners3D2{idx}];
end
cam_corners3D2_aft = TOptm(1:3,1:3)*cam_corner2_all+TOptm(1:3,4);
cam_corners3D2_px = K1*cam_corners3D2_aft;
cam_corners3D2_px =cam_corners3D2_px(1:2,:)./cam_corners3D2_px(3,:);
cam_corners3D1_px = K1*cam_corner1_all;
cam_corners3D1_px =cam_corners3D1_px(1:2,:)./cam_corners3D1_px(3,:);

plot(cam_corners3D1_px(1,:),cam_corners3D1_px(2,:),"ob",'MarkerSize',7);
plot(cam_corners3D2_px(1,:),cam_corners3D2_px(2,:),"*r",'MarkerSize',7);

