clc;
clear;

addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");


K =[640.5098521801531, 0.0, 640.5; 0.0, 640.5098521801531, 360.5; 0.0, 0.0, 1.0];
D = [0.0, 0.0, 0.0, 0.0, 0.0];
borW=0.77;
borH=0.63;
pcd_path1 = "/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-lidar/noise-0.03/pcd1";
pcd_path2 = "/home/ramlab/Documents/publication/unifiedCali/data/simu/dual-lidar/noise-0.03/pcd2";

pcd_list1 = dir(pcd_path1);
pcd_list1 = pcd_list1(3:end);
pcd_list2 = dir(pcd_path2);
pcd_list2 = pcd_list2(3:end);

pcd_corner3D1 = {};
pc_bors_ceoff1 ={};
pcd_corner3D2 = {};
pc_bors_ceoff2 ={};
for idx = 1:size(pcd_list1,1)
    % 输入数据
    pcd_file1 = strcat(pcd_list1(idx).folder,'/',pcd_list1(idx).name);
    pcd_file2 = strcat(pcd_list2(idx).folder,'/',pcd_list2(idx).name);
    pc_raw1 = pcread(pcd_file1);
    pc_raw2 = pcread(pcd_file2);
    pc_array1 = pc_raw1.Location()';
    pc_array2 = pc_raw2.Location()';
    
    [pts_bor1,bor_coeff1,err1] = boardpts_ext(pc_array1,borW,borH);
    pc_corners1 = borcorner_ext(pts_bor1,borW,borH);
    [pts_bor2,bor_coeff2,err2] = boardpts_ext(pc_array2,borW,borH);
    pc_corners2 = borcorner_ext(pts_bor2,borW,borH);
    
    % save data
    pcd_corner3D1{idx} = pc_corners1;
    pc_bors_ceoff1{idx}=bor_coeff1;
    pcd_corner3D2{idx} = pc_corners2;
    pc_bors_ceoff2{idx}=bor_coeff2;
    
end