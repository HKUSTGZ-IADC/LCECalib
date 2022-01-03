clc;
clear;
addpath("..");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools");
addpath("../tools/board_extraction");

%%parameters
borW=0.77;
borH=0.63;
pcd_path1 = "/home/ramlab/Documents/publication/unifiedCali/data/real-world/dual-lidar/pcd1";
pcd_path2 = "/home/ramlab/Documents/publication/unifiedCali/data/real-world/dual-lidar/pcd2";

pcd_list1 = dir(pcd_path1);
pcd_list1 = pcd_list1(3:end);
pcd_list2 = dir(pcd_path2);
pcd_list2 = pcd_list2(3:end);

pcd_corner3D1 = {};
pc_bors_ceoff1 ={};
pcd_corner3D2 = {};
pc_bors_ceoff2 ={};
for idx = 1:12
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
TInit = plane_init(pc_bors_ceoff2,pc_bors_ceoff1,pcd_corner3D2,pcd_corner3D1);

TOptm = corner_optm(pcd_corner3D1,pcd_corner3D2,TInit);

figure;
axis equal;
plot3(pc_array1(1,:),pc_array1(2,:),pc_array1(3,:),'.r');
pc_array2_aft = TOptm(1:3,1:3)*pc_array2+ TOptm(1:3,4);
hold on;
plot3(pc_array2_aft(1,:),pc_array2_aft(2,:),pc_array2_aft(3,:),'.b');

pc_all = [pc_array1,pc_array2_aft];
pt = pointCloud(pc_all');
pt.Intensity = [ones(1,size(pc_array1,2)),255*ones(1,size(pc_array2_aft,2))]';
pcwrite(pt,'test.pcd');

