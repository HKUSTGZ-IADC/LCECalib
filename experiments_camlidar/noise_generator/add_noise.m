clc;
clear;
clean_data_path = "/home/cfy/Documents/unifiedCali/data/simu/cam-lidar2/noise-free";
pose_path = "/home/cfy/Documents/unifiedCali/data/simu/cam-lidar2/pose.txt"; % the pose of the checkerboard
save_path = "/home/cfy/Documents/unifiedCali/data/simu/cam-lidar2/noise-0.1";
bor_w = 0.77; 
bor_h = 0.63;
diver_angle = 0.003;%rad
noise_level = 0.1; %unit meter
pcd_path = clean_data_path+"/pcd/";
pcd_save_path = save_path+"/pcd/";
pcd_dir = dir(pcd_path);
pcd_dir = pcd_dir(3:end);
bor_corners = [bor_h/2,bor_w/2,0;...
               -bor_h/2,bor_w/2,0;...
               -bor_h/2,-bor_w/2,0;...
               bor_h/2,-bor_w/2,0;...
               bor_h/2,bor_w/2,0]';
for idx=1:size(pcd_dir,1)
    pc_raw=pcread(pcd_path+pcd_dir(idx).name);
    pc_array = pc_raw.Location()';
    poses = load(pose_path);
    R = eul2rotm(poses(idx,[6,5,4]));
    t = poses(idx,[1,2,3])'-[0,0,1.2]';
    [pts_edge,pts_edge2]=f_bor_edge_ext2(pc_array,bor_w,bor_h);
    pts_out = f_bor_edge_add_noise(R,t,pts_edge2,diver_angle,bor_w,bor_h);
    bor_corners_aft = R*(bor_corners)+t;
%     figure;
%     plot3(pts_edge(1,:),pts_edge(2,:),pts_edge(3,:),'.b');
%     hold on;
%     plot3(pts_edge2(1,:),pts_edge2(2,:),pts_edge2(3,:),'.b');
%     plot3(pts_out(1,:),pts_out(2,:),pts_out(3,:),'.g');
%     plot3(bor_corners_aft(1,:),bor_corners_aft(2,:),bor_corners_aft(3,:),'-*r');
%     axis equal;

    ns = createns(pc_array');
    [inlier, dist] = knnsearch(ns,[pts_edge,pts_edge2]','k',1);
    inlier_inv = setdiff([1:size(pc_array,2)],inlier);
    pc_all = [[pts_edge,pts_out],pc_array(:,inlier_inv)];
    pc_all = f_add_noise_dis(pc_all,noise_level);
    pc_save = pointCloud(pc_all');
    pcwrite(pc_save,pcd_save_path+pcd_dir(idx).name);
%     figure;
%     plot3(pc_all(1,:),pc_all(2,:),pc_all(3,:),'.r');
%     axis equal;
end


