clc;
clear;
close all;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%
%%parameters
K = [1614.5, 0, 641.23;
     0, 1614.7, 480.14;
     0,0,1];
D = [-0.0044973, 0.020426, 0, 0, 0];
borW=0.2 * 8;
borH=0.2 * 5;
pcd_path = "/Monster/dataset/event_camera/camera_lidar_calib/matlab_data_64/pointCloud";
img_path = "/Monster/dataset/event_camera/camera_lidar_calib/matlab_data_64/images";
img_list = dir(img_path);
pcd_list = dir(pcd_path);
pcd_list = pcd_list(3:end);
img_list = img_list(3:end);


img_corner3D={};
pcd_corner3D={};
pc_bors_ceoff={};
cam_bors_coeff={};
cnt = 0;
for idx = 1:8
    % 输入数据
    img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
    img_raw = imread(img_file);
    pc_raw = pcread(pcd_file);
    
%     location = pc_raw.Location;
%     % Check if the point cloud is organized
%     if isequal(ndims(location),3)
%         x = reshape(location(:, :, 1), [], 1);
%         y = reshape(location(:, :, 2), [], 1);
%         z = reshape(location(:, :, 3), [], 1);
%         location = [x, y, z];
%     end   
    pc_array = pc_raw.Location()';
    
%     pc_array = reshape(pc_raw.Location, [pc_raw.Count, 3])';
%     pc_array = pc_raw.Location()';
    
    [pts_bor,bor_coeff,err] = boardpts_ext(pc_array,borW,borH);
    if (isempty(pts_bor)) 
      continue;
    end
    pc_corners = borcorner_ext(pts_bor,borW,borH);
    [im_corners,cam_plane_coeff,pro_err] = imgbor_ext(img_raw,K,D,0.081,borW,borH);
    
    % save data
    cnt = cnt + 1;
    pcd_corner3D{cnt} = pc_corners;
    pc_bors_ceoff{cnt} = bor_coeff;
    img_corner3D{cnt} = im_corners;
    cam_bors_coeff{cnt} = cam_plane_coeff;
    
end
% optimize

TInit = plane_init(pc_bors_ceoff,cam_bors_coeff,pcd_corner3D,img_corner3D);
TOptm = corner_optm(img_corner3D,pcd_corner3D,TInit);

img_un = myundistortImage(img_raw,K,D);
figure;
img_pro = pt_project_depth2image(TOptm,K,pc_array,img_un); % tform from camera to LiDAR
imshow(img_pro);



