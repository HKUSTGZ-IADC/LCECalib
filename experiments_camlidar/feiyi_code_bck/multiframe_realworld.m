clc;
clear;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%parameters
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
borW=0.767;
borH=0.626;
pcd_path = "/Monster/dataset/event_camera/camera_lidar_calib/example/pcd";
img_path = "/Monster/dataset/event_camera/camera_lidar_calib/example/img";
img_list = dir(img_path);
pcd_list = dir(pcd_path);
pcd_list = pcd_list(3:end);
img_list = img_list(3:end);


img_corner3D={};
pcd_corner3D={};
pc_bors_ceoff={};
cam_bors_coeff={};
for idx = 1:15
    % 输入数据
    img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
    img_raw = imread(img_file);
    pc_raw = pcread(pcd_file);
    pc_array = pc_raw.Location()';
    
    [pts_bor,bor_coeff,err] = boardpts_ext(pc_array,borW,borH);
    pc_corners = borcorner_ext(pts_bor,borW,borH);
    [im_corners,cam_plane_coeff,pro_err] = imgbor_ext(img_raw,K,D,0.07,borW,borH);
    
    % save data
    pcd_corner3D{idx} = pc_corners;
    pc_bors_ceoff{idx}=bor_coeff;
    img_corner3D{idx} = im_corners;
    cam_bors_coeff{idx} = cam_plane_coeff;
    
end
% optimize

TInit = plane_init(pc_bors_ceoff,cam_bors_coeff,pcd_corner3D,img_corner3D);
TOptm = corner_optm(img_corner3D,pcd_corner3D,TInit);

img_un = myundistortImage(img_raw,K,D);
figure;
img_pro = pt_project_depth2image(TOptm,K,pc_array,img_un);
imshow(img_pro);



