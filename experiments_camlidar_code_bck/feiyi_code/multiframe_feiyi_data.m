clc;
clear;
close all;
addpath("..");
addpath("../tools");
addpath("../20210125_IRLS_ICP");
addpath("../20210125_IRLS_ICP/kernel");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%% parameters
params = load('real_data/real_data_1/params.mat');
borW = params.borW;
borH = params.borH;
pattern_size = params.pattern_size;
K = params.K;
D = params.D;
TGt = params.TGt;
num_data = 15;
is_display = 0;
all_iterations = 100;

%%
pcd_path = "real_data/real_data_1/pcd";
img_path = "real_data/real_data_1/img";
img_list = dir(img_path);
pcd_list = dir(pcd_path);
pcd_list = pcd_list(3:end);
img_list = img_list(3:end);

img_corner3D = {};
pcd_corner3D = {};
pc_bors_ceoff = {};
cam_bors_coeff = {};

aver_t_err = [];
aver_r_err = [];
for idx = 1:num_data
    img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
    pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
    img_raw = imread(img_file);
    pc_raw = pcread(pcd_file);
    pc_array = pc_raw.Location()';
    
    [pts_bor,bor_coeff,err] = boardpts_ext(pc_array,borW,borH);
    pc_corners = borcorner_ext(pts_bor,borW,borH,is_display);
    [im_corners,cam_plane_coeff,pro_err] = imgbor_ext(img_raw,K,D,pattern_size,borW,borH,is_display);
    
    % save data
    pcd_corner3D{idx} = pc_corners;
    pc_bors_ceoff{idx} = bor_coeff;
    img_corner3D{idx} = im_corners;
    cam_bors_coeff{idx} = cam_plane_coeff;

    for iter = 1:all_iterations
      t_errs= [];
      multi_theta_errs=[];

      TInit = plane_init(pc_bors_ceoff,cam_bors_coeff,pcd_corner3D,img_corner3D);
      TOptm = corner_optm(img_corner3D,pcd_corner3D,TInit);
      
      deltaT = inv(TGt) * TOptm;
      deltaQ = rotm2quat(deltaT(1:3,1:3));
      angle_err = abs(2*acosd(deltaQ(1)));
      multi_theta_errs = [multi_theta_errs, angle_err];
      t_errs = [t_errs, norm(deltaT(1:3,4))];
%       imshow(pt_project_depth2image(TOptm,K,pc_array,myundistortImage(img_raw,K,D)));
    end
    aver_t_err = [aver_t_err, t_errs'];
    aver_r_err = [aver_r_err, multi_theta_errs'];
    sprintf('idx: %d', idx)
end
% save('real_data/real_data_1/result_proposed.mat', 'aver_r_err', 'aver_t_err', 'TOptm', 'pcd_corner3D', 'pc_bors_ceoff', 'img_corner3D', 'cam_bors_coeff');

%% 
figure; boxplot(aver_r_err);
xlabel("Number of Poses"); title("Rotation Error [deg]");
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.3;
set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
title('Mean and Median Rotation Error', 'FontSize', 30, 'FontWeight', 'normal');
box on;

figure; boxplot(aver_t_err);
xlabel("Number of Poses"); ylabel("Translation Error [m]");
grid on;
ax = gca;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.3;
set(gca, 'FontName', 'Times', 'FontSize', 25, 'LineWidth', 1.5);
title('Mean and Median Translation Error', 'FontSize', 30, 'FontWeight', 'normal');
box on;




