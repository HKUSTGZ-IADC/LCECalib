clc;
clear;
addpath("..");
addpath("../20210125_IRLS_ICP");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%parameters
borW=0.77;
borH=0.63;

K1 =[640.5098521801531, 0.0, 640.5; 0.0, 640.5098521801531, 360.5; 0.0, 0.0, 1.0];
D1 = [0.0, 0.0, 0.0, 0.0, 0.0];
K2 =[640.5098521801531, 0.0, 640.5; 0.0, 640.5098521801531, 360.5; 0.0, 0.0, 1.0];
D2 = [0.0, 0.0, 0.0, 0.0, 0.0];
TGt = [0.94215      0.27068     -0.19768          0.3
    -0.29404      0.95056    -0.099833          0.2
    0.16088      0.15218      0.97517          0.4
    0            0            0            1];
TInit = [];
load("simu_data/noise-0.007-corners.mat");
load("idx.mat");
aver_t_err=[];
aver_theta=[];

for PoseNum=1:32
    disp(PoseNum);
    thetas = [];
    pro_error = [];
    t_errs= [];
    multi_theta_errs=[];
    for iter=1:1000
        %       reidx = randperm(size(img_corner3D1,2));
        reidx = reidxs(iter,:);
        sub_cam_corners3D1={};
        sub_cam_bors_coeff1={};
        sub_cam_corners3D2={};
        sub_cam_bors_coeff2={};
        for idx = 1:PoseNum
            % save data
            sub_cam_corners3D1{idx} = img_corner3D1{reidx(idx)};
            sub_cam_bors_coeff1{idx} = cam_bors_coeff1{reidx(idx)};
            sub_cam_corners3D2{idx} = img_corner3D2{reidx(idx)};
            sub_cam_bors_coeff2{idx} = cam_bors_coeff2{reidx(idx)};
        end
        % optimize
        if PoseNum<4
            TInit =[0.94274      0.27022     -0.19551      0.29659
     -0.29341      0.95064     -0.10091      0.20132
      0.15859       0.1525       0.9755      0.40078
            0            0            0            1];
        else
            TInit = plane_init(sub_cam_bors_coeff2,sub_cam_bors_coeff1,sub_cam_corners3D2,sub_cam_corners3D1);
        end
        
        TOptm = corner_optm(sub_cam_corners3D1,sub_cam_corners3D2,TInit);
        deltaT = inv(TGt)*TOptm;
        deltaQ = rotm2quat(deltaT(1:3,1:3));
        angle_err=abs(2*acosd(deltaQ(1)));
        multi_theta_errs = [multi_theta_errs,angle_err];
        t_errs = [t_errs,norm(deltaT(1:3,4))];
    end
    aver_t_err = [aver_t_err,t_errs'];
    aver_theta = [aver_theta,multi_theta_errs'];
end

figure;
boxplot(aver_t_err);
title("t error");
figure;
boxplot(aver_theta);
title("R error");
