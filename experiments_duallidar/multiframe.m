clc;
clear;
addpath("..");
addpath("../20210125_IRLS_ICP");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

%%parameters
borW=0.77;
borH=0.63;
TGt = [0.995   7.3252e-18     0.099833     -0.21897
     0.029503      0.95534     -0.29404    -0.042625
    -0.095375      0.29552      0.95056     -0.20059
            0            0            0            1];
TInit = [];
load("simu_data/noise-free-corners.mat");

aver_t_err=[];
aver_theta=[];

for PoseNum=1:32
  disp(PoseNum);
  thetas = [];
  pro_error = [];
  t_errs= [];
  multi_theta_errs=[];
  for iter=1:1000
      reidx = randperm(size(pcd_corner3D1,2));
      sub_lidar_corners3D1={};
      sub_pc_bors_coeff1={};
      sub_lidar_corners3D2={};
      sub_pc_bors_coeff2={};
      for idx = 1:PoseNum
          sub_lidar_corners3D1{idx} = pcd_corner3D1{reidx(idx)};
          sub_pc_bors_coeff1{idx} = pc_bors_ceoff1{reidx(idx)};
          sub_lidar_corners3D2{idx} = pcd_corner3D2{reidx(idx)};
          sub_pc_bors_coeff2{idx} = pc_bors_ceoff2{reidx(idx)};
      end
      % optimize
      if PoseNum<4
        TInit =[0.99661    0.0030972     0.082224     -0.22868
       0.0184       0.9656     -0.25939    -0.044024
    -0.080199      0.26002      0.96227     -0.16446
            0            0            0            1]; 
      else
          TInit = plane_init(sub_pc_bors_coeff2,sub_pc_bors_coeff1,sub_lidar_corners3D2,sub_lidar_corners3D1);
      end
      
      TOptm = corner_optm(sub_lidar_corners3D1,sub_lidar_corners3D2,TInit);
      deltaT = inv(TGt)*TOptm;
      deltaQ = rotm2quat(deltaT(1:3,1:3));
      angle_err=abs(2*acosd(deltaQ(1)));
%       axangle = rotm2axang(deltaT(1:3,1:3));
%       angle_err2 = rad2deg(axangle(4))
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
