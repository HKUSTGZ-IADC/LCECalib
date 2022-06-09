clc; clear;
close all;
addpath("..");
addpath("../20210125_IRLS_ICP");
addpath("../tools/plane_ransac");
addpath("../tools/board_extraction");

data_fold = {'noise-free', 'noise-0.015', 'noise-0.03'};

for data_option = 1:3
  sprintf('data_option: %d', data_option)
  %% parameters
  params = load('simu_data/params.mat');
  borW = params.borW;
  borH = params.borH;
  pattern_size = params.pattern_size;
  K = params.K;
  D = params.D;
  TGt = params.TGt;
  TInit = [];
  all_iterations = 100;

  load(fullfile('simu_data', data_fold{data_option}, 'corners.mat'));
 
  %% Calibration 
  aver_t_err = [];
  aver_r_err = [];
  for PoseNum = 1:length(cam_bors_coeff) - 1
    sprintf('PoseNum: %d', PoseNum)
    thetas = [];
    pro_error = [];
    t_errs= [];
    multi_theta_errs=[];
    for iter=1:all_iterations
        reidx = randperm(size(img_corner3D,2));
        sub_cam_corners3D={};
        sub_lidar_corners3D={};
        sub_pc_bors_coeff={};
        sub_cam_bors_coeff={};
        for idx = 1:PoseNum
            sub_cam_corners3D{idx} = img_corner3D{reidx(idx)};
            sub_lidar_corners3D{idx} = pcd_corner3D{reidx(idx)};
            sub_pc_bors_coeff{idx} = pc_bors_ceoff{reidx(idx)};
            sub_cam_bors_coeff{idx} = cam_bors_coeff{reidx(idx)};
        end
        
        % optimize
        if PoseNum < 4
          TInit =[-0.28311     -0.93952     -0.19272      0.44626
                   0.16489      0.15027      -0.9748     -0.32762
                   0.94481     -0.30776      0.11238     -0.12789
                         0            0            0            1]; 
        else
            TInit = plane_init(sub_pc_bors_coeff,sub_cam_bors_coeff,sub_lidar_corners3D,sub_cam_corners3D);
        end
        
        TOptm = corner_optm(sub_cam_corners3D,sub_lidar_corners3D,TInit);
        deltaT = inv(TGt)*TOptm;
        deltaQ = rotm2quat(deltaT(1:3,1:3));
        angle_err=abs(2*acosd(deltaQ(1)));
        multi_theta_errs = [multi_theta_errs,angle_err];
        t_errs = [t_errs,norm(deltaT(1:3,4))];
    end
    aver_t_err = [aver_t_err,t_errs'];
    aver_r_err = [aver_r_err,multi_theta_errs'];
  end
  save(fullfile('simu_data', data_fold{data_option}, 'result_proposed.mat'), 'aver_r_err', 'aver_t_err', 'TOptm');
  
  %% plot results
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
end

