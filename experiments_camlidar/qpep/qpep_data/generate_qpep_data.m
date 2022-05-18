clc, clear;
close all;

%%
lce_data = load('noise-0.03.mat');
% N = length(lce_data.camsPts);
N = 20;

% ref_pts = zeros(3, M);
% ref_normal = zeros(4, M);
% target_pts = zeros(3, M);
% target_normal = zeros(4, M);

cnt = 1;
for i = 1:N
  for j = 1:length(lce_data.pc_bors_pts{i})
    cam_pts = lce_data.camsPts{i};
    ref_pts(:, cnt) = double(cam_pts(:, 1)');

    cam_bor_coeff = lce_data.cam_bors_coeff{i};
    ref_normal(:, cnt) = double(cam_bor_coeff(:, 1));
    
    pc_bor_pts = lce_data.pc_bors_pts{i};
    target_pts(:, cnt) = double(pc_bor_pts(:, j)');

    pc_bor_coeff = lce_data.pc_bors_ceoff{i};
    target_normal(:, cnt) = double(pc_bor_coeff(:, 1));
    
    cnt = cnt + 1;
  end
end

% GT extrinsics from camera to LiDAR
Rgt = lce_data.TGt(1:3, 1:3);
tgt = lce_data.TGt(1:3, 4);

% compute point-to-plane error
rid = int32(rand(1) * size(ref_normal, 2));
dis = abs(ref_normal(1:3, rid)' * (Rgt * target_pts(:, rid) + tgt) ...
  + ref_normal(4, rid)')

%% 
save('qpep-data-pTop.mat', 'ref_pts', 'ref_normal', ...
  'target_pts', 'target_normal', 'Rgt', 'tgt');