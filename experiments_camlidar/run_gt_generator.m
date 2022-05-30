clc; clear; close all;

add_path_lcecalib;

format short

% data_type = 'simu_data_bias';
% data_type = 'simu_data';
data_type = 'real_data';
% data_type = 'fp_data';

visualization_flag = 0;
debug_flag = 0;
save_result_flag = 0;
plot_result_flag = 0;

for data_option = 5:5
  sprintf('data_option: %d', data_option)
  data_path = fullfile('data', data_type, strcat(data_type, '_', num2str(data_option)));
  
  params = load(fullfile(data_path, 'img/params.mat'));
  borW = params.borW; borH = params.borH; 
  numW = params.numW; numH = params.numH;
  pattern_size = params.pattern_size;
  K = params.K; D = params.D;
  TGt = params.TGt;
  num_data = params.num_data;
  use_edge_flag = params.use_edge_flag;
  use_planar_flag = params.use_planar_flag;  
  edge_weight = params.edge_weight;
  planar_weight = params.planar_weight;

  %% Extract features
  img_list = dir(fullfile(data_path, 'img')); 
  img_list = img_list(3:end);
  pcd_list = dir(fullfile(data_path, 'pcd'));
  pcd_list = pcd_list(3:end);  
  pcd_raw_list = dir(fullfile(data_path, 'raw_pcd'));
  pcd_raw_list = pcd_raw_list(3:end);  

  img_corner3D = {};
  pcd_corner3D = {};
  pc_bors_ceoff = {};
  cam_bors_coeff = {};
  for idx = 1:size(pcd_list, 1)
      img_file = strcat(img_list(idx).folder,'/',img_list(idx).name);
      pcd_file = strcat(pcd_list(idx).folder,'/',pcd_list(idx).name);
      img_raw = imread(img_file);
      pc_raw = pcread(pcd_file);
      pc_array = pc_raw.Location()';

      [pts_bor, bor_coeff, err] = boardpts_ext(pc_array,borW,borH, data_type);  %% extract board points
      pc_corners = borcorner_ext(pts_bor,borW,borH);  %% extract board corners
      [im_corners, cam_plane_coeff, pro_err] = imgbor_ext(img_raw,K,D,pattern_size,borW,borH);

      % save data
      pcd_corner3D{idx} = pc_corners;
      pc_bors_ceoff{idx} = bor_coeff;
      img_corner3D{idx} = im_corners;
      cam_bors_coeff{idx} = cam_plane_coeff;
  end
end



