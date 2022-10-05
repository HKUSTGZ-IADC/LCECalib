%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation strudy: perform batch tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

add_path_lcecalib;
add_path_qpep;
format short

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 1 - proposed method
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 1;
flg_use_iter_num = 100;
flg_use_ptp = 1;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 2 - wo point projection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 0;
flg_outlier_rejection = 1;
flg_use_iter_num = 100;
flg_use_ptp = 1;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 3 - wo outlier rejection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 0;
flg_use_iter_num = 100;
flg_use_ptp = 1;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 4 - iteration number=1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 1;
flg_use_iter_num = 1;
flg_use_ptp = 1;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 5 - iteration number=10 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 1;
flg_use_iter_num = 5;
flg_use_ptp = 1;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 6 - wo using point-to-plane
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 1;
flg_use_iter_num = 100;
flg_use_ptp = 0;
flg_use_ptl = 1;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ablation test 7 - wo using point-to-line
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
data_type = 'real_data';
flg_point_projection = 1;
flg_outlier_rejection = 1;
flg_use_iter_num = 100;
flg_use_ptp = 1;
flg_use_ptl = 0;
save('data_tmp/tmp_lcecalib_ablation_study_setup.mat', ...
  'flg_point_projection', 'flg_outlier_rejection', 'flg_use_iter_num', ...
  'flg_use_ptp', 'flg_use_ptl');
run_lcecalib_qpep_ablation_study;

