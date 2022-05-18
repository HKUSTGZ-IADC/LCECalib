function [] = add_path_lcecalib()
  addpath("..");
  addpath("../tools");
  addpath("../20210125_IRLS_ICP");
  addpath("../20210125_IRLS_ICP/kernel");
  addpath("../tools/plane_ransac");
  addpath("../tools/board_extraction");
  
  addpath('evaluation');
  addpath('tools');
  addpath('baseline');
end