function [] = add_path_qpep()
  warning('off');
  format long g

  libQPEP_dir = '../../LibQPEP/MATLAB';
  addpath(fullfile(libQPEP_dir, 'func_files'));
  addpath(fullfile(libQPEP_dir, 'solvers'));
  addpath(fullfile(libQPEP_dir, 'utils'));
  addpath(fullfile(libQPEP_dir, 'calib'));
  addpath(fullfile(libQPEP_dir, 'sdpt3'));
  run(fullfile(libQPEP_dir, 'sdpt3', 'install_sdpt3.m'))
  addpath(fullfile(libQPEP_dir, 'sedumi'));
  if(~verLessThan('matlab', '8.0.0'))  
    run(fullfile(libQPEP_dir, 'sedumi', 'install_sedumi.m'))
  end
  p = genpath(fullfile(libQPEP_dir, 'YALMIP'));
  addpath(p);
  addpath('qpep_api');
end