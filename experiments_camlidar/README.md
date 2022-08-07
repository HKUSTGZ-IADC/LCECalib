### LCECalib: Automatic Checkboard-Based LiDAR-Frame Camera-Event Camera Extrinsic Calibration

#### Dataset Preparation
1. Please prepare data according to below folder structure
   * fp_data_20220424/fp_data_20220424_1
     * *img/(1, 2, 3, ...).png*: calibration images
     * *pcd/(1, 2, 3, ...).pcd*: calibration board point clouds 
2. Please prepare camera intrinsics, board hyperparameters of the code
   * Calibrate intrinsics of the camera using prepared data
   * Modify parameters in *fp_data_20220424/params_to_matlab.m*
   * Run *fp_data_20220424/params_to_matlab.m*
   * Parameters *params.mat* will be stored in *img/*
#### Download Example Data
1. Data used in the TMech paper: http://gofile.me/4jm56/wuo1qjH7Q
   * simu_data/
     * simu_data_1 - simu_data_10: 
   * real_data/
     * real_data_1 - real_data_3: *RLFS01* - *RLFS03*
     * real_data_4 - real_data_9: *RLES01-LF*, *RLES01-LE*, *RLES02-LF*, *RLES02-LE*, *RLES03-LF*, *RLES03-LE*
2. Data used in other projects: http://gofile.me/4jm56/hfm6I48kz

#### How to Calibrate
1. Perform calibration for data presented in the **TMech paper**
   * Please set correct ```data_type``` and ```data_option``` in ```run_lcecalib_qpep.m```
   * Please set the flag ```save_result_flag``` and ```plot_result_flag``` in ```run_lcecalib_qpep.m```
   * Run ```run_lcecalib_qpep.m```
2. Perform calibration for other data
   * Please set correct ```data_type``` and ```data_option``` in ```run_lcecalib_qpep_other_data.m```
   * Please set the flag ```save_result_flag``` and ```plot_result_flag``` in ```run_lcecalib_qpep_other_data.m```
   * Run ```run_lcecalib_qpep_other_data.m```

#### Supplementary Materials

1. How to reconstruct frame images from event streams: E2VID
   * Download the project: ```git clone https://git.ram-lab.com/gogojjh/E2VID```
   * Open a docker container
     * build an image: ```cd docker && docker build . -t nvidia/cuda:10.1-py3-conda-torc```
     * build a container: ```nvidia-docker run -it --name e2calib -v /home/jjiao/Docker_ws/docker_fold/documents:/usr/app -p 8001:8888 nvidia/cuda:10.1-py3-conda-torch /bin/bash```
     * open a container: ```docker exec -it e2calib /bin/bash```
     * ```cd e2calib/python```
   * Convert *rosbag* into *h5 file*: ```python convert.py --input_file xxx.bag --output_file xxx.h5 --ros_topic /davis/events```
   * Generate trigger timestamps in the integer type: ```python format_timestamps.py --timestamps_file trigger.txt --timestamps_file_save trigger_format.txt```
   * Reconstruct frame images without trigger: ```python3 offline_reconstruction.py  --h5file xxx.h5 --output_folder path_to_folder --freq_hz 5 --upsample_rate 4 --height 260 --width 346```
   * Reconstruct frame images with trigger: ```python3 offline_reconstruction.py  --h5file xxx.h5 --output_folder path_to_folder --timestamps_file trigger_format.txt --upsample_rate 4 --height 260 --width 346```
<!-- 2. How to extract rectangles from timesurface maps and checkerboard points from point clouds
   * Download the project: ```git clone https://git.ram-lab.com/gogojjh/udi_utils```
   * Build the project: ```catkin build udi_utils_calibration```
   * Run the *roslaunch* file: ```roslaunch udi_utils_calibration calib_hkust_lab.launch calib_status:=false```
   * Run the bag: ```rosbag play xxx.bag --clock -r 1.0```
   * Results are saved to ```workspace/calib_data/calib_data_real/pre_calib``` -->

