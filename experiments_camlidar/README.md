LiDAR-Camera Extrinsic Calibration

### Dataset Preparation

1. please prepare data according to below folder structure

   > real_data/real_data_1
   >
   > ​	img/(1, 2, 3, ...).png: calibration images
   >
   > ​	pcd/(1, 2, 3, ...).pcd: calibration board point clouds 
   >
   
2. please prepare camera intrinsics, board hyperparameters of the code

   * calibrate intrinsics of the camera using prepared data: using *CameraCalibrator* in MATLAB
   * modify parameters in *params_to_matlab.m* and run the code with calibrated intrinsics
   * save *params.mat* in the data folder

3. Data used in the TMech paper

   * Download link: http://gofile.me/4jm56/wuo1qjH7Q
   * simu_data/
     * simu_data_1 - simu_data_10: 
   * real_data/
     * real_data_1 - real_data_3: *RLFS01* - *RLFS03*
     * real_data_4 - real_data_9: *RLES01-LF*, *RLES01-LE*, *RLES02-LF*, *RLES02-LE*, *RLES03-LF*, *RLES03-LE*

### How to Calibration

1. **Version 1 of LCECalib**
   1. Run *multiframe_simu_data.m* and *multiframe_real_data.m* to calibrate simulated sensors and real-world sensors respectively (using the proposed method). Results are saved as *result_proposed.mat* in the data folder
   2. Run *baseline_multiframe* to calibrate sensors (using the baseline method [Zhou2018Automatic]). Results are saved as *result_baseline.mat* in the data folder
2. **Version 2 of LCECalib** (utilize QPEP to solve the PnP and Point-To-Plane registration problem)
   1. set *data_type* for *simu_data*, *simu_data_bias*, *real_data*, *fp_data*
   2. Run *run_lcecalib.m* to calibrate sensors

### How to Analyze Results

1. Run *plot_result.m* to 
   * compute and plot calibration error (Fig. 7, Table II)
   * save the calibration error plot
   * plot LiDAR point back-projection results (Fig. 9)
   * print calibrated extrinsics (Table II)
   * compute mean planar error (Table II)
2. Run *plot_pipeline result.m* to plot sensor measurement processing result (Fig. 4)

### Supplementary Materials

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
2. How to extract rectangles from timesurface maps and checkerboard points from point clouds
   * Download the project: ```git clone https://git.ram-lab.com/gogojjh/udi_utils```
   * Build the project: ```catkin build udi_utils_calibration```
   * Run the *roslaunch* file: ```roslaunch udi_utils_calibration calib_hkust_lab.launch calib_status:=false```
   * Run the bag: ```rosbag play xxx.bag --clock -r 1.0```
   * Results are saved to ```workspace/calib_data/calib_data_real/pre_calib```

