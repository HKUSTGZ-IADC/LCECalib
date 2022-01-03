LiDAR-Camera Extrinsic Calibration

### Dataset Preparation

1. please prepare data according to below folder structure

   > real_data/real_data_1
   >
   > ​	img/1.png, img/2.png, ...: calibration images
   >
   > ​	pcd/1.pcd, pcd/2.pcd, ...: calibration board point clouds 
   >
   > ​	raw_pcd/1.pcd, raw_pcd/2.pcd, ...: calibration raw point clouds 

2. please prepare camera intrinsics, board hyperparameters of the code

   * calibrate intrinsics of the camera using prepared data
   * modify parameters in *parameters_to_matlab.m* and run the code
   * save *params.mat* and *calibration.mat* in the data folder

3. Data used in the TMech paper

   * simu_data/{noise-free, noise-0.015, noise-0.03}: data subjected to std={0, 0.015, 0.03} Gaussian noise

   * real_data/
     * real_data_1 - real_data_3: *RLFS01* - *RLFS03*
     * real_data_4 - real_data_9: *RLES01-LF*, *RLES01-LE*, *RLES02-LF*, *RLES02-LE*, *RLES03-LF*, *RLES03-LE*

### How to Calibration

1. Run *multiframe_simu_data.m* and *multiframe_real_data.m* to calibrate simulated sensors and real-world sensors respectively (using the proposed method). Results are saved as *result_proposed.mat* in the data folder
2. Run *baseline_multiframe* to calibrate sensors (using the baseline method [Zhou2018Automatic]). Results are saved as *result_baseline.mat* in the data folder

### How to Analyze Results

1. Run *plot_result.m* to 
   * compute and plot calibration error (Fig. 7, Table II)
   * save the calibration error plot
   * plot LiDAR point back-projection results (Fig. 9)
   * print calibrated extrinsics (Table II)
   * compute mean planar error (Table II)
2. Run *plot_pipeline result.m* to plot sensor measurement processing result (Fig. 4)

