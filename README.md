# Potree_web_portal

In this study, Potree (http://www.potree.org) is adopted to create a prototype web portal. However, it lacks a built-in functionality for forward/backward projection between imagery and LiDAR data, as well as tools for displaying intensity profiles and lane width. These functions were developed within the Potree-based web portal. The forward/backward projection functions and intensity profile/lane width displaying tools are discussed in the following paragraphs.

### System requirements ###

**[[MATLAB](https://www.mathworks.com/l)]** 

**[[CloudCompare or I-Live](https://www.danielgm.net/cc/)]** 

Learning Part:

**[[OS with GPU or Google Colab](https://colab.research.google.com/)]** 

### Steps after Data Collection ###

#### PWMMS-HA (Transit):

Once the trajectory has been processed at the provided address:

```bash
DPRG Server:\Common\Data\platforms\PWMMS\PWMMS-HA\Year\Date_Month\gnssins\processed
```
This trajectory will serve as an input for the first step.

1. LiDAR Reconstruction:

* The following Folders should be created at the given address:

```bash
Raw (Including raw .pcap file from each LiDAR sensor)
Reconstruction
tile
DPRG Server:\Common\Data\platforms\PWMMS\PWMMS-HA\Year\Date_Month\lidar\
```
In addition, these folders should include four main subfolders with the following names and The names of these subfolders are based on the mounted LiDAR sensors on PWMMS-HA (Transit):

```bash
HDL32EF
HDL32EL
HDL32ER
VLP16
```
 * This executable code should be utilized under the following name to reconstruct LiDAR point clouds:
```bash
 PCAPModule_fixGprmc_03032021.exe
```
The input for this code includes the following folders and files:

```bash
bop (Folder including all Trajectory files)
PCAP (Folder including all pcap raw files)
bop_files.txt (File including address to bop files)
pcap_files.txt (File including address to pcap files)
calib_files.txt (File including the name of all calibration files with scalib format)
config (File including PCAP Module Configuration File. One of the configurable parameters within this file is the range of reconstruction.)
```

The output for this code includes the following folders and files:
```bash
```

#### Lane Marking Extraction (Transit/F150):

1. prepare a project folder for each MMS/run (Geometric_template_folder)
* Cropped trajectory: x, y, z, time (please delete the turning or alignment area)
```bash
sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\export_Mission1_TC_WGS84_cut.txt"
```
* Las_list: directories for all bare earth point clouds
```bash
sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\Las_list.txt"
```
* Intensity normalization maps: lookup tables from intensity normalization
   
2. run Main_part1.m
* There are two different programs for MMS (Main_Transit_inner.m/Main_2023_F150_inner.m)

```bash
Parameters in Main.m

%tiling
tiling_original = 0; % if you want to extract lane marking using original intensity = 1
downsample_rate = 10; % the downsampling rate for reading trajectory (decided based on driving speed: 50 mph)
len_width = 12.8; % the length (m) of road surface block along driving direction (decided based on the minimum radius of curvature required for designing highways)
width_left = 9; % the left width (m) of road surface block across driving direction (decided based on the average local point spacing of a bare earth point cloud)
width_right = 9; % the right width (m) of road surface block across driving direction (decided based on the average local point spacing of a bare earth point cloud)

sensor_start_id_is_zero = true; % if the starting number of sensor ID = 0 in bare earth point clouds
scanline_start_id_is_zero = true; % if the starting number of laser beam ID = 0 in bare earth point clouds
minptthreshold = 380000; % the min point threshold for extraction for avoiding too many points captured when the MMS stops

%thresholding
int_threshold = 80; % the intensity percentile threshold for extraction (decided based on the status of lane markings; you can adjust from 98-75)

%de-noise
time_gap = 0.8;  % time interval threshold for determine a scan line (decided based on driving speed: 50 mph)
scanline_cluster_dist = 0.08; % min distance threshold (m) for determine a scan line (decided based lane marking standard width: 15 cm)
scanline_cluster_dist2 = 0.50; % max distance threshold (m) for determine a scan line (decided based lane marking standard width: 15 cm)

%DBSCAN
search_radius = 0.15; % search radius threshold for DBSCAN (decided based on the average local point spacing  of a bare earth point cloud; you can adjust from 0.15-0.25)
minpt_threshold = 5; % min point threshold for DBSCAN (decided based on the average local point spacing  of a bare earth point cloud; you can adjust from 5-15)

%area-based
norm_dist_threshold = 0.15; % normal distance threshold (m) for line fitting of lane marking segments (decided based lane marking standard width: 15 cm)
area_ratio_threshold = 0.7; % ration threshold for determining a straight lane marking segment (decided based on the average local point spacing of a bare earth point cloud)

%combination-based
dist_threshold = 0.40; % distance threshold (m) for combining two straight lane marking segments (decided based on the average local point spacing of a bare earth point cloud)

%completition
segment_dist = 3.2; % distance threshold (m) for refine a straight lane marking segment (decided based on the average local point spacing of a bare earth point cloud)
minpt_ref_threshold = 10; % min point threshold for clustering lane marking segments (decided based on the average local point spacing of a bare earth point cloud)
buffer_threshold  = 0.12; % buffer threshold (m) for refine a straight line segment (decided based lane marking standard width: 15 cm)

%reference-direction
minpt_ref_threshold_2 = 5; % min point threshold for reference segment (decided based on the average local point spacing of a bare earth point cloud)
minpt_threshold_2 = 5; % min point threshold for clustering (decided based on the average local point spacing of a bare earth point cloud)

% remove FP
maxDistance = 0.015; % vertical distance threshold (m) for removing false positive (decided based on the average vertical noise level of a bare earth point cloud)
ratio_buffer_thres = 0.25; % buffer threshold (m) for removing false positive (decided based lane marking standard width: 15 cm)
RMSE_thres = 0.055; % vertical RMSE threshold (m) for removing false positive (decided based on the average vertical noise level of a bare earth point cloud)

% tracking
% LM_collinear
norm_dist_threshold2 = 0.08; % distance threshold (m) for combining two straight lane marking segments (decided based on the average local point spacing of a bare earth point cloud)
para_ang_threshold = 2; % angle threshold (degree) for combining two straight line segments  (decided based on the minimum radius of curvature required for designing highways)

% LM_checkingconnect
min_to_max = 1; % driving direction (based on x or y coordinates) for the two first blocks (E/WB: x ; S/NB: y)

% classification
solid_threshold = 3; % min distance threshold (m) for classify solid lines (decided based lane marking standard length for dash lines: 3 m)
cluster_col = 8; % num of column in output centerline point for cluster ID
solid_id = 1; % ID of solid lines in output centerline point 
dash_id = 2; % ID of dash lines in output centerline point
dotted_id = 3; % ID of dotted lines in output centerline point
ambiguous = 4; % ID of ambiguous lane marking in output centerline point

% centerline point
%intensity image
intensity_col = 4; % num of column in output centerline point for intensity
cluster_id = 8; % num of column in output centerline point for cluster ID
classification_id = 9; % num of column in output centerline point for classification ID

%Moving window setting
int_block = 1.5; % length threshold (m) of half of a moving window for generating centerline point (decided based lane marking standard length for dash lines: 3 m)
spacing = 0.2; % interval threshold (m) for generating centerline point

%output
lane_marking_point_id = 4; % centerline ID for extracted lane marking centerline point
inter_lane_marking_point_id = 1; % centerline ID for interpolated lane marking centerline point
z_check = 0.2; % height threshold (m) for showing height difference message of extracted/interpolated lane marking centerline point
point_pair_start_id = 1; % starting ID for extracted/interpolated lane marking centerline point   
```
   
4. prepare "main_id.txt"
* Import the centerline in classification subfolder into CloudCompare, and find the IDs for a line (which are the most complete lane markings) along the trajectory
```bash
centerline sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\classification\Transit_20231002_Run3__smPT_class_all.txt"
main_id.txt sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\main_id.txt"
```

5. run Clustering_refine_main.m
```bash
Parameters in Clustering_refine_main.m
   
len = 112640; % the length (m) of trajectory
num_lane = 3; % max number of lane for the original centerlines
```
* After running Clustering_refine_main.m, check the results in CloudCompare again and make sure you have a line along the trajectory
```bash
result sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\classification\Transit_20231002_Run3_main2.txt"
```

6. run Clustering_refine_outer_kmean.m
```bash
Parameters in Clustering_refine_outer_kmean.m
   
len = 112640; % the length (m) of trajectory
num_lane = 3; % max number of lanes for the original centerlines
   
Note: you can adjust line23 in Clustering_refine_outer_kmean.m into "num_piece = floor(len/(1.5));" for a small radius of curvature for turning parts in centerlines
      you can adjust line60 in LM_clustering_both.m into "num_piece = floor(len/(12));" for a small radius of curvature for turning parts in centerlines
```
* After running Clustering_refine_outer_kmean.m, you can combine all resultant centerline txt files into one txt file (please ignore the file with the suffix "other")
```bash
result sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\classification\Transit_20231002_Run3_refine_all.txt"
```
   
7. run Main_part2.m
* There are two different programs for MMS (Main_Transit_inner_part2.m/Main_2023_F150_inner_part2.m)
* Use the same inputs for part1
* Use the centerline from Clustering_refine_outer_kmean.m
* Len_list: Centerline ID, length of centerline
```bash
sample: "DPRG4:\Common\Data\PWMMS\PWMMS-HA\20231002_US52_I70_I465\20231002_Run3\sample\20231002_Run3\Len_list.txt"
```
```bash
Parameter Main_part2.m

lane_width_list = [1, 1, 4; 2, 1, 5; 3, 5, 6; 4, 6, 3; 5, 3, 7; 6, 7, 2]; % [lane#, left centerline ID,  right centerline ID; lane#, left centerline ID,  right centerline ID, lane#, left centerline ID,  right centerline ID; lane#, left centerline ID,  right centerline ID];

% intensity thresholding
len_width = 12.8; % the length (m) of road surface block along driving direction (decided based on the minimum radius of curvature required for designing highways)
downsample_rate = 10; % the downsampling rate for reading trajectory (decided based on driving speed: 50 mph)
int_thre = 80; % the intensity percentile threshold for extraction (decided based on the status of lane markings; you can adjust from 98-75)
max_point = 3800000; %the min point threshold for extraction to avoid too many points captured when the MMS stops

% milepost
Start_point_is_first_point = false; % if no mile marker information = true
MMPisIncreasing = true; % if driving direction aligns with mile markers = true
Start_MMP = 25; % starting mile marker for the driving direction
Start_point = [563056.34183700000,4420051.60952000000,247.637001]; % coordinates of starting mile marker for the driving direction

% LM_checkingconnect
min_to_max = 1; % driving direction (based on x or y coordinates) for the two first blocks (E/WB: x ; S/NB: y)

% centerline point
%intensity image
intensity_col = 4; % num of the column in output centerline point for intensity
cluster_id = 8; % num of the column in output centerline point for cluster ID
classification_id = 9; % num of the column in output centerline point for classification ID

%output
lane_marking_point_id = 4; % centerline ID for extracted lane marking centerline point

%Moving window setting
int_block = 1.5; % length threshold (m) of half of a moving window for generating centerline point (decided based lane marking standard length for dash lines: 3 m)
spacing = 0.2; % interval threshold (m) for generating centerline point

%intensity profile generation
interval = 0.25; % interval threshold (m) for intensity profile generation
%output LAS
z_check = 0.2; % height threshold (m) for showing height difference message of extracted/interpolated lane marking centerline point
point_pair_start_id = 1; % starting ID for extracted/interpolated lane marking centerline point

% interpolation
NoLaneMarkingGap = 40; %distance threshold (m) for avoiding linear interpolation
inter_lane_marking_point_id = 1; % centerline ID for interpolated lane marking centerline point

% lane width estimation
narrowlane = 10; % distance threshold (ft) for narrow lane width
widelane = 12; % % distance threshold (ft) for wide lane width
narrowlane_id = 1; % centerline ID for narrow lane width area
widelane_id = 2; % centerline ID for wide lane width area
```















## Related publications ##
Please cite related papers if you use this code:

```
@article{cheng2022generalized,
  title={Generalized LiDAR Intensity Normalization and Its Positive Impact on Geometric and Learning-Based Lane Marking Detection},
  author={Cheng, Yi-Ting and Lin, Yi-Chun and Habib, Ayman},
  journal={Remote Sensing},
  volume={14},
  number={17},
  pages={4393},
  year={2022},
  publisher={MDPI}
}
```
```
@article{cheng2020intensity,
  title={Intensity thresholding and deep learning based lane marking extraction and lane width estimation from mobile light detection and ranging (LiDAR) point clouds},
  author={Cheng, Yi-Ting and Patel, Ankit and Wen, Chenglu and Bullock, Darcy and Habib, Ayman},
  journal={Remote Sensing},
  volume={12},
  number={9},
  pages={1379},
  year={2020},
  publisher={MDPI}
}
```
```
@article{patel2021transfer,
  title={Transfer learning for LiDAR-based lane marking detection and intensity profile generation},
  author={Patel, Ankit and Cheng, Yi-Ting and Ravi, Radhika and Lin, Yi-Chun and Bullock, Darcy and Habib, Ayman},
  journal={Geomatics},
  volume={1},
  number={2},
  pages={287--309},
  year={2021},
  publisher={MDPI}
}
```
```
