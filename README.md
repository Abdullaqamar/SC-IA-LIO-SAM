# SC-IA-LIO-SAM

This repository provides a modified version of IA-LIO-SAM with the following fixes:

- Resolved Ouster LiDAR "not dense points" issue
- Fixed OpenCV compatibility issues on Ubuntu Noetic

## How to Run

```bash
source devel/setup.bash && roslaunch lio_sam run.launch
```

In a separate terminal:

```bash
rosbag play kiss-icp.bag
```

---

## Acknowledgement

This work builds upon prior contributions:

- [IA-LIO-SAM](https://github.com/minwoo0611/IA_LIO_SAM)
- LIO-SAM
  - by T. Shan, B. Englot, D. Meyers, W. Wang, C. Ratti, and D. Rus
  - *LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping*

I extend my gratitude to the original authors for their research and open-source contributions.
