# Efficient Doppler LiDAR Odometry using Scan Slicing and Vehicle Kinematics

## Introduction: The ROS node estimates the 3D odometry of a vehicle using FMCW LiDAR. In the IEKF framework, we estimate the vehicle’s pose using not only 3D point cloud but also Doppler velocity.

### Developer：[Chenglin Pang](https://github.com/pangchenglin)

### Related Paper:

coming soon.

## [Prerequisites]

### [ **PCL && Eigen**]

[PCL>= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).]

[Eigen>= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).]

## [Build]

[Clone the repository and catkin_make:]

```cd
    git clone https://github.com/pangchenglin/4DLO.git
    cd ..
    catkin_make
    source devel/setup.bash
```

## **Run on Dataset**

If you want to test our codes, you can download [AEVA dataset](https://drive.google.com/file/d/1JpQNnXejow3qy1qp5tVzak9qnuFmjYHW/view) which is collected by steam_icp. But this dataset need be transformed as rosbag. You also can download the [dataset](链接: https://pan.baidu.com/s/1EgG6L0BWhe0yb_t5uzWA8g?pwd=quru 提取码: quru--来自百度网盘超级会员v1的分享) our transformed.

## Acknowledgments

Thanks for [VoxelMap](https://github.com/hku-mars/VoxelMap) (An efficient and probabilistic adaptive voxel mapping method for LiDAR odometry) and [steam_icp](https://github.com/utiasASRL/steam_icp) (Picking Up Speed: Continuous-Time Lidar-Only Odometry using Doppler Velocity Measurements）

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.For any technical issues, please contact us via email [pclin93@gmail.com](mailto:pclin93@gmail.com).
