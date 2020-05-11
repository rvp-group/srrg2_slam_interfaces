# Package `srrg2_slam_interfaces`

This package contains C++ interfaces and base modules for generic SLAM systems. Those interfaces are designed to allow rapid prototyping of new SLAM solution, maximizing code reuse. In this sense, SLAM building blocks can be combined to address multi-cue SLAM in a modular and scalable way.

###### Hot features:
* minimal external dependencies
* highly efficient due to extensive use of C++ 17 structures
* easy to extend to embed heterogeneous sensors

## How to build
The `srrg2_slam_interfaces` is developed using our `srrg2` framework.
All our software is tested both with Ubuntu 18.04 and 16.04 (GCC 5 and 7), still the remaining of this guide refers to Ubuntu 18.04.
Please follow this guide to build and run `srrg2_slam_interfaces` on your machine:

1. initialize the `srrg2` Catkin workspace following the guide [here](https://github.com/srrg-sapienza/srrg2_core/tree/master/srrg2_core). As indicated in the aforementioned guide, we suggest to have a directory in which you clone all the `srrg2` repositories (referred here as `SRRG2_SOURCE_ROOT`) and a directory that contains the Catkin workspace (referred here as `SRRG2_WS_ROOT`)

2. clone all the `srrg2` dependencies of this package
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_cmake_modules.git # basic cmake-modules
git clone https://gitlab.com/srrg-software/srrg_hbst.git # VPR library (to compute loop closures in Visual-SLAM pipelines)
git clone https://github.com/srrg-sapienza/srrg2_core.git # core data-structures and
git clone https://github.com/srrg-sapienza/srrg2_solver.git # solver (both for registration and global optimization)
```

3. clone this repository
```bash
cd <SRRG2_SOURCE_ROOT>
git clone https://github.com/srrg-sapienza/srrg2_slam_interfaces.git
```

4. link all the required packages in your Catkin workspace
```bash
cd <SRRG2_WS_ROOT>/src
ln -s <SRRG2_SOURCE_ROOT>/srrg2_cmake_modules .
ln -s <SRRG2_SOURCE_ROOT>/srrg_hbst/ .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_core/srrg2_core .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_solver/srrg2_solver .
ln -s <SRRG2_SOURCE_ROOT>/srrg2_slam_interfaces/srrg2_slam_interfaces .
```

5. build using Catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_slam_interfaces
```

6. [OPTIONAL] build unit-tests using catkin
```bash
cd <SRRG2_WS_ROOT>
catkin build srrg2_slam_interfaces --catkin-make-args tests
```

# How to use

This package provides only basic unit-tests as executables. Please refer to one of our pipelines to test it:

* [`srrg2_laser_slam_2d`](https://github.com/srrg-sapienza/srrg2_laser_slam_2d): multi-cue 2D-LiDAR pipeline; it can manage multiple laser rangefinders together with wheel odometry.
* [`srrg2_proslam`](https://github.com/srrg-sapienza/srrg2_proslam): VO pipeline (Stereo or RGB-D) that natively supports sensor offsets.

Additional packages:
* [`srrg2_executor`](https://github.com/srrg-sapienza/srrg2_executor): provides a shell interface to load, modify and run `BOSS` configuration file; this is the main package to **run all `srrg2` pipelines**.
* [`srrg2_config_visualizer`](https://github.com/srrg-sapienza/srrg2_config_visualizer): GUI to load, edit and generate `BOSS` configuration file; still experimental.
