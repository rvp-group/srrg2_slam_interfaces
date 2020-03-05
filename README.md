# SRRG2-SLAM-INTERFACES
This repository contains a single Catkin package:

  1. [`srrg2_slam_interfaces`](srrg2_slam_interfaces): generic interfaces and API designed to standardize multi-cue SLAM.

This package alone is not a complete SLAM system. The following additional `srrg2` packages built on top of `srrg2_slam_interfaces` constitute the complete pipelines:

* [`srrg2_laser_slam_2d`](https://github.com/srrg-sapienza/srrg2_laser_slam_2d): multi-cue 2D-LiDAR pipeline; it can manage multiple laser rangefinders together with wheel odometry.
* [`srrg2_proslam`](https://github.com/srrg-sapienza/srrg2_proslam): VO pipeline (Stereo or RGB-D).

Other additional `srrg2` packages:

* [`srrg2_executor`](https://github.com/srrg-sapienza/srrg2_executor): provides a shell interface to load, modify and run `BOSS` configuration file; this is the main package to **run all `srrg2` pipelines**.
* [`srrg2_config_visualizer`](https://github.com/srrg-sapienza/srrg2_config_visualizer): GUI to load, edit and generate `BOSS` configuration file; still experimental.

To know how to build the `srrg2_slam_interfaces` package, please refer to the [`readme`](srrg2_slam_interfaces/README.md) file in the Catkin package. If you want to build also the additional `srrg2` packages, refer to the instructions in their readme files.

## Publications
To have a more detailed overview on this architecture, you can read our new [preprint](https://arxiv.org/abs/2003.00754).
If you use our code, please cite us in your work.
```bibtex
@misc{colosi2020plugandplay,
    title={Plug-and-Play SLAM: A Unified SLAM Architecture for Modularity and Ease of Use},
    author={Mirco Colosi and Irvin Aloise and Tiziano Guadagnino and Dominik Schlegel and Bartolomeo Della Corte and Kai O. Arras and Giorgio Grisetti},
    year={2020},
    eprint={2003.00754},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

## Contributors
* Mirco Colosi
* Irvin Aloise
* Dominik Schlegel
* Giorgio Grisetti
* Bartolomeo Della Corte
* Tiziano Guadagnino

## License
BSD 3.0
