# Supervising iCub calibration 

## :information_source: Intro

This repository contains the software required to supervise the calibration of the iCub robot, specifically:
- [camera calibration](cameraSupervision/README.md);
- [reaching / grasping offsets calibration](demoRedBallCalibration/README.md) for the red ball demo.

## :arrow_forward: How to install

### Manual installation

If you want to install the repository manually, you can refer to the following instructions.

#### Dependencies

- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [YCM](https://github.com/robotology/ycm)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [OpenCV](https://github.com/opencv/opencv)

#### Compilation

On `Linux` system, the code can be compiled as follows:

``` 
git clone https://github.com/robotology/calibration-supervisor.git
cd calibration-supervisor
mkdir build && cd build
ccmake ..
make install 
```

Please note that the variables `SUPERVISE_CAMERA` and `SUPERVISE_REDBALL` refer to *camera calibration supervision* and *demo red ball calibration* respectively, which are *enabled* by default. If you want to disable one, please set the related variable to `OFF`.
