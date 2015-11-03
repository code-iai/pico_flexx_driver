# PMD CamBoard pico flexx Driver

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Table of contents
- [Description](#description)
- [Dependencies](#dependencies)
- [Install](#install)
- [Usage](#usage)
- [Topics](#topics)

## Description

This package is a ROS interface to the [CamBoard pico flexx](http://www.pmdtec.com/picoflexx/) from pmd.
The included node publishes the point cloud, camera info, depth, ir and noise images on ROS topics.
It supports dynamic reconfigure and nodelets with zero copy transfers.
A launch file with static TF publisher, nodelet manager and machine tag support is included.

## Dependencies

- ROS Indigo (or newer should also work)
- OpenCV (2.4.x, using the one from the official Ubuntu repositories is recommended)
- [libroyale](http://www.pmdtec.com/picoflexx/) (1.0.5.40 or newer)

## Install

1. Install the ROS. [Instructions for Ubuntu 14.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Download the Royale SDK from http://www.pmdtec.com/picoflexx/ and extract it
4. Extract the linux 64 bit archive from the extracted SDK.
5. Install the headers and library from the SDK using the provided [script](install_libroyale.sh)
   ```
cd <catkin_ws>/src/pico_flexx_driver
sudo ./install_libroyale.sh <path to extracted linux 64 bit archive>
```
6. Run `catkin_make`
7. Plug in the CamBoard pico flexx device
8. Run `roslaunch pico_flexx_driver pico_flexx_driver.launch publish_tf:=true`
9. Start `rosrun rviz rviz`, set the `Fixed frame` to `pico_flexx_link` and add a `PointCloud2` and select `/pico_flexx/points`

## Usage

To start the pico flexx driver, please use the provided launch file: `roslaunch pico_flexx_driver pico_flexx_driver.launch`

### Parameters

The launch file has the following paramters:

- `base_name` (default="pico_flexx"):
  Name of the node. All topics will be advertised under this name.
- `sensor` (default=""):
  ID of the sensor that should be used. IDs of all connected devices are listed on startup.
- `operation_mode` (default="0"):
  ID of the operation mode A list of supported modes is listed on startup.
- `automatic_exposure` (default="true"):
  Enable or disable automatic expusure.
- `exposure_time` (default="1000"):
  Exposure time. Only for manual exposure.
- `max_noise` (default="0.07"):
  Maximum allowed noise. Data with higher noise will be filtered out.
- `use_png` (default="false"):
  Use PNG instead of TIFF compression for 16 Bit images (depth, mono16 and noise).
- `jpeg_quality` (default="90"):
  JPEG quality setting for 8 Bit image compression (mono8).
- `png_level` (default="1"):
  PNG compression level. Only used if TIFF is not used.
- `queue_size` (default="5"):
  Queue size for publisher.
- `publish_tf` (default="false"):
  Publish a static TF transform for the optical frame of the camera.
- `base_name_tf` (default="pico_flexx"):
  Base name of the tf frames.
- `machine` (default="localhost"):
  Machine on with the nodes should run.
- `define_machine` (default="true"):
  Whether the machine for localhost should be defined our not. Disable this if the launch file is included somewhere where machines are already defined.
- `nodelet_manager` (default="pico_flexx"):
  Name of the nodelet manager.
- `start_manager` (default="true"/>
  Whether to start a nodelet manager our not. Disable this if a different nodelet manager should be used.

## Dynamic reconfigure

Some parameters can be reconfigured during runtime, for example with `rosrun rqt_reconfigure rqt_reconfigure`. The reconfigureable parameters are:
- `operation_mode`:
  Choose from a list of operation modes.
- `exposure_mode`:
  `AUTOMATIC` or `MANUAL`.
- `exposure_time`:
  Exposure time. Only for manual exposure.
- `max_noise`:
  Maximum allowed noise. Data with higher noise will be filtered out.
- `compression`:
  `PNG` or `TIFF`.
- `jpeg_quality`:
  JPEG quality setting for 8 Bit image compression (mono8).
- `png_level`:
  PNG compression level. Only used if TIFF is not used.

## Topics

`/pico_flexx/camera_info`:
This topic publishes the camera intrinsic parameters.

`/pico_flexx/image_depth`:
This is the undistorted depth image. It is a 16-Bit image where each pixel is a distance measured in millimeters.

`/pico_flexx/image_depth/compressed`:
Compressed version of the depth image. Compatible to the image_transport package.

`/pico_flexx/image_mono16`:
This is the undistorted IR image. It is a 16-Bit image where each pixel is an intensity measurement.

`/pico_flexx/image_mono16/compressed`:
Compressed version of the mono16 image. Compatible to the image_transport package.

`/pico_flexx/image_mono8`:
This is the undistorted IR image. It is a 8-Bit image where each pixel is an intensity measurement.

`/pico_flexx/image_mono8/compressed`:
Compressed version of the mono8 image. Compatible to the image_transport package.

`/pico_flexx/image_noise`:
This is the undistorted noise image. It is a 16-Bit image where each pixel is a noise value of the corresponding depth pixel measured in 0.01 millimeters.

`/pico_flexx/image_noise/compressed`:
Compressed version of the noise image. Compatible to the image_transport package.

`/pico_flexx/points`:
This is the point cloud created by the sensor. It contains 5 fields, X, Y, Z, Intensity (16-Bit), Noise (16-Bit).
The X, Y, Z coordinates are undistorted. The point cloud is organized, so that the each point belongs to the pixel with the same index in one of the other images.
