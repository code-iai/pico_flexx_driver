/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * This file is part of pico_flexx_driver.
 *
 * pico_flexx_driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * pico_flexx_driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with pico_flexx_driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <nodelet/nodelet.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <royale.hpp>

#include <dynamic_reconfigure/server.h>
#include <pico_flexx_driver/pico_flexx_driverConfig.h>

#define PF_DEFAULT_NS       "pico_flexx"
#define PF_TF_LINK          "_link"
#define PF_TF_OPT_FRAME     "_optical_frame"
#define PF_TOPIC_INFO       "/camera_info"
#define PF_TOPIC_MONO8      "/image_mono8"
#define PF_TOPIC_MONO16     "/image_mono16"
#define PF_TOPIC_DEPTH      "/image_depth"
#define PF_TOPIC_NOISE      "/image_noise"
#define PF_TOPIC_CLOUD      "/points"

// fix for royale sdk definitions
#undef __PRETTY_FUNCTION__

// Set this to '0' to disable the extended colored output
#define EXTENDED_OUTPUT 1

#if EXTENDED_OUTPUT

#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

#define OUT_FUNCTION(NAME) ([](const std::string &name)\
{ \
  size_t end = name.rfind('(');\
  if(end == std::string::npos) end = name.size();\
  size_t begin = 1 + name.rfind(' ', end);\
  return name.substr(begin, end - begin);\
}(NAME))
#define OUT_AUX(FUNC_COLOR, MSG_COLOR, STREAM, MSG) STREAM(FUNC_COLOR "[" << OUT_FUNCTION(__PRETTY_FUNCTION__) << "] " MSG_COLOR << MSG << NO_COLOR)

#define OUT_DEBUG(msg) OUT_AUX(FG_BLUE, NO_COLOR, ROS_DEBUG_STREAM, msg)
#define OUT_INFO(msg) OUT_AUX(FG_GREEN, NO_COLOR, ROS_INFO_STREAM, msg)
#define OUT_WARN(msg) OUT_AUX(FG_YELLOW, FG_YELLOW, ROS_WARN_STREAM, msg)
#define OUT_ERROR(msg) OUT_AUX(FG_RED, FG_RED, ROS_ERROR_STREAM, msg)

#else

#define NO_COLOR        ""
#define FG_BLACK        ""
#define FG_RED          ""
#define FG_GREEN        ""
#define FG_YELLOW       ""
#define FG_BLUE         ""
#define FG_MAGENTA      ""
#define FG_CYAN         ""

#define OUT_DEBUG(msg) ROS_DEBUG_STREAM(msg)
#define OUT_INFO(msg) ROS_INFO_STREAM(msg)
#define OUT_WARN(msg) ROS_WARN_STREAM(msg)
#define OUT_ERROR(msg) ROS_WARN_STREAM(msg)

#endif

class PicoFlexx : public royale::IDepthDataListener, public royale::IExposureListener
{
private:
  enum Topics
  {
    CAMERA_INFO = 0,
    MONO_8,
    MONO_16,
    DEPTH,
    NOISE,
    CLOUD,
    COUNT
  };

  ros::NodeHandle nh, priv_nh;
  sensor_msgs::CameraInfo cameraInfo;
  std::vector<ros::Publisher> publisher;
  std::vector<bool> status;
  boost::recursive_mutex lockServer;
  dynamic_reconfigure::Server<pico_flexx_driver::pico_flexx_driverConfig> server;
  pico_flexx_driver::pico_flexx_driverConfig configMin, configMax, config;
  int cbExposureTime;

  std::unique_ptr<royale::ICameraDevice> cameraDevice;
  std::unique_ptr<royale::DepthData> data;

  std::mutex lockStatus, lockData, lockTiming;
  std::condition_variable cvNewData;
  bool running, newData, ignoreNewExposure;
  uint64_t frame, framesPerTiming, processTime, delayReceived;
  std::string baseNameTF;
  std::chrono::high_resolution_clock::time_point startTime;
  std::thread threadProcess;

public:
  PicoFlexx(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"))
    : royale::IDepthDataListener(), royale::IExposureListener(), nh(nh), priv_nh(priv_nh), server(lockServer)
  {
    running = false;
    newData = false;
    frame = 0;
    framesPerTiming = 25;
    processTime = 0;
    delayReceived = 0;
    publisher.resize(COUNT);
    status.resize(COUNT, false);

    config.use_case = 0;
    config.exposure_mode = 0;
    config.exposure_time = 1000;
    config.max_noise = 0.07;
    config.range_factor = 2.0;

    configMin.use_case = 0;
    configMin.exposure_mode = 0;
    configMin.exposure_time = 50;
    configMin.max_noise = 0.0;
    configMin.range_factor = 0.0;

    configMax.use_case = 5;
    configMax.exposure_mode = 1;
    configMax.exposure_time = 2000;
    configMax.max_noise = 0.10;
    configMax.range_factor = 7.0;
  }

  ~PicoFlexx()
  {
  }

  void start()
  {
    if(!initialize())
    {
      return;
    }
    running = true;

    threadProcess = std::thread(&PicoFlexx::process, this);

    OUT_INFO("waiting for clients to connect");
  }

  void stop()
  {
    cameraDevice->stopCapture();
    running = false;

    threadProcess.join();
    return;
  }

  void onNewData(const royale::DepthData *data)
  {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    lockTiming.lock();
    delayReceived += (now.time_since_epoch() - std::chrono::duration_cast<std::chrono::nanoseconds>(data->timeStamp)).count();
    lockTiming.unlock();

    lockData.lock();
    this->data = std::unique_ptr<royale::DepthData>(new royale::DepthData);
    this->data->version = data->version;
    this->data->timeStamp = data->timeStamp;
    this->data->width = data->width;
    this->data->height = data->height;
    this->data->exposureTimes = data->exposureTimes;
    this->data->points = data->points;
    newData = true;
    lockData.unlock();
    cvNewData.notify_one();
  }

  void onNewExposure(const uint32_t newExposureTime)
  {
    if(ignoreNewExposure)
    {
      ignoreNewExposure = false;
      cbExposureTime = (int)newExposureTime;
      return;
    }

    if(config.exposure_time == (int)newExposureTime)
    {
      return;
    }

    OUT_DEBUG("exposure changed: " FG_YELLOW << newExposureTime);
    config.exposure_time = (int)newExposureTime;
    server.updateConfig(config);
  }

  void callbackTopicStatus()
  {
    lockStatus.lock();
    bool clientsConnected = false;
    for(size_t i = 0; i < COUNT; ++i)
    {
      status[i] = publisher[i].getNumSubscribers() > 0;
      clientsConnected = clientsConnected || status[i];
    }

    bool isCapturing(false);
    cameraDevice->isCapturing(isCapturing);
    if(clientsConnected && !isCapturing)
    {
      OUT_INFO("client connected. starting device...");

      lockTiming.lock();
      processTime = 0;
      frame = 0;
      delayReceived = 0;
      lockTiming.unlock();
      ignoreNewExposure = config.exposure_mode == 0; // ignore if manual mode

      if(cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
      {
        OUT_ERROR("could not start capture!");
        running = false;
        ros::shutdown();
      }

      if(config.exposure_mode == 0 && cbExposureTime != config.exposure_time)
      {
        setExposure((uint32_t)config.exposure_time);
      }
    }
    else if(!clientsConnected && isCapturing)
    {
      OUT_INFO("no clients connected. stopping device...");
      if(cameraDevice->stopCapture() != royale::CameraStatus::SUCCESS)
      {
        OUT_ERROR("could not stop capture!");
        running = false;
        ros::shutdown();
      }
    }
    lockStatus.unlock();
  }

  void callbackConfig(pico_flexx_driver::pico_flexx_driverConfig &config, uint32_t level)
  {
    if(level == 0xFFFFFFFF)
    {
      return;
    }

    if(level & 0x01)
    {
      royale::Vector<royale::String> useCases;
      cameraDevice->getUseCases(useCases);
      OUT_INFO("reconfigured use case: " << FG_CYAN << useCases.at(config.use_case) << NO_COLOR);
      if(!setUseCase((size_t)config.use_case))
      {
        config.use_case = this->config.use_case;
        return;
      }
      this->config.use_case = config.use_case;
    }

    if(level & 0x02)
    {
      OUT_INFO("reconfigured exposure_mode: " << FG_CYAN << (config.exposure_mode == 1 ? "automatic" : "manual") << NO_COLOR);

      if(!setExposureMode(config.exposure_mode == 1))
      {
        config.exposure_mode = this->config.exposure_mode;
        return;
      }
      this->config.exposure_mode = config.exposure_mode;
    }

    if(level & 0x04)
    {
      OUT_INFO("reconfigured exposure_time: " << FG_CYAN << config.exposure_time << NO_COLOR);
      royale::ExposureMode exposureMode;
      cameraDevice->getExposureMode(exposureMode);
      bool isCapturing(false);
      cameraDevice->isCapturing(isCapturing);
      if(exposureMode == royale::ExposureMode::AUTOMATIC || (isCapturing && !setExposure((uint32_t)config.exposure_time)))
      {
        config.exposure_time = this->config.exposure_time;
        return;
      }
      this->config.exposure_time = config.exposure_time;
    }

    if(level & 0x08)
    {
      OUT_INFO("reconfigured max_noise: " << FG_CYAN << config.max_noise << " meters" << NO_COLOR);
      lockStatus.lock();
      this->config.max_noise = config.max_noise;
      lockStatus.unlock();
    }

    if(level & 0x10)
    {
      OUT_INFO("reconfigured range_factor: " << FG_CYAN << config.range_factor << " meters" << NO_COLOR);
      lockStatus.lock();
      this->config.range_factor = config.range_factor;
      lockStatus.unlock();
    }

    if(level & 0x01 || level & 0x02)
    {
      royale::Pair<uint32_t, uint32_t> limits;
      cameraDevice->getExposureLimits(limits);
      configMin.exposure_time = limits.first;
      configMax.exposure_time = limits.second;
      server.setConfigMin(configMin);
      server.setConfigMax(configMax);
    }
  }

private:

  bool initialize()
  {
    if(running)
    {
      OUT_ERROR("driver is already running!");
      return false;
    }

    bool automaticExposure;
    int32_t useCase, exposureTime, queueSize;
    std::string sensor, baseName;
    double maxNoise, rangeFactor;

    priv_nh.param("base_name", baseName, std::string(PF_DEFAULT_NS));
    priv_nh.param("sensor", sensor, std::string(""));
    priv_nh.param("use_case", useCase, 0);
    priv_nh.param("automatic_exposure", automaticExposure, true);
    priv_nh.param("exposure_time", exposureTime, 1000);
    priv_nh.param("max_noise", maxNoise, 0.7);
    priv_nh.param("range_factor", rangeFactor, 2.0);
    priv_nh.param("queue_size", queueSize, 2);
    priv_nh.param("base_name_tf", baseNameTF, baseName);

    OUT_INFO("parameter:" << std::endl
             << "         base_name: " FG_CYAN << baseName << NO_COLOR << std::endl
             << "            sensor: " FG_CYAN << (sensor.empty() ? "default" : sensor) << NO_COLOR << std::endl
             << "          use_case: " FG_CYAN << useCase << NO_COLOR << std::endl
             << "automatic_exposure: " FG_CYAN << (automaticExposure ? "true" : "false") << NO_COLOR << std::endl
             << "     exposure_time: " FG_CYAN << exposureTime << NO_COLOR << std::endl
             << "         max_noise: " FG_CYAN << maxNoise << " meters" NO_COLOR << std::endl
             << "      range_factor: " FG_CYAN << rangeFactor << NO_COLOR << std::endl
             << "        queue_size: " FG_CYAN << queueSize << NO_COLOR << std::endl
             << "      base_name_tf: " FG_CYAN << baseNameTF << NO_COLOR);

    uint32_t major, minor, patch, build;
    royale::getVersion(major, minor, patch, build);
    OUT_INFO("libroyale version: " FG_CYAN << major << '.' << minor << '.' << patch << '.' << build << NO_COLOR);

    royale::LensParameters params;
    if(!selectCamera(sensor)
       || !setUseCase((size_t)useCase)
       || !setExposureMode(automaticExposure)
       || !getCameraSettings(params)
       || !createCameraInfo(params))
    {
      return false;
    }

    if(cameraDevice->registerExposureListener(this) != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("could not register exposure listener!");
      return false;
    }


    if(cameraDevice->registerDataListener(this) != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("could not register data listener!");
      return false;
    }

    setTopics(baseName, queueSize);

    royale::Pair<uint32_t, uint32_t> limits;
    cameraDevice->getExposureLimits(limits);
    configMin.exposure_time = limits.first;
    configMax.exposure_time = limits.second;
    server.setConfigMin(configMin);
    server.setConfigMax(configMax);

    royale::Vector<royale::String> useCases;
    cameraDevice->getUseCases(useCases);

    config.use_case = std::max(std::min(useCase, (int)useCases.size() - 1), 0);
    config.exposure_mode = automaticExposure ? 1 : 0;
    config.exposure_time = std::max(std::min(exposureTime, configMax.exposure_time), configMin.exposure_time);
    config.max_noise = std::max(std::min(maxNoise, configMax.max_noise), configMin.max_noise);
    config.range_factor = std::max(std::min(rangeFactor, configMax.range_factor), configMin.range_factor);

    server.setConfigDefault(config);

    dynamic_reconfigure::Server<pico_flexx_driver::pico_flexx_driverConfig>::CallbackType f;
    f = boost::bind(&PicoFlexx::callbackConfig, this, _1, _2);
    server.setCallback(f);

    return true;
  }

  void setTopics(const std::string &baseName, const int32_t queueSize)
  {
    publisher.resize(COUNT);
    ros::SubscriberStatusCallback cb = boost::bind(&PicoFlexx::callbackTopicStatus, this);
    publisher[CAMERA_INFO] = nh.advertise<sensor_msgs::CameraInfo>(baseName + PF_TOPIC_INFO, queueSize, cb, cb);
    publisher[MONO_8] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_MONO8, queueSize, cb, cb);
    publisher[MONO_16] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_MONO16, queueSize, cb, cb);
    publisher[DEPTH] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_DEPTH, queueSize, cb, cb);
    publisher[NOISE] = nh.advertise<sensor_msgs::Image>(baseName + PF_TOPIC_NOISE, queueSize, cb, cb);
    publisher[CLOUD] = nh.advertise<sensor_msgs::PointCloud2>(baseName + PF_TOPIC_CLOUD, queueSize, cb, cb);
  }

  bool selectCamera(const std::string &id)
  {
    royale::String _id = id;
    royale::CameraManager manager;

    royale::Vector<royale::String> camlist = manager.getConnectedCameraList();
    if(camlist.empty())
    {
      OUT_ERROR("no cameras connected!");
      return false;
    }

    OUT_INFO("Detected " << camlist.size() << " camera(s):");

    if(id.empty())
    {
      _id = camlist[0];
    }

    int index = -1;
    for(size_t i = 0; i < camlist.size(); ++i)
    {
      if(_id == camlist[i])
      {
        index = (int)i;
        OUT_INFO("  " << i << ": " FG_CYAN << camlist[i] << FG_YELLOW " (selected)" << NO_COLOR);
      }
      else
      {
        OUT_INFO("  " << i << ": " FG_CYAN << camlist[i] << NO_COLOR);
      }
    }

    if(index < 0)
    {
      OUT_ERROR("camera with id '" << _id << "' not found!");
      return false;
    }
    cameraDevice = manager.createCamera(camlist[index]);

    if(cameraDevice == nullptr)
    {
      OUT_ERROR("cannot create camera device!");
      return false;
    }

    if(cameraDevice->initialize() != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("cannot initialize camera device");
      return false;
    }
    return true;
  }

  bool getCameraSettings(royale::LensParameters &params)
  {
    bool ret = true;
    royale::Vector<royale::String> useCases;
    cameraDevice->getUseCases(useCases);
    royale::String useCase;
    cameraDevice->getCurrentUseCase(useCase);
    royale::ExposureMode expMode;
    cameraDevice->getExposureMode(expMode);
    royale::Pair<uint32_t, uint32_t> limits;
    cameraDevice->getExposureLimits(limits);
    royale::Vector<royale::Pair<royale::String,royale::String>> info;
    cameraDevice->getCameraInfo(info);

    royale::String cameraProperty;
    cameraDevice->getCameraName(cameraProperty);
    OUT_INFO("camera name: " FG_CYAN << cameraProperty << NO_COLOR);
    cameraDevice->getId(cameraProperty);
    OUT_INFO("camera id: " FG_CYAN << cameraProperty << NO_COLOR);
    royale::CameraAccessLevel accessLevel;
    cameraDevice->getAccessLevel(accessLevel);
    OUT_INFO("access level: " FG_CYAN "L" << (int)accessLevel+ 1 << NO_COLOR);
    OUT_INFO("exposure mode: " FG_CYAN << (expMode == royale::ExposureMode::AUTOMATIC ? "automatic" : "manual") << NO_COLOR);
    OUT_INFO("exposure limits: " FG_CYAN << limits.first << " / " << limits.second << NO_COLOR);

    OUT_INFO("camera info:");
    if(info.empty())
    {
      OUT_INFO("  no camera info available!");
    }
    else
    {
      for(size_t i = 0; i < info.size(); ++i)
      {
        OUT_INFO("  " << info[i].first << ": " FG_CYAN << info[i].second << NO_COLOR);
      }
    }

    OUT_INFO("use cases:");
    if(useCases.empty())
    {
      OUT_ERROR("  no use cases available!");
      ret = false;
    }
    else
    {
      for(size_t i = 0; i < useCases.size(); ++i)
      {
        OUT_INFO("  " << i << ": " FG_CYAN << useCases[i] << (useCases[i] == useCase ? FG_YELLOW " (selected)" : "") << NO_COLOR);
      }
    }

    if(cameraDevice->getLensParameters(params) == royale::CameraStatus::SUCCESS)
    {
      OUT_INFO("camera intrinsics:");
      uint16_t maxSensorWidth;
      cameraDevice->getMaxSensorWidth(maxSensorWidth);
      OUT_INFO("width: " FG_CYAN << maxSensorWidth << NO_COLOR);

      uint16_t maxSensorHeight;
      cameraDevice->getMaxSensorHeight(maxSensorHeight);
      OUT_INFO("height: " FG_CYAN << maxSensorHeight << NO_COLOR);
      OUT_INFO("fx: " FG_CYAN << params.principalPoint.first
               << NO_COLOR ", fy: " FG_CYAN << params.principalPoint.second
               << NO_COLOR ", cx: " FG_CYAN << params.focalLength.first
               << NO_COLOR ", cy: " FG_CYAN << params.focalLength.second << NO_COLOR);
      if(params.distortionRadial.size() == 3)
      {
        OUT_INFO("k1: " FG_CYAN << params.distortionRadial[0]
                 << NO_COLOR ", k2: " FG_CYAN << params.distortionRadial[1]
                 << NO_COLOR ", p1: " FG_CYAN << params.distortionTangential.first
                 << NO_COLOR ", p2: " FG_CYAN << params.distortionTangential.second
                 << NO_COLOR ", k3: " FG_CYAN << params.distortionRadial[2] << NO_COLOR);
      }
      else
      {
        OUT_ERROR("distortion model unknown!");
        ret = false;
      }
    }
    else
    {
      OUT_ERROR("could not get lens parameter!");
      ret = false;
    }
    return ret;
  }

  bool setUseCase(const size_t idx)
  {
    royale::Vector<royale::String> useCases;
    cameraDevice->getUseCases(useCases);
    royale::String useCase;
    cameraDevice->getCurrentUseCase(useCase);

    if(useCases.empty())
    {
      OUT_ERROR("no use cases available!");
      return false;
    }

    if(idx >= useCases.size())
    {
      OUT_ERROR("use case invalid!");
      return false;
    }

    if(useCases[idx] == useCase)
    {
      OUT_INFO("use case not changed!");
      return true;
    }

    if(cameraDevice->setUseCase(useCases[idx]) != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("could not set use case!");
      return false;
    }
    OUT_INFO("use case changed to: " FG_YELLOW << useCases[idx]);

    std::string name;
    useCases[idx].toStdString(name);
    size_t end = name.find("FPS");
    size_t start = name.rfind('_', end);

    if(end == std::string::npos || start == std::string::npos)
    {
      OUT_WARN("could not extract frames per second from operation mode.");
      lockTiming.lock();
      framesPerTiming = 100;
      lockTiming.unlock();
      return true;
    }
    start += 1;

    std::string fpsString = name.substr(start, end - start);
    if(fpsString.find_first_not_of("0123456789") != std::string::npos)
    {
      OUT_WARN("could not extract frames per second from operation mode.");
      lockTiming.lock();
      framesPerTiming = 100;
      lockTiming.unlock();
      return true;
    }

    lockTiming.lock();
    framesPerTiming = std::stoi(fpsString) * 5;
    lockTiming.unlock();
    return true;
  }

  bool setExposureMode(const bool automatic)
  {
    royale::ExposureMode newMode = automatic ? royale::ExposureMode::AUTOMATIC : royale::ExposureMode::MANUAL;

    royale::ExposureMode exposureMode;
    cameraDevice->getExposureMode(exposureMode);
    if(newMode == exposureMode)
    {
      OUT_INFO("exposure mode not changed!");
      return true;
    }

    if(cameraDevice->setExposureMode(newMode) != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("could not set operation mode!");
      return false;
    }

    OUT_INFO("exposure mode changed to: " FG_YELLOW << (automatic ? "automatic" : "manual"));
    return true;
  }

  bool setExposure(const uint32_t exposure)
  {
    royale::Pair<uint32_t, uint32_t> limits;
    cameraDevice->getExposureLimits(limits);

    if(exposure < limits.first || exposure > limits.second)
    {
      OUT_ERROR("exposure outside of limits!");
      return false;
    }

    if(cameraDevice->setExposureTime(exposure) != royale::CameraStatus::SUCCESS)
    {
      OUT_ERROR("could not set exposure time!");
      return false;
    }

    OUT_INFO("exposure time changed to: " FG_YELLOW << exposure);
    return true;
  }

  bool createCameraInfo(const royale::LensParameters &params)
  {
    if(params.distortionRadial.size() != 3)
    {
      OUT_ERROR("distortion model unknown!" << params.distortionRadial.size());
      return false;
    }

    uint16_t maxSensorHeight;
    cameraDevice->getMaxSensorHeight(maxSensorHeight);
    cameraInfo.height = maxSensorHeight;

    uint16_t maxSensorWidth;
    cameraDevice->getMaxSensorWidth(maxSensorWidth);
    cameraInfo.width = maxSensorWidth;

    cameraInfo.K[0] = params.focalLength.first;
    cameraInfo.K[1] = 0;
    cameraInfo.K[2] = params.principalPoint.first;
    cameraInfo.K[3] = 0;
    cameraInfo.K[4] = params.focalLength.second;
    cameraInfo.K[5] = params.principalPoint.second;
    cameraInfo.K[6] = 0;
    cameraInfo.K[7] = 0;
    cameraInfo.K[8] = 1;

    cameraInfo.R[0] = 1;
    cameraInfo.R[1] = 0;
    cameraInfo.R[2] = 0;
    cameraInfo.R[3] = 0;
    cameraInfo.R[4] = 1;
    cameraInfo.R[5] = 0;
    cameraInfo.R[6] = 0;
    cameraInfo.R[7] = 0;
    cameraInfo.R[8] = 1;

    cameraInfo.P[0] = params.focalLength.first;
    cameraInfo.P[1] = 0;
    cameraInfo.P[2] = params.principalPoint.first;
    cameraInfo.P[3] = 0;
    cameraInfo.P[4] = 0;
    cameraInfo.P[5] = params.focalLength.second;
    cameraInfo.P[6] = params.principalPoint.second;
    cameraInfo.P[7] = 0;
    cameraInfo.P[8] = 0;
    cameraInfo.P[9] = 0;
    cameraInfo.P[10] = 1;
    cameraInfo.P[11] = 0;

    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.D.resize(5);
    cameraInfo.D[0] = params.distortionRadial[0];
    cameraInfo.D[1] = params.distortionRadial[1];
    cameraInfo.D[2] = params.distortionTangential.first;
    cameraInfo.D[3] = params.distortionTangential.second;
    cameraInfo.D[4] = params.distortionRadial[2];

    return true;
  }

  void process()
  {
    std::unique_ptr<royale::DepthData> data;
    sensor_msgs::CameraInfoPtr msgCameraInfo;
    sensor_msgs::ImagePtr msgMono8, msgMono16, msgDepth, msgNoise;
    sensor_msgs::PointCloud2Ptr msgCloud;

    msgCameraInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
    msgMono8 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    msgMono16 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    msgDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    msgNoise = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    msgCloud = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);

    std::chrono::high_resolution_clock::time_point start, end;
    std::unique_lock<std::mutex> lock(lockData);
    while(running && ros::ok())
    {
      if(!cvNewData.wait_for(lock, std::chrono::milliseconds(300), [this] { return this->newData; }))
      {
        continue;
      }

      start = std::chrono::high_resolution_clock::now();
      this->data.swap(data);
      newData = false;
      lock.unlock();

      lockStatus.lock();
      extractData(*data, msgCameraInfo, msgCloud, msgMono8, msgMono16, msgDepth, msgNoise);
      publish(msgCameraInfo, msgCloud, msgMono8, msgMono16, msgDepth, msgNoise);
      lockStatus.unlock();

      end = std::chrono::high_resolution_clock::now();
      lockTiming.lock();
      processTime += (end - start).count();

      timings();
      lockTiming.unlock();

      lock.lock();
    }
  }

  void extractData(const royale::DepthData &data, sensor_msgs::CameraInfoPtr &msgCameraInfo, sensor_msgs::PointCloud2Ptr &msgCloud,
                   sensor_msgs::ImagePtr &msgMono8, sensor_msgs::ImagePtr &msgMono16, sensor_msgs::ImagePtr &msgDepth, sensor_msgs::ImagePtr &msgNoise) const
  {
    std_msgs::Header header;
    header.frame_id = baseNameTF + PF_TF_OPT_FRAME;
    header.seq = 0;
    header.stamp.fromNSec(std::chrono::duration_cast<std::chrono::nanoseconds>(data.timeStamp).count());

    if(status[CAMERA_INFO])
    {
      *msgCameraInfo = cameraInfo;
      msgCameraInfo->header = header;
      msgCameraInfo->height = data.height;
      msgCameraInfo->width = data.width;
    }

    if(!(status[MONO_8] || status[MONO_16] || status[DEPTH] || status[NOISE] || status[CLOUD]))
    {
      return;
    }

    msgMono16->header = header;
    msgMono16->height = data.height;
    msgMono16->width = data.width;
    msgMono16->is_bigendian = false;
    msgMono16->encoding = sensor_msgs::image_encodings::MONO16;
    msgMono16->step = (uint32_t)(sizeof(uint16_t) * data.width);
    msgMono16->data.resize(sizeof(uint16_t) * data.points.size());

    msgDepth->header = header;
    msgDepth->height = data.height;
    msgDepth->width = data.width;
    msgDepth->is_bigendian = false;
    msgDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    msgDepth->step = (uint32_t)(sizeof(float) * data.width);
    msgDepth->data.resize(sizeof(float) * data.points.size());

    msgNoise->header = header;
    msgNoise->height = data.height;
    msgNoise->width = data.width;
    msgNoise->is_bigendian = false;
    msgNoise->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    msgNoise->step = (uint32_t)(sizeof(float) * data.width);
    msgNoise->data.resize(sizeof(float) * data.points.size());

    msgCloud->header = header;
    msgCloud->height = data.height;
    msgCloud->width = data.width;
    msgCloud->is_bigendian = false;
    msgCloud->is_dense = false;
    msgCloud->point_step = (uint32_t)(5 * sizeof(float));
    msgCloud->row_step = (uint32_t)(5 * sizeof(float) * data.width);
    msgCloud->fields.resize(6);
    msgCloud->fields[0].name = "x";
    msgCloud->fields[0].offset = 0;
    msgCloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msgCloud->fields[0].count = 1;
    msgCloud->fields[1].name = "y";
    msgCloud->fields[1].offset = msgCloud->fields[0].offset + (uint32_t)sizeof(float);
    msgCloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msgCloud->fields[1].count = 1;
    msgCloud->fields[2].name = "z";
    msgCloud->fields[2].offset = msgCloud->fields[1].offset + (uint32_t)sizeof(float);
    msgCloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msgCloud->fields[2].count = 1;
    msgCloud->fields[3].name = "noise";
    msgCloud->fields[3].offset = msgCloud->fields[2].offset + (uint32_t)sizeof(float);
    msgCloud->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    msgCloud->fields[3].count = 1;
    msgCloud->fields[4].name = "intensity";
    msgCloud->fields[4].offset = msgCloud->fields[3].offset + (uint32_t)sizeof(float);
    msgCloud->fields[4].datatype = sensor_msgs::PointField::UINT16;
    msgCloud->fields[4].count = 1;
    msgCloud->fields[5].name = "gray";
    msgCloud->fields[5].offset = msgCloud->fields[4].offset + (uint32_t)sizeof(uint16_t);
    msgCloud->fields[5].datatype = sensor_msgs::PointField::UINT8;
    msgCloud->fields[5].count = 1;
    msgCloud->data.resize(5 * sizeof(float) * data.points.size());

    const float invalid = std::numeric_limits<float>::quiet_NaN();
    const float maxNoise = (float)config.max_noise;
    const royale::DepthPoint *itI = &data.points[0];
    float *itCX = (float *)&msgCloud->data[0];
    float *itCY = itCX + 1;
    float *itCZ = itCY + 1;
    float *itCN = itCZ + 1;
    uint16_t *itCM = (uint16_t *)(itCN + 1);
    float *itD = (float *)&msgDepth->data[0];
    float *itN = (float *)&msgNoise->data[0];
    uint16_t *itM = (uint16_t *)&msgMono16->data[0];
    for(size_t i = 0; i < data.points.size(); ++i, ++itI, itCX += 5, itCY += 5, itCZ += 5, itCN += 5, itCM += 10, ++itD, ++itM, ++itN)
    {
      if(itI->depthConfidence && itI->noise < maxNoise)
      {
        *itCX = itI->x;
        *itCY = itI->y;
        *itCZ = itI->z;
        *itCN = itI->noise;
        *itD = itI->z;
        *itN = itI->noise;
      }
      else
      {
        *itCX = invalid;
        *itCY = invalid;
        *itCZ = invalid;
        *itCN = 0.0f;
        *itD = 0.0f;
        *itN = 0.0f;
      }
      *itCM = itI->grayValue;
      *itM = itI->grayValue;
    }

    computeMono8(msgMono16, msgMono8, msgCloud);
  }

  void computeMono8(const sensor_msgs::ImageConstPtr &msgMono16, sensor_msgs::ImagePtr &msgMono8, sensor_msgs::PointCloud2Ptr &msgCloud) const
  {
    msgMono8->header = msgMono16->header;
    msgMono8->height = msgMono16->height;
    msgMono8->width = msgMono16->width;
    msgMono8->is_bigendian = msgMono16->is_bigendian;
    msgMono8->encoding = sensor_msgs::image_encodings::MONO8;
    msgMono8->step = (uint32_t)(sizeof(uint8_t) * msgMono8->width);
    msgMono8->data.resize(sizeof(uint8_t) * msgMono8->width * msgMono8->height);

    const uint16_t *pMono16 = (const uint16_t *)&msgMono16->data[0];
    uint8_t *pMono8 = (uint8_t *)&msgMono8->data[0];
    const size_t size = msgMono8->width * msgMono8->height;

    uint64_t sum = 0;
    uint64_t count = 0;

    const uint16_t *itI = pMono16;
    for(size_t i = 0; i < size; ++i, ++itI)
    {
      if(*itI)
      {
        sum += *itI;
        ++count;
      }
    }
    const double average = (double)sum / (double)count;
    double deviation = 0;

    itI = pMono16;
    for(size_t i = 0; i < size; ++i, ++itI)
    {
      if(*itI)
      {
        const double diff = (double) * itI - average;
        deviation += (diff * diff);
      }
    }
    deviation = sqrt(deviation / ((double)count - 1.0));

    const uint16_t minV = (uint16_t)std::max(average - config.range_factor * deviation, 0.0);
    const uint16_t maxV = (uint16_t)(std::min(average + config.range_factor * deviation, 65535.0) - minV);
    const double maxVF = 255.0 / (double)maxV;
    uint8_t *itO = pMono8;
    uint8_t *itP = ((uint8_t *)&msgCloud->data[0]) + 18;
    itI = pMono16;
    for(size_t i = 0; i < size; ++i, ++itI, ++itO, itP += 20)
    {
      uint16_t v = *itI;
      if(v < minV)
      {
        v = 0;
      }
      else
      {
        v = (uint16_t)(v - minV);
      }
      if(v > maxV)
      {
        v = maxV;
      }

      const uint8_t newV = (uint8_t)((double)v * maxVF);
      *itO = newV;
      *itP = newV;
    }
  }

  void publish(sensor_msgs::CameraInfoPtr &msgCameraInfo, sensor_msgs::PointCloud2Ptr &msgCloud,
               sensor_msgs::ImagePtr &msgMono8, sensor_msgs::ImagePtr &msgMono16,
               sensor_msgs::ImagePtr &msgDepth, sensor_msgs::ImagePtr &msgNoise) const
  {
    if(status[CAMERA_INFO])
    {
      publisher[CAMERA_INFO].publish(msgCameraInfo);
      msgCameraInfo = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
    }
    if(status[MONO_8])
    {
      publisher[MONO_8].publish(msgMono8);
      msgMono8 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    }
    if(status[MONO_16])
    {
      publisher[MONO_16].publish(msgMono16);
      msgMono16 = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    }
    if(status[DEPTH])
    {
      publisher[DEPTH].publish(msgDepth);
      msgDepth = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    }
    if(status[NOISE])
    {
      publisher[NOISE].publish(msgNoise);
      msgNoise = sensor_msgs::ImagePtr(new sensor_msgs::Image);
    }
    if(status[CLOUD])
    {
      publisher[CLOUD].publish(msgCloud);
      msgCloud = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2);
    }
  }

  void timings()
  {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    if(!frame)
    {
      startTime = now;
    }
    else if(frame % framesPerTiming == 0)
    {
      double timePerFrame, framesPerSecond, avgDelay;

      timePerFrame = (double)(processTime / framesPerTiming) / 1000000.0;
      framesPerSecond = (double)framesPerTiming / ((double)(now - startTime).count() / 1000000000.0);
      avgDelay = ((double)delayReceived / (double)framesPerTiming) / 1000000.0;

      processTime = 0;
      startTime = now;
      delayReceived = 0;
      OUT_INFO("processing: " FG_YELLOW "~" << std::setprecision(4) << timePerFrame << " ms." NO_COLOR
               " fps: " FG_YELLOW "~" << framesPerSecond << " Hz" NO_COLOR
               " delay: " FG_YELLOW "~" << avgDelay << " ms." NO_COLOR);
    }
    ++frame;
  }
};

class PicoFlexxNodelet : public nodelet::Nodelet
{
private:
  PicoFlexx *picoFlexx;

public:
  PicoFlexxNodelet() : Nodelet(), picoFlexx(NULL)
  {
  }

  ~PicoFlexxNodelet()
  {
    if(picoFlexx)
    {
      picoFlexx->stop();
      delete picoFlexx;
    }
  }

  virtual void onInit()
  {
    picoFlexx = new PicoFlexx(getNodeHandle(), getPrivateNodeHandle());
    picoFlexx->start();
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(PicoFlexxNodelet, nodelet::Nodelet)

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, PF_DEFAULT_NS);

  PicoFlexx picoFlexx;
  picoFlexx.start();
  ros::spin();
  picoFlexx.stop();
  return 0;
}
