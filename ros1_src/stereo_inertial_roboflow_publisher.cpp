/**
	stereo_inertial_roboflow_publisher.cpp
	Purpose: ROS Implementation for OAK-D camera
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
*/

#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/ObjectHypothesis.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <stereo_msgs/DisparityImage.h>

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <cassert> // assert
#include <bits/stdc++.h>

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>
//#include <depthai_bridge/NeuralNetworkDetectionConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

#include "depthai/depthai.hpp"

#include "jsoncpp/json/json.h"

#include "processing.h"
#include "pipe.h"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};


int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_inertial_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath, nnConfigPath;
    std::string monoResolution = "720p", rgbResolution = "1080p";
    int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, expTime, sensIso;
    int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel, rectify,  manualExposure;
    bool enableNeuralNetworkDetection, enableDotProjector, enableFloodLight;
    bool usb2Mode, poeMode;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectormA, floodLightmA;
    float confidenceThreshold = 0.5;
    float overlapThreshold = 0.5;
    int boxNeighbors;
    std::string nnName(BLOB_NAME), nnConfig(BLOB_NAME);  // Set your blob name for the model here

    badParams += !pnh.getParam("mxId", mxId);
    badParams += !pnh.getParam("usb2Mode", usb2Mode);
    badParams += !pnh.getParam("poeMode", poeMode);

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("imuMode", imuModeParam);

    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("rectify", rectify);

    badParams += !pnh.getParam("stereo_fps", stereo_fps);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
    badParams += !pnh.getParam("monoResolution", monoResolution);
    badParams += !pnh.getParam("rgbResolution", rgbResolution);
    badParams += !pnh.getParam("manualExposure", manualExposure);
    badParams += !pnh.getParam("expTime", expTime);
    badParams += !pnh.getParam("sensIso", sensIso);

    badParams += !pnh.getParam("rgbScaleNumerator", rgbScaleNumerator);
    badParams += !pnh.getParam("rgbScaleDinominator", rgbScaleDinominator);
    badParams += !pnh.getParam("previewWidth", previewWidth);
    badParams += !pnh.getParam("previewHeight", previewHeight);

    badParams += !pnh.getParam("angularVelCovariance", angularVelCovariance);
    badParams += !pnh.getParam("linearAccelCovariance", linearAccelCovariance);
    badParams += !pnh.getParam("enableNeuralNetworkDetection", enableNeuralNetworkDetection);


    // Applies only to PRO model
    badParams += !pnh.getParam("enableDotProjector", enableDotProjector);
    badParams += !pnh.getParam("enableFloodLight", enableFloodLight);
    badParams += !pnh.getParam("dotProjectormA", dotProjectormA);
    badParams += !pnh.getParam("floodLightmA", floodLightmA);

    if(enableNeuralNetworkDetection){
        badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);
        badParams += !pnh.getParam("confidenceThreshold", confidenceThreshold);
        badParams += !pnh.getParam("overlapThreshold", overlapThreshold);
        badParams += !pnh.getParam("boxNeighbors", boxNeighbors);
        badParams += !pnh.getParam("nnName", nnName);
        badParams += !pnh.getParam("nnConfig", nnConfig);
    }

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
    }


    if(enableNeuralNetworkDetection){
        if(resourceBaseFolder.empty()) {
            throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
        }
        nnPath = resourceBaseFolder + "/" + nnName;
        nnConfigPath = resourceBaseFolder + "/" + nnConfig;
        std::cout << " NeuralNetworkDetection enabled: nnPath: " << nnPath << std::endl;
        std::cout << " NeuralNetworkDetection enabled: nnConfigPath: " << nnConfigPath << std::endl;
    }
    else
        std::cout << " NeuralNetworkDetection disabled" << std::endl;

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);


    int nn_width, nn_height;
    std::vector<std::string> class_names;
    if(enableNeuralNetworkDetection){
        std::ifstream file(nnConfigPath);
        // json reader
        Json::Reader reader;
        // this will contain complete JSON data
        Json::Value completeJsonData;
        // reader reads the data and stores it in completeJsonData
        reader.parse(file, completeJsonData);

        nn_width = std::stoi(completeJsonData["environment"]["RESOLUTION"].asString());
        nn_height = nn_width;

        const Json::Value classes = completeJsonData["class_names"];

        for(int i = 0; i < classes.size(); i++){
            class_names.push_back(classes[i].asString());
        }
    }


    dai::Pipeline pipeline;
    int image_width, image_height;
    bool isDeviceFound = false;
    std::tie(pipeline, image_width, image_height) = createPipeline(
                                                        enableNeuralNetworkDetection,
                                                        lrcheck,
                                                        extended,
                                                        subpixel,
                                                        rectify,
                                                        stereo_fps,
                                                        confidence,
                                                        LRchecktresh,
                                                        monoResolution,
                                                        rgbResolution,
                                                        rgbScaleNumerator,
                                                        rgbScaleDinominator,
                                                        previewWidth,
                                                        previewHeight,
                                                        nnPath,
                                                        nnConfigPath);



    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for(auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                if(poeMode) {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                } else {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId
                                         + "\" is already booted on different process.  \"");
            }
        } else if(mxId.empty()) {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if(!isDeviceFound) {
        throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    if(!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    // Apply camera controls
    auto controlQueue = device->getInputQueue("control");

    //Set manual exposure
    if(manualExposure){
        dai::CameraControl ctrl;
        ctrl.setManualExposure(expTime, sensIso);
        controlQueue->send(ctrl);
    }


    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    stereoQueue = device->getOutputQueue("depth", 30, false);

    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;

    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorBrightness(dotProjectormA);
        }

        if(enableFloodLight) {
            device->setIrFloodLightBrightness(floodLightmA);
        }
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();


    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);

    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, image_width, image_height);


    auto depthconverter = rgbConverter;
    auto depthCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, image_width, image_height);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        stereoQueue,
        pnh,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                  &depthconverter,  // since the converter has the same frame name
                                    // and image type is also same we can reuse it
                  std::placeholders::_1,
                  std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();

    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, image_width, image_height);
    auto imgQueue = device->getOutputQueue("rgb", 30, false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        imgQueue,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    if(enableNeuralNetworkDetection) {
        auto previewQueue = device->getOutputQueue("preview", 30, false);
        auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);

        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
            previewQueue,
            pnh,
            std::string("color/preview/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            previewCameraInfo,
            "color/preview");
        previewPublish.addPublisherCallback();

#define DETECTION_THREAD        1
#if DETECTION_THREAD

std::thread detection_task(DetectionTask,
                            device,
                            "detections",
                            "spatialData",
                            "spatialCalcConfig",
                            tfPrefix + "_rgb_camera_optical_frame" + "/color/detections",
                            image_width,  image_height,
                            nn_width, nn_height,
                            class_names,
                            confidenceThreshold,
                            overlapThreshold,
                            boxNeighbors);

#else

        auto detectionQueue = device->getOutputQueue("detections", 30, false);
                                    dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);
                                    dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(
                                        detectionQueue,
                                        pnh,
                                        std::string("color/detections"),
                                        std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                                        30);

                                    detectionPublish.addPublisherCallback();
#endif
        ros::spin();

#if DETECTION_THREAD
        AbortDetectionTask();
        detection_task.join();
#endif
        return 0;

    }
    ros::spin();


    return 0;
}
