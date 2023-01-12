/**
	processing.cpp
	Purpose: ROS Implementation for OAK-D camera nn detection
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
*/
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_ros_msgs/SpatialDetectionArray.h>
#include <depthai_ros_msgs/SpatialDetection.h>
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
//#include <depthai_bridge/SpatialDetectionConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

#include "depthai/depthai.hpp"

#include "jsoncpp/json/json.h"

#include <iostream>
#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include "processing.h"


typedef struct{
  cv::Rect box;
  float score;
  int class_number;
  float x;
  float y;
  float z;
}PREDECTION;


bool AbortDetectionTaskFlag = false;

void AbortDetectionTask(){
  AbortDetectionTaskFlag = true;
}

std::vector<PREDECTION> filter_and_classificate_tensors(std::vector<std::vector<float>> tensors,
                                                        int num_anchor_boxes,
                                                        int number_of_classes,
                                                        float confidence_threshold,
                                                        int output_image_width, int output_image_height,
                                                        int nn_image_width, int nn_image_height);
void publish_predections(std::vector<PREDECTION> predections, ros::Publisher dectection_publisher, int seq_number);
std::vector<PREDECTION> nonMaximumSuppressionSimple(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors);
//std::vector<PREDECTION> nonMaximumSuppressionSimpleV2(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors);
void configSpatialCalculations(std::shared_ptr<dai::Device> device,
                               std::string spatialCalcConfigInQueueName,
                               std::vector<PREDECTION> predections,
                               int output_image_width, int output_image_height);

void DetectionTask( std::shared_ptr<dai::Device> device,
                    std::string nnDataMessageQueueName,
                    std::string spatialCalcQueueName,
                    std::string spatialCalcConfigInQueueName,
                    std::string topic_name,
                    int output_image_width, int output_image_height,
                    int nn_image_width, int nn_image_height,
                    std::vector<std::string> class_names,
                    float confidence_threshold,
                    float overlap_threshold,
                    int box_neighbors)
{

    int cnt = 0;
    int messageCounter = 0;
    int seq_number = 0;
    bool display_info = false;

    ros::NodeHandle n;
    ros::Publisher dectection_publisher = n.advertise<depthai_ros_msgs::SpatialDetectionArray>(topic_name, 1000);

    auto spatialCalcQueue = device->getOutputQueue(spatialCalcQueueName, 8, false);
    auto spatialCalcConfigInQueue = device->getInputQueue(spatialCalcConfigInQueueName);
    auto nnDataMessageQueue =device->getOutputQueue(nnDataMessageQueueName, 30, false);


    int number_of_classes = class_names.size();

    while(!AbortDetectionTaskFlag){

        auto nnDataPtr = nnDataMessageQueue->tryGet<dai::NNData>();

        if(nnDataPtr != nullptr) {

            if(display_info){

                  std::cout << "Layer names: ";
                  for(auto val : nnDataPtr->getAllLayerNames()) {
                      std::cout << val << ", ";
                  }
                  std::cout << std::endl;

                  auto rimestamp = nnDataPtr->getTimestamp();
                  auto devive_timestamp = nnDataPtr->getTimestampDevice();

                  std::cout << "NNData size: " << nnDataPtr->getData().size() << std::endl;
                  std::cout << "FP16 values: ";
                  for(auto val : nnDataPtr->getLayerFp16("output")) {
                      //std::cout << std::to_string(val) << "x ";
                  }
                  std::cout << std::endl;

                  //auto val = nnDataPtr->getLayerFp16("auto");
                  //std::cout << val << std::endl;
                  std::cout << "UINT8 values: ";
                  for(auto val : nnDataPtr->getLayerUInt8("uint8")) {
                      std::cout << std::to_string(val) << "x ";
                  }
                  std::cout << std::endl;
                  display_info = false;
            }



            auto in_nn_layer = nnDataPtr->getLayerFp16("output");
            int num_anchor_boxes = in_nn_layer.size() / (number_of_classes + 5);


            int in_nn_layer_size = in_nn_layer.size();

            std::vector<std::vector<float>> tensors;
            tensors.resize(num_anchor_boxes);
            for (int i = 0; i < num_anchor_boxes; i++)
            {
                tensors[i].resize(number_of_classes + 5);
            }


            for (int i = 0; i < in_nn_layer_size; i++)
            {
                int row = i / (number_of_classes + 5);
                int col = i % (number_of_classes + 5);
                tensors[row][col] = in_nn_layer[i];
            }


            std::vector<PREDECTION> predections;

            predections = filter_and_classificate_tensors(tensors,
                                                          num_anchor_boxes,
                                                          number_of_classes,
                                                          confidence_threshold,
                                                          output_image_width, output_image_height,
                                                          nn_image_width, nn_image_height);



            std::vector<PREDECTION> suspressed_predections;
            suspressed_predections = nonMaximumSuppressionSimple(predections, overlap_threshold, box_neighbors);

            configSpatialCalculations(device,spatialCalcConfigInQueueName, suspressed_predections, output_image_width, output_image_height);

            //std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if(suspressed_predections.size()){
                std::vector<PREDECTION>::iterator predection;
                predection = suspressed_predections.begin();

                auto spatialData = spatialCalcQueue->get<dai::SpatialLocationCalculatorData>()->getSpatialLocations();
                if(spatialData.size()){
                    for(auto depthData : spatialData) {
                        predection->x = depthData.spatialCoordinates.x / 1000.0;
                        predection->y = depthData.spatialCoordinates.y / 1000.0;
                        predection->z = depthData.spatialCoordinates.z / 1000.0;

                        #if 0
                        std::cout << "X: " << (int)depthData.spatialCoordinates.x << " mm" << std::endl;
                        std::cout << "Y: " << (int)depthData.spatialCoordinates.y << " mm" << std::endl;
                        std::cout << "Z: " << (int)depthData.spatialCoordinates.z << " mm" << std::endl;
                        #endif
                        predection++;
                        if(predection == suspressed_predections.end()) break;
                    }

                }
            }

            publish_predections(suspressed_predections, dectection_publisher, seq_number++);

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

}

void configSpatialCalculations(std::shared_ptr<dai::Device> device,
                               std::string spatialCalcConfigInQueueName,
                               std::vector<PREDECTION> predections,
                               int output_image_width, int output_image_height){


    if(predections.size()){
        auto spatialCalcConfigInQueue = device->getInputQueue(spatialCalcConfigInQueueName);


        dai::SpatialLocationCalculatorConfig cfg;
        dai::SpatialLocationCalculatorConfigData config;
        config.depthThresholds.lowerThreshold = 100;
        config.depthThresholds.upperThreshold = 10000;

        float xmin;
        float ymin;
        float xmax;
        float ymax;

        for(auto predection : predections){
            xmin = (float)(predection.box.x-predection.box.width)/(float)output_image_width;
            ymin = (float)(predection.box.y-predection.box.height)/(float)output_image_height;
            xmax = (float)(predection.box.x+predection.box.width)/(float)output_image_width;
            ymax = (float)(predection.box.y+predection.box.height)/(float)output_image_height;
            dai::Point2f topLeft(xmin, ymin);
            dai::Point2f bottomRight(xmax, ymax);

            config.roi = dai::Rect(topLeft, bottomRight);
            config.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
            cfg.addROI(config);
        }
        spatialCalcConfigInQueue->send(cfg);
    }
}

void publish_predections(std::vector<PREDECTION> predections, ros::Publisher dectection_publisher, int seq_number){

    std::vector<PREDECTION>::iterator predection;

    depthai_ros_msgs::SpatialDetectionArray detection_array;
    detection_array.header.seq=seq_number;
    detection_array.header.stamp = ros::Time::now();

    depthai_ros_msgs::SpatialDetection detection;
    vision_msgs::ObjectHypothesis object_hypothesis;
    for(predection = predections.begin(); predection != predections.end(); predection++){
        detection.results.clear();

        detection.bbox.center.x = predection->box.x;
        detection.bbox.center.y = predection->box.y;
        detection.bbox.center.theta = 0;
        detection.bbox.size_x = predection->box.width;
        detection.bbox.size_y = predection->box.height;

        detection.position.x = predection->x;
        detection.position.y = predection->y;
        detection.position.z = predection->z;

        object_hypothesis.id = predection->class_number;
        object_hypothesis.score = predection->score;



        detection.results.push_back(object_hypothesis);
        detection_array.detections.push_back(detection);

    }

    dectection_publisher.publish(detection_array);

}

std::vector<PREDECTION> filter_and_classificate_tensors(std::vector<std::vector<float>> tensors,
                                                        int num_anchor_boxes,
                                                        int number_of_classes,
                                                        float confidence_threshold,
                                                        int output_image_width, int output_image_height,
                                                        int nn_image_width, int nn_image_height){
    std::vector<PREDECTION> predections;

    if((nn_image_width / nn_image_height) != 1){
        std::cout << "Unable to scale detections, due wrong aspect ration imput to neural network" << std::endl;
        return predections;
    }

    for(int i = 0; i < num_anchor_boxes; i++){
        if(tensors[i][4] >= confidence_threshold){

            PREDECTION predection;

            float scale = (float)output_image_height / (float)nn_image_height;
            float pad = (output_image_width - (nn_image_width * scale)) / 2;

            float box_x = tensors[i][0];
            float box_y = tensors[i][1];
            float box_width = tensors[i][2];
            float box_height = tensors[i][3];

            predection.box.x = (box_x * scale) + pad;
            predection.box.y = box_y * scale;
            predection.box.width = box_width * scale;
            predection.box.height = box_height * scale;


            int class_number = 0;
            float max_conf = 0;
            for(int j = 0; j < number_of_classes; j++){
                if(tensors[i][5+j] > max_conf){
                    max_conf = tensors[i][5+j];
                    class_number = j;
                }
            }
            predection.score = tensors[i][4];
            predection.class_number = (float)class_number;

            predections.push_back(predection);
        }
    }

    return predections;
}

std::vector<PREDECTION> nonMaximumSuppressionSimple(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors){

    //return input_predections;

    std::vector<PREDECTION>::iterator predection;
    std::vector<PREDECTION> predections;
    std::vector<cv::Rect> srcRects;
    std::vector<cv::Rect> resRects;


    for(predection = input_predections.begin(); predection != input_predections.end(); predection++){
        srcRects.push_back(predection->box);

    }


    const size_t size = srcRects.size();
    if (!size)
        return predections;

    // Sort the bounding boxes by the bottom - right y - coordinate of the bounding box
    std::multimap<int, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
    {
        idxs.emplace(srcRects[i].br().y, i);
    }

    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0)
    {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Rect& rect1 = srcRects[lastElem->second];
        PREDECTION predection = input_predections[lastElem->second];

        int neigborsCount = 0;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); )
        {
            // grab the current rectangle
            const cv::Rect& rect2 = srcRects[pos->second];

            float intArea = static_cast<float>((rect1 & rect2).area());
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > overlap_threshold)
            {
                pos = idxs.erase(pos);
                    ++neigborsCount;
            }
            else
            {
                ++pos;
            }
        }
        if (neigborsCount >= box_neighbors){
            resRects.push_back(rect1);
            predections.push_back(predection);
        }
    }

    return predections;

}
#if 0
std::vector<PREDECTION> nonMaximumSuppressionSimpleV2(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors){


    std::vector<PREDECTION>::iterator predection;
    std::vector<PREDECTION> predections;
    std::vector<cv::Rect> srcRects;
    std::vector<cv::Rect> resRects;


    for(predection = input_predections.begin(); predection != input_predections.end(); predection++){
        srcRects.push_back(predection->box);

    }
    resRects.clear();

    const size_t size = srcRects.size();
    if (!size)
        return;

    assert(srcRects.size() == scores.size());

    // Sort the bounding boxes by the detection score
    std::multimap<float, size_t> idxs;
    for (size_t i = 0; i < size; ++i)
    {
        idxs.emplace(scores[i], i);
    }

    // keep looping while some indexes still remain in the indexes list
    while (idxs.size() > 0)
    {
        // grab the last rectangle
        auto lastElem = --std::end(idxs);
        const cv::Rect& rect1 = srcRects[lastElem->second];

        int neigborsCount = 0;
        float scoresSum = lastElem->first;

        idxs.erase(lastElem);

        for (auto pos = std::begin(idxs); pos != std::end(idxs); )
        {
            // grab the current rectangle
            const cv::Rect& rect2 = srcRects[pos->second];

            float intArea = static_cast<float>((rect1 & rect2).area());
            float unionArea = rect1.area() + rect2.area() - intArea;
            float overlap = intArea / unionArea;

            // if there is sufficient overlap, suppress the current bounding box
            if (overlap > thresh)
            {
                scoresSum += pos->first;
                pos = idxs.erase(pos);
                ++neigborsCount;
            }
            else
            {
                ++pos;
            }
        }
        if (neigborsCount >= neighbors && scoresSum >= minScoresSum){
            resRects.push_back(rect1);
            predections.push_back(predection);

        }
    }
    return predections;

}
#endif
