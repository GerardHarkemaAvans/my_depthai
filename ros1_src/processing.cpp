/**
	processing.cpp
	Purpose: ROS Implementation for OAK-D camera nn detection
	@author Gerard Harkema
	@version 0.9 2023/01/05
    license: CC BY-NC-SA
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
}PREDECTION;


bool AbortDetectionTaskFlag = false;

void AbortDetectionTask(){
  AbortDetectionTaskFlag = true;
}

std::vector<PREDECTION> filter_and_classificate_tensors(std::vector<std::vector<float>> tensors, int num_anchor_boxes, int number_of_classes, float confidence_threshold, float scale_x, float scale_y);
void publish_predections(std::vector<PREDECTION> predections, ros::Publisher dectection_publisher, int seq_number);
std::vector<PREDECTION> nonMaximumSuppressionSimple(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors);

void DetectionTask(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::string topic_name,
                    float scale_x, float scale_y,
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

    int number_of_classes = class_names.size();

    while(!AbortDetectionTaskFlag){

        auto daiDataPtr = daiMessageQueue->tryGet<dai::NNData>();

        if(daiDataPtr != nullptr) {

            if(display_info){

                  std::cout << "Layer names: ";
                  for(auto val : daiDataPtr->getAllLayerNames()) {
                      std::cout << val << ", ";
                  }
                  std::cout << std::endl;

                  auto rimestamp = daiDataPtr->getTimestamp();
                  auto devive_timestamp = daiDataPtr->getTimestampDevice();

                  std::cout << "NNData size: " << daiDataPtr->getData().size() << std::endl;
                  std::cout << "FP16 values: ";
                  for(auto val : daiDataPtr->getLayerFp16("output")) {
                      //std::cout << std::to_string(val) << "x ";
                  }
                  std::cout << std::endl;

                  //auto val = daiDataPtr->getLayerFp16("auto");
                  //std::cout << val << std::endl;
                  std::cout << "UINT8 values: ";
                  for(auto val : daiDataPtr->getLayerUInt8("uint8")) {
                      std::cout << std::to_string(val) << "x ";
                  }
                  std::cout << std::endl;
                  display_info = false;
            }



            auto in_nn_layer = daiDataPtr->getLayerFp16("output");
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
                                                          scale_x,
                                                          scale_y);


            std::vector<PREDECTION> suspressed_predections;

            suspressed_predections = nonMaximumSuppressionSimple(predections, overlap_threshold, box_neighbors);

            publish_predections(suspressed_predections, dectection_publisher, seq_number++);

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
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


        object_hypothesis.id = predection->class_number;
        object_hypothesis.score = predection->score;
        detection.results.push_back(object_hypothesis);
        detection_array.detections.push_back(detection);
        break;

    }

    dectection_publisher.publish(detection_array);

}

std::vector<PREDECTION> filter_and_classificate_tensors(std::vector<std::vector<float>> tensors, int num_anchor_boxes,int number_of_classes, float confidence_threshold, float scale_x, float scale_y){

  std::vector<PREDECTION> predections;
#if 0
    std::vector<std::vector<float>>::iterator tensor;
  //std::vector<PREDECTION>::iterator predection;

    for(tensor = tensors.begin(); tensor != tensors.end(); tensor++){
        PREDECTION predection;

        predection.box.x = (float &)tensor[0] * scale_x;
        predection.box.y = (float &)tensor[1] * scale_y;
        predection.box.width = (float &)tensor[2] * scale_x;
        predection.box.height = (float &)tensor[3] * scale_y;

        int class_number = 0;
        float max_conf = 0;
        for(int j = 0; j < number_of_classes; j++){
          if((float &)tensor[5+j] > max_conf){
            max_conf = (float &)tensor[5+j];
            class_number = j;
          }
        }
        predection.score = (float &)tensor[4];
        predection.class_number = (float)class_number;

        predections.push_back(predection);
    }
#else
  for(int i = 0; i < num_anchor_boxes; i++){
    if(tensors[i][4] >= confidence_threshold){

      PREDECTION predection;

      predection.box.x = tensors[i][0] * scale_x;
      predection.box.y = tensors[i][1] * scale_y;
      predection.box.width = tensors[i][2] * scale_x;
      predection.box.height = tensors[i][3] * scale_y;

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
  #endif
  return predections;
}

std::vector<PREDECTION> nonMaximumSuppressionSimple(std::vector<PREDECTION>input_predections, float overlap_threshold, int box_neighbors){


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
