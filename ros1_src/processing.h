/**
	processing.h
	Purpose: ROS Implementation for OAK-D camera nn detection
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
*/

#ifndef PROCESSING
#define PROCESSING

void DetectionTask(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::string topic_name,
                    int output_image_width, int output_image_height,
                    int nn_image_width, int nn_image_height,
                    std::vector<std::string> class_names,
                    float confidence_threshold,
                    float overlap_threshold,
                    int box_neighbors);

void AbortDetectionTask();

#endif
