#ifndef PROCESSING
#define PROCESSING




void DetectionTask(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::string topic_name,
                    float scale_x, float scale_y,
                    std::vector<std::string> class_names,
                    float confidence_threshold,
                    float overlap_threshold,
                    int box_neighbors);

void AbortDetectionTask();

#endif
