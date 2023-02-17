/**
	pipe.cpp
	Purpose: Implementation pip for OAK-D camera
	@author Gerard Harkema
	@version 0.9 2023/01/05
    License: CC BY-NC-SA
*/


#ifndef PIPE
#define PIPE
std::tuple<dai::Pipeline, int, int> createPipeline(bool enableNeuralNetworkDetection,
                                                   bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   bool rectify,
                                                   int stereo_fps,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   std::string stereoResolution,
                                                   std::string rgbResolutionStr,
                                                   int rgbScaleNumerator,
                                                   int rgbScaleDinominator,
                                                   int previewWidth,
                                                   int previewHeight,
                                                   std::string nnPath,
                                                   std::string nnConfigPath,
                                                   float confidenceThreshold,
                                                   bool yolo_nn_network);
#endif
