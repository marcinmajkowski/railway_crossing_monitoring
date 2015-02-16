#pragma once

#include <vector>
#include <string>
#include <cv.h>
#include "LevelCrossingEvent.h"
#include "RoadTrafficDetector.h"
#include "SafetyBarrierDetector.h"
#include "RailTrafficDetector.h"

class LevelCrossingMonitoring {
public:
    // constructor
    LevelCrossingMonitoring(std::string reference_image_path = "reference.png");

    // returns vector of events which occur in given image
    std::vector<LevelCrossingEvent> operator()(cv::Mat_<uchar> input);

    // check whether object is initialized properly
    bool is_ready() { return ready; }

private:
    // grayscale reference (empty background) image
    cv::Mat_<uchar> reference;

    // objects used to detect particular events

    RoadTrafficDetector road_traffic;
    SafetyBarrierDetector safety_barrier;
    RailTrafficDetector rail_traffic;

    // object is initialized properly
    bool ready;
};
