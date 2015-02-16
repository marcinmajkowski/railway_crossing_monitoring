#pragma once

#include <cv.h>

class WaitingOnTrainDetector
{
public:
    WaitingOnTrainDetector(
        std::string waiting_mask_path = "masks/waiting.png",
        int canny_first_threshold = 460,
        int canny_secong_threshold = 70,
        double min_coverage = 8.0);

    // returns true if cars are waiting on train
    bool operator()(cv::Mat_<uchar> input);

    // check whether object is initialized properly
    bool is_ready() { return ready; }

private:
    // mask used to show only waiting zone
    cv::Mat_<uchar> waiting_mask;

    // number of non zero pixels in waiting_mask
    int waiting_pixels;

    // first threshold for the hysteresis procedure
    int canny_first_threshold;

    // second threshold for the hysteresis procedure
    int canny_second_threshold;

    // minimum percentage of edge pixels in waiting region
    double min_coverage;

    // object is initialized properly
    bool ready;
};

