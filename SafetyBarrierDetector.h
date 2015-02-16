#pragma once

#include <cv.h>

class SafetyBarrierDetector {
public:
    // default constructor with default values
    SafetyBarrierDetector(
        std::string barrier_mask_path = "masks/barrier.png",
        std::string near_barrier_mask_path = "masks/barrier_negative.png",
        int canny_first_threshold = 460,
        int cannt_second_threshold = 70,
        double min_barrier_coverage = 8.0,
        double max_near_barrier_coverage = 1.5);

    // returns true if barrier in the given image is deployed
    bool operator()(cv::Mat_<uchar> input);

    // check whether object is initialized properly
    bool is_ready() { return ready; }

private:
    // mask used to show only barrier
    cv::Mat_<uchar> barrier_mask;

    // mask used to show region in which barrier edges shouldn not occur
    cv::Mat_<uchar> near_barrier_mask;

    // number of non zero pixels in barrier_mask
    int barrier_pixels;

    // number of non zero pixels in near_barrier_mask
    int near_barrier_pixels;

    // first threshold for the hysteresis procedure
    int canny_first_threshold;

    // second threshold for the hysteresis procedure
    int canny_second_threshold;

    // minimum percentage of edge pixels in barrier region
    double min_barrier_coverage;

    // maximum percentage of edge pixels in near barrier region
    double max_near_barrier_coverage;

    // object is initialized properly
    bool ready;
};
