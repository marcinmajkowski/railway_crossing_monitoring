#pragma once

#include <cv.h>

class RailTrafficDetector
{
public:
    RailTrafficDetector(
        cv::Mat_<uchar> &empty,
        std::string railway_mask_path = "masks/railway_track.png",
        double min_average = 46.0);
    bool operator()(cv::Mat_<uchar> input);
    bool is_ready() { return ready; }

private:
    cv::Mat_<uchar> reference;
    cv::Mat_<uchar> railway_mask;
    int railway_pixels;
    double min_average;
    bool ready;
};

