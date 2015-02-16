#pragma once

#include <vector>
#include <cv.h>
#include "LevelCrossingEvent.h"

class RoadTrafficDetector
{
public:
    RoadTrafficDetector(
        cv::Mat_<uchar> &empty,
        std::string road_mask_path = "masks/road.png",
        std::string zone_a_mask_path = "masks/zone_a.png",
        std::string zone_b_top_mask_path = "masks/zone_b_top.png",
        std::string zone_b_bottom_mask_path = "masks/zone_b_bottom.png",
        std::string zone_c_top_mask_path = "masks/zone_c_top.png",
        std::string zone_c_bottom_mask_path = "masks/zone_c_bottom.png",
        std::vector<double> line_between_lanes = std::vector<double>{0.5, -2.5},
        double min_object_area = 5000,
        int erode_size = 14,
        int dilate_size = 56,
        int threshold_value = 65);
    std::vector<LevelCrossingEvent> operator()(cv::Mat_<uchar> input);
    bool is_ready() { return ready; }

private:
    cv::Mat_<uchar> reference;
    cv::Mat_<uchar> road_mask;
    cv::Mat_<uchar> zone_a_mask;
    cv::Mat_<uchar> zone_b_top_mask;
    cv::Mat_<uchar> zone_b_bottom_mask;
    cv::Mat_<uchar> zone_c_top_mask;
    cv::Mat_<uchar> zone_c_bottom_mask;
    std::vector<double> line_between_lanes;
    double min_object_area;
    int erode_size;
    int dilate_size;
    int threshold_value;
    bool ready;
};

bool intersects(const std::vector<cv::Point> &contour, const cv::Mat_<uchar> &mask);
