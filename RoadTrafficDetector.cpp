#include "RoadTrafficDetector.h"

#include <vector>
#include <highgui.h>

using namespace std;
using namespace cv;

RoadTrafficDetector::RoadTrafficDetector(
    cv::Mat_<uchar> &empty,
    std::string road_mask_path,
    std::string zone_a_mask_path,
    std::string zone_b_top_mask_path,
    std::string zone_b_bottom_mask_path,
    std::string zone_c_top_mask_path,
    std::string zone_c_bottom_mask_path,
    std::vector<double> line_between_lanes,
    double min_object_area,
    int erode_size,
    int dilate_size,
    int threshold_value)
    : road_mask(imread(road_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , zone_a_mask(imread(zone_a_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , zone_b_top_mask(imread(zone_b_top_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , zone_b_bottom_mask(imread(zone_b_bottom_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , zone_c_top_mask(imread(zone_c_top_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , zone_c_bottom_mask(imread(zone_c_bottom_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , line_between_lanes(line_between_lanes)
    , erode_size(erode_size)
    , dilate_size(dilate_size)
    , threshold_value(threshold_value)
    , ready(false)
{
    equalizeHist(empty, reference);
    if (!reference.data) {
        cout << "Failed to load road traffic reference image\n";
        return;
    }

    if (!road_mask.data) {
        cout << "Failed to load " << road_mask_path << "\n";
        return;
    }

    if (!zone_a_mask.data) {
        cout << "Failed to load " << zone_a_mask_path << "\n";
        return;
    }

    if (!zone_b_top_mask.data) {
        cout << "Failed to load " << zone_b_top_mask_path << "\n";
        return;
    }

    if (!zone_b_bottom_mask.data) {
        cout << "Failed to load " << zone_b_bottom_mask_path << "\n";
        return;
    }

    if (!zone_c_top_mask.data) {
        cout << "Failed to load " << zone_c_top_mask_path << "\n";
        return;
    }

    if (!zone_c_bottom_mask.data) {
        cout << "Failed to load " << zone_c_bottom_mask_path << "\n";
        return;
    }

    ready = true;
}

vector<LevelCrossingEvent> RoadTrafficDetector::operator()(cv::Mat_<uchar> input)
{
    // vector of events occuring in the given image
    vector<LevelCrossingEvent> events;

    // processed image
    Mat_<uchar> image;

    // applying histogram equalization on current input image
    equalizeHist(input, image);

    // difference image between current and reference images
    absdiff(image, reference, image);

    // applying mask to obtained difference image
    bitwise_and(image, road_mask, image);

    // morphological opening
    erode(image, image, getStructuringElement(MORPH_ELLIPSE, Size(erode_size, erode_size)));
    dilate(image, image, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size, dilate_size)));

    // thresholding
    threshold(image, image, threshold_value, 255, CV_THRESH_BINARY);

    // contours of detected objects
    vector<vector<Point>> road_traffic_contours;
    findContours(image, road_traffic_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // centroids of detected objects
    vector<Point> road_traffic_centroids;
    for (const auto &contour : road_traffic_contours) {
        Moments m = moments(contour);
        Point centroid(m.m10 / m.m00, m.m01 / m.m00);
        road_traffic_centroids.push_back(centroid);
    }

    // separating contours between these with centroid on the top and the bottom lane
    // adequate area is needed as well
    vector<vector<Point>> road_traffic_contours_top;
    vector<vector<Point>> road_traffic_contours_bottom;
    vector<vector<Point>> road_traffic_contours_small;
    for (int i = 0; i < road_traffic_contours.size(); ++i) {
        if (contourArea(road_traffic_contours[i]) >  min_object_area) {
            if (road_traffic_centroids[i].y < road_traffic_centroids[i].x * line_between_lanes[0] + line_between_lanes[1])
                road_traffic_contours_top.push_back(road_traffic_contours[i]);
            else
                road_traffic_contours_bottom.push_back(road_traffic_contours[i]);
        } else {
            road_traffic_contours_small.push_back(road_traffic_contours[i]);
        }
    }

    // checking events on top lane
    for (const auto &contour : road_traffic_contours_top) {
        if (intersects(contour, zone_a_mask))
            events.push_back(LevelCrossingEvent::ONTRACK);

        if (intersects(contour, zone_b_top_mask))
            events.push_back(LevelCrossingEvent::ENTERING);

        if (intersects(contour, zone_c_top_mask))
            events.push_back(LevelCrossingEvent::LEAVING);
    }
    
    // checking events on bottom lane
    for (const auto &contour : road_traffic_contours_bottom) {
        if (intersects(contour, zone_a_mask))
            events.push_back(LevelCrossingEvent::ONTRACK);

        if (intersects(contour, zone_b_bottom_mask))
            events.push_back(LevelCrossingEvent::ENTERING);

        if (intersects(contour, zone_c_bottom_mask))
            events.push_back(LevelCrossingEvent::LEAVING);
    }

    return events;
}

bool intersects(const vector<Point> &contour, const Mat_<uchar> &mask)
{
    Mat_<uchar> c = Mat_<uchar>::zeros(mask.size());
    fillConvexPoly(c, contour.data(), contour.size(), 255);

    /*
    Below is:

    bitwise_and(mask, c, c);
    return countNonZero(c);

    optimized.
    */

    auto it_m = mask.begin();
    auto it_c = c.begin();
    while (it_m != mask.end()) {
        if (*it_m && *it_c) return true;
        ++it_m, ++it_c;
    }

    return false;
}
