#include "RailTrafficDetector.h"

#include <iostream>
#include <numeric>
#include <highgui.h>

using namespace std;
using namespace cv;

RailTrafficDetector::RailTrafficDetector(Mat_<uchar> &empty, string railway_mask_path, double min_average)
    : railway_mask(imread(railway_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , railway_pixels(countNonZero(railway_mask))
    , min_average(min_average)
    , ready(false)
{
    equalizeHist(empty, reference);
    if (!reference.data) {
        cout << "Failed to load rail traffic reference image\n";
        return;
    }

    if (!railway_mask.data) {
        cout << "Failed to load " << railway_mask_path << "\n";
        return;
    }

    ready = true;
}

bool RailTrafficDetector::operator()(cv::Mat_<uchar> input)
{
    // processed image
    Mat_<uchar> image;

    // applying histogram equalization on current input image
    equalizeHist(input, image);

    // difference image between current and reference images
    absdiff(image, reference, image);

    // applying mask to obtained difference
    bitwise_and(image, railway_mask, image);

    // calculating average gray value in difference masked region
    double average =
        double(std::accumulate(image.begin(), image.end(), 0))
        / railway_pixels;

    // if average is higher than minimum, train is detected
    return average > min_average;
}
