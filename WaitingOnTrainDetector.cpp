#include "WaitingOnTrainDetector.h"

#include <highgui.h>

using namespace cv;
using namespace std;

WaitingOnTrainDetector::WaitingOnTrainDetector(
        string waiting_mask_path,
        int canny_first_threshold,
        int canny_secong_threshold,
        double min_coverage)
        : waiting_mask(imread(waiting_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
        , canny_first_threshold(canny_first_threshold)
        , canny_second_threshold(canny_second_threshold)
        , min_coverage(min_coverage)
        , ready(false)
{
    if (!waiting_mask.data) {
        cout << "Failed to load " << waiting_mask_path << "\n";
        return;
    }

    ready = true;
}

bool WaitingOnTrainDetector::operator()(Mat_<uchar> input)
{
    // edge image of current input image
    Mat_<uchar> edges;
    Canny(input, edges, canny_first_threshold, canny_second_threshold);

    // image with mask applied
    Mat_<uchar> masked;

    // calculating coverage with edges for masked regions
    bitwise_and(edges, barrier_mask, masked);
    double barrier_coverage = 
        (double)countNonZero(masked) / barrier_pixels * 100;
    bitwise_and(edges, near_barrier_mask, masked);
    double near_barrier_coverage = 
        (double)countNonZero(masked) / near_barrier_pixels * 100;
    
    // returns true when barrier is deployed
    return barrier_coverage > min_barrier_coverage && 
        near_barrier_coverage < max_near_barrier_coverage;
}
