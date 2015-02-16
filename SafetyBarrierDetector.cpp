#include "SafetyBarrierDetector.h"

#include <highgui.h>

using namespace cv;
using namespace std;

SafetyBarrierDetector::SafetyBarrierDetector(
    string barrier_mask_path, string near_barrier_mask_path,
    int canny_first_threshold, int canny_second_threshold,
    double min_barrier_coverage, double max_near_barrier_coverage)
    : barrier_mask(imread(barrier_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , near_barrier_mask(imread(near_barrier_mask_path, CV_LOAD_IMAGE_GRAYSCALE))
    , barrier_pixels(countNonZero(barrier_mask))
    , near_barrier_pixels(countNonZero(near_barrier_mask))
    , canny_first_threshold(canny_first_threshold)
    , canny_second_threshold(canny_second_threshold)
    , min_barrier_coverage(min_barrier_coverage)
    , max_near_barrier_coverage(max_near_barrier_coverage)
    , ready(false)
{
    if (!barrier_mask.data) {
        cout << "Failed to load " << barrier_mask_path << "\n";
        return;
    }

    if (!near_barrier_mask.data) {
        cout << "Failed to load " << near_barrier_mask_path << "\n";
        return;
    }

    ready = true;
}

bool SafetyBarrierDetector::operator()(Mat_<uchar> input)
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
