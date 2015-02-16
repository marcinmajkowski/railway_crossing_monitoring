#include "LevelCrossingMonitoring.h"

#include <highgui.h>

using namespace std;
using namespace cv;

LevelCrossingMonitoring::LevelCrossingMonitoring(string reference_image_path)
    : reference(imread(reference_image_path, CV_LOAD_IMAGE_GRAYSCALE))
    , road_traffic(reference)
    , rail_traffic(reference)
    , ready(false)
{
    if (!reference.data) {
        cout << "Failed to load " << reference_image_path << "\n";
        return;
    }

    if (!road_traffic.is_ready()) {
        cout << "Failed to initialize road traffic object.\n";
        return;
    }

    if (!safety_barrier.is_ready()) {
        cout << "Failed to initialize safety barrier object.\n";
        return;
    }

    if (!rail_traffic.is_ready()) {
        cout << "Failed to initialize rail traffic object.\n";
        return;
    }

    ready = true;
}

vector<LevelCrossingEvent> LevelCrossingMonitoring::operator()(Mat_<uchar> input)
{
    vector<LevelCrossingEvent> events;

    // level crossing monitoring object is not properly initialized
    if (!ready)
        return events;

    if (rail_traffic(input)) { // there is a train
        events.push_back(LevelCrossingEvent::TRAIN);
    } else { // use road_traffic detector only if there is no train
        events = road_traffic(input);
    }

    if (safety_barrier(input)) { // safety barrier is deployed
        events.push_back(LevelCrossingEvent::BARRIER);
    }

    return events;
}
