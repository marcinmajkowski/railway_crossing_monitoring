#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include "LevelCrossingMonitoring.h"

using namespace std;
using namespace cv;

template <class Container, class T>
bool contains(const Container &c, const T &val)
{
    return find(begin(c), end(c), val) != end(c);
}

int main(int argc, char *argv[])
{
    // this will return a vector of events which occur in the given picture
    LevelCrossingMonitoring monitoring;
    if (!monitoring.is_ready()) {
        cout << "Failed to initialize monitoring object.\n";
        return -1;
    }

    // paths to all images
    vector<string> all_paths(&argv[1], &argv[argc]);

    // main loop testing all images
    for (const auto &path : all_paths) {
        // current loaded image
        Mat_<uchar> image = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
        if (!image.data) {
            cout << "Failed to load " << path << "\n";
            return -1;
        }

        // vector of events occured in the current picture
        vector<LevelCrossingEvent> events = monitoring(image);

        // screen output
        cout << path << " : ";

        if (contains(events, LevelCrossingEvent::ONTRACK))
            cout << "event 1 ";

        if (contains(events, LevelCrossingEvent::ENTERING))
            cout << "event 2 ";

        if (contains(events, LevelCrossingEvent::LEAVING))
            cout << "event 3 ";

        if (contains(events, LevelCrossingEvent::BARRIER))
            cout << "event 4 ";

        if (contains(events, LevelCrossingEvent::TRAIN))
            cout << "event 5 ";

        cout << '\n';
    }

    return 0;
}