#ifndef COORDINATE_SMOOTHER_H
#define COORDINATE_SMOOTHER_H

#include "../coordinate/Coordinate.h"

#include <geometry_msgs/Point32.h>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

namespace Utils {
    class CoordinateSmoother {
        public:
            CoordinateSmoother();

            geometry_msgs::Point32 getSmoothedGlobalCoordinate(std::vector<Spatial::Coordinate> points);
    };
};

#endif