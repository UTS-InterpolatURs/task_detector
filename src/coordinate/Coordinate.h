#ifndef COORDINATE_H
#define COORDINATE_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "geometry_msgs/Point32.h"

namespace Spatial {
    class Coordinate {
        public:
            Coordinate();
            Coordinate(int imageX, int imageY, int z, const sensor_msgs::CameraInfo info);

            geometry_msgs::Point32 getGlobalCoordinates();
            cv::Point getImageCoordinates();
        private:
            int imageX, imageY, z;
            sensor_msgs::CameraInfo info;
    };
};

#endif