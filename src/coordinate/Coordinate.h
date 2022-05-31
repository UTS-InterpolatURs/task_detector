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
            // Defines an empty coordinate constructor
            Coordinate();

            // Defines a coordinate constructor that will be build out the coordinate class
            Coordinate(int imageX, int imageY, int z, const sensor_msgs::CameraInfo info);

            // Converts the image coordinates into coordinates that can be used by the robot manipulator code (i.e. camera frame coordinates)
            geometry_msgs::Point32 getGlobalCoordinates();
            
            // Gets the image coordinates
            cv::Point getImageCoordinates();
        private:
            // Stores image coordinates and depth
            int imageX, imageY, z;

            // Stores camera info at the time the coordinate was created (used for conversion)
            sensor_msgs::CameraInfo info;
    };
};

#endif