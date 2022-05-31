#include "Coordinate.h"

namespace Spatial {
    Coordinate::Coordinate() {}

    // Creates coordinate with values
    Coordinate::Coordinate(int imageX, int imageY, int z, sensor_msgs::CameraInfo info) {
        this->imageX = imageX;
        this->imageY = imageY;
        this->z = z;
		this->info = info;
    }

    // Gets image coordinates
    cv::Point Coordinate::getImageCoordinates() {
        return cv::Point(imageX, imageY);
    }

    geometry_msgs::Point32 Coordinate::getGlobalCoordinates() {
        // Creates empty Point32 message
        geometry_msgs::Point32 point;

        // Performs conversion on X, Y, and Z using the camera extrinsics and returns coordinates (in meters)
		point.x = (z * (imageX - info.K[2]) / info.K[0]) / 1000;
		point.y = (z * (imageY - info.K[5]) / info.K[4]) / 1000;
		point.z = static_cast<float>(z) / 1000; // Casts the depth value to a float and then converts to meters

        return point;
    }
}