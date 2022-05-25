#include "Coordinate.h"

namespace Spatial {
    Coordinate::Coordinate() {}

    Coordinate::Coordinate(int imageX, int imageY, int z, sensor_msgs::CameraInfo info) {
        this->imageX = imageX;
        this->imageY = imageY;
        this->z = z;
		this->info = info;
    }

    cv::Point Coordinate::getImageCoordinates() {
        return cv::Point(imageX, imageY);
    }

    geometry_msgs::Point32 Coordinate::getGlobalCoordinates() {
        geometry_msgs::Point32 point;

		point.x = (z * (imageX - info.K[2]) / info.K[0]) / 1000;
		point.y = (z * (imageY - info.K[5]) / info.K[4]) / 1000;
		point.z = z / 1000;

        return point;
    }
}