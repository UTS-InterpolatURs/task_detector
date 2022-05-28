#include "CoordinateSmoother.h"

namespace Utils {
    CoordinateSmoother::CoordinateSmoother() {}

    geometry_msgs::Point32 CoordinateSmoother::getSmoothedGlobalCoordinate(std::vector<Spatial::Coordinate> points) {
        geometry_msgs::Point32 averagedPoint;

        float xSum = 0, ySum = 0, zSum = 0;

        for(auto coord : points) {
            geometry_msgs::Point32 global = coord.getGlobalCoordinates();
            xSum += global.x;
            ySum += global.y;
            zSum += global.z;
        }

        averagedPoint.x = xSum / points.size();
        averagedPoint.y = ySum / points.size();
        averagedPoint.z = zSum / points.size();
        
        return averagedPoint;        
    }
}