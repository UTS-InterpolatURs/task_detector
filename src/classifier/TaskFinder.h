#ifndef TASK_FINDER_H
#define TASK_FINDER_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

#include "geometry_msgs/Point32.h"

#include "Classifier.h"
#include "../coordinate/Coordinate.h"

#include <string>
#include <vector>

namespace Classification {

    // Used to represent a found feature
    struct Feature {
        Spatial::Coordinate centerPoint;
        cv::Rect boundingRect;
    };

    // Used to represent all the possible detected features on the Taskboard
    struct Taskboard {
        Feature keyHole;
        Feature coinCell;
        Feature buttons;
        Feature ethernet; // Empty
        Feature key;
        Feature batterBox;
        Feature batteryHoles;
    };

    class TaskFinder {
        public:
            TaskFinder();
            TaskFinder(ros::NodeHandle n);
            
            // Finds all the known features within the image using both a standard color image and depth aligned image
            Taskboard findFeatures(cv::Mat image, cv::Mat depthImage, const sensor_msgs::CameraInfoConstPtr info);

        private:
            // Stored classifiers
            Classifier keyHoleClassifier;
            Classifier coinCellClassifier;
            Classifier buttonsClassifier;
            Classifier ethernetClassifier;
            Classifier batteryHoles;

            // These functions process each detection to remove noise as each classifier has their own known noise issue that must be addressed seperately
            Feature processKeyhole(std::vector<cv::Rect> detections, cv::Mat depthImage, sensor_msgs::CameraInfo info);
            Feature processCoinCell(std::vector<cv::Rect> detections, cv::Mat depthImage, sensor_msgs::CameraInfo info);
            Feature processButtons(std::vector<cv::Rect> detections);
            Feature processEthernet(std::vector<cv::Rect> detections);
            Feature processBatteryHoles(std::vector<cv::Rect> detections);
    };
};

#endif