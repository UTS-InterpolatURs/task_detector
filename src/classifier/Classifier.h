#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

#include "geometry_msgs/Point32.h"

#include <string>
#include <vector>

namespace Classification {
    class Classifier {
        public:
            // Empty constructor
            Classifier();

            // Constructor where the classifier model path is specified (i.e. cascade.xml)
            Classifier(std::string path);

            // Classifies the objects in the image for the current classifier (e.g. keyhole)
            std::vector<cv::Rect> classify(cv::Mat image);

        private:
            // Stores the cascade classifier object
            cv::CascadeClassifier classifier;

    };
};

#endif