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
            Classifier();
            Classifier(std::string path);

            std::vector<cv::Rect> classify(cv::Mat image);

        private:
            cv::CascadeClassifier classifier;

    };
};

#endif