#include "Classifier.h"

namespace Classification {
    Classifier::Classifier() {}
    
    Classifier::Classifier(std::string path) {
        if(!classifier.load(path)) {
            ROS_ERROR("Faied to load classifier at %s", path);
        }
    }

    std::vector<cv::Rect> Classifier::classify(cv::Mat image) {
        std::vector<cv::Rect> detections;
        classifier.detectMultiScale(image, detections);
        return detections;
    }
}