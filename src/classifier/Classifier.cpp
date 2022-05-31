#include "Classifier.h"

namespace Classification {
    Classifier::Classifier() {}
    
    // Creates and loads the classifier
    Classifier::Classifier(std::string path) {
        if(!classifier.load(path)) {
            ROS_ERROR("Faied to load classifier at %s", path); // Returns error if failed to load coordinate
        }
    }

    std::vector<cv::Rect> Classifier::classify(cv::Mat image) {
        // Creates empty array of detected rectangles
        std::vector<cv::Rect> detections;

        // Finds and stores all detected objects in the image
        classifier.detectMultiScale(image, detections);
        return detections;
    }
}