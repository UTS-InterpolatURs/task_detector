#include "TaskFinder.h"

namespace Classification {
    TaskFinder::TaskFinder() {}

    // Constructs TaskFinder and creates classifiers
    TaskFinder::TaskFinder(ros::NodeHandle n) {
        keyHoleClassifier = Classifier("/home/jon/development/haar/keyhole/data/cascade.xml");
        coinCellClassifier = Classifier("/home/jon/development/haar/coincell/data/cascade.xml");
        buttonsClassifier = Classifier("/home/jon/development/haar/buttons/data/cascade.xml");
        ethernetClassifier = Classifier("/home/jon/development/haar/empty_ethernet/data/cascade.xml");
    }

    Taskboard TaskFinder::findFeatures(cv::Mat image, cv::Mat depthImage, const sensor_msgs::CameraInfoConstPtr info) {
        // Creates empty taskboard
        Taskboard board;

        // Gets all detected keyholes in the taskboard
        std::vector<cv::Rect> keyHoles = keyHoleClassifier.classify(image);
        // Processes, filters, and stores keyhole detection results
        board.keyHole = processKeyhole(keyHoles, depthImage, *info);

        // std::vector<cv::Rect> coinCells = coinCellClassifier.classify(image);
        // board.coinCell = processCoinCell(coinCells, depthImage);

        // std::vector<cv::Rect> buttons = buttonsClassifier.classify(image);
        // board.buttons = processButtons(buttons);

        return board;
    }

    Feature TaskFinder::processKeyhole(std::vector<cv::Rect> detections, cv::Mat depthImage, sensor_msgs::CameraInfo info) {
        // Checks to ensure there is a detected keyhole
        if(detections.size() == 0) return Feature();
        // Gets first keyhole (model is accurate and reliable enough that only one will ever be found)
        cv::Rect keyHole = detections.at(0);
        
        // Gets center point of the detected feature
        cv::Point point = (keyHole.br() + keyHole.tl()) * 0.5;

        // Gets the lowest point in the image (assumed to be the top leftmost pixel)
        ushort lowest = depthImage.at<ushort>(0,0);

        // Gets the depth at the specified center point
        ushort pixelDepth = depthImage.at<ushort>(point.y, point.x);

        // Creates coordinate representation
        Spatial::Coordinate coord(point.x, point.y, pixelDepth, info);

        // Constructs feature
        Feature feature;
        feature.centerPoint = coord;
        feature.boundingRect = keyHole;
        
        return feature;
    }

    // Does the same as above just with the coin cell feature
    Feature TaskFinder::processCoinCell(std::vector<cv::Rect> detections, cv::Mat depthImage, sensor_msgs::CameraInfo info) {
        if(detections.size() == 0) return Feature();
        cv::Rect coinCell = detections.at(0);

        cv::Point point = (coinCell.br() + coinCell.tl()) * 0.5;
        
        ushort lowest = depthImage.at<ushort>(0,0);
        ushort pixelDepth = depthImage.at<ushort>(point.y, point.x);

        Spatial::Coordinate coord(point.x, point.y, lowest-pixelDepth, info);

        Feature feature;
        feature.centerPoint = coord;
        feature.boundingRect = coinCell;

        return feature;
    }

}