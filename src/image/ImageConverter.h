#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace Image {
    class ImageConverter {
        public:
            // Empty Constructor for ImageConverter class
            ImageConverter();

            // Used to convert a sensor_msgs::Image message to a cv::Mat image
            cv::Mat convertMessageToCVImage(const sensor_msgs::ImageConstPtr &msg, std::string encoding = sensor_msgs::image_encodings::BGR8);

            // Converts cv::Mat to cv_bridge::CvImage
            cv_bridge::CvImage convertCVImageToCVBridgeImage(cv::Mat image, std::string encoding = sensor_msgs::image_encodings::BGR8);

            // Converts cv::Mat into a sensor_msgs::Image
            sensor_msgs::Image convertCVImageToMessage(cv::Mat image, std::string encoding = sensor_msgs::image_encodings::BGR8);

        private:
            int messageGenCount = 0;
    };
};

#endif