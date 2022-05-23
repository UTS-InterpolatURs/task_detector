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
            ImageConverter();
            // ~ImageConverter();

            cv::Mat convertMessageToCVImage(const sensor_msgs::ImageConstPtr &msg, std::string encoding = sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImage convertCVImageToCVBridgeImage(cv::Mat image, std::string encoding = sensor_msgs::image_encodings::BGR8);
            sensor_msgs::Image convertCVImageToMessage(cv::Mat image, std::string encoding = sensor_msgs::image_encodings::BGR8);

        private:
            int messageGenCount = 0;
    };
};

#endif