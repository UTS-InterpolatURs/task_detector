#include "ImageConverter.h"
#include <std_msgs/Header.h>

namespace Image {

    ImageConverter::ImageConverter() {}

    cv::Mat ImageConverter::convertMessageToCVImage(const sensor_msgs::ImageConstPtr &msg, std::string encoding) {
        cv_bridge::CvImagePtr cvPtr;

        try {
            cvPtr = cv_bridge::toCvCopy(msg, encoding);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("CV_BRIDGE EXCEPTION: thing");
        }

        return cvPtr->image;
    }

    cv_bridge::CvImage ImageConverter::convertCVImageToCVBridgeImage(cv::Mat image, std::string encoding) {
        std_msgs::Header header;
        header.seq = this->messageGenCount;
        header.frame_id = 0x01;
        header.stamp = ros::Time::now();

        this->messageGenCount += 1;

        cv_bridge::CvImage cvImage(header, encoding, image);
        return cvImage;
    }

    sensor_msgs::Image ImageConverter::convertCVImageToMessage(cv::Mat image, std::string encoding) {
        return *this->convertCVImageToCVBridgeImage(image, encoding).toImageMsg();
    }
}