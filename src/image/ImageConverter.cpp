#include "ImageConverter.h"
#include <std_msgs/Header.h>

namespace Image {

    ImageConverter::ImageConverter() {}

    cv::Mat ImageConverter::convertMessageToCVImage(const sensor_msgs::ImageConstPtr &msg, std::string encoding) {
        // Creates an empty pointer for a cv_bridge CvImage
        cv_bridge::CvImagePtr cvPtr;

        try {
            // Tries to convert the message into a CvBridge Image with the specified encoding
            cvPtr = cv_bridge::toCvCopy(msg, encoding);
        } catch (cv_bridge::Exception &e) { // Image conversion failed
            ROS_ERROR("CV_BRIDGE EXCEPTION: thing");
        }

        return cvPtr->image; // Returns a cv::Mat image
    }

    cv_bridge::CvImage ImageConverter::convertCVImageToCVBridgeImage(cv::Mat image, std::string encoding) {
        // Creates an empty header message
        std_msgs::Header header;

        // Builds Header message with an abitrary frame id, message count, and send time
        header.seq = this->messageGenCount;
        header.frame_id = 0x01;
        header.stamp = ros::Time::now();

        // Increases the message count
        this->messageGenCount += 1;

        // Constructs CvImage using image and header message
        cv_bridge::CvImage cvImage(header, encoding, image);
        return cvImage;
    }

    sensor_msgs::Image ImageConverter::convertCVImageToMessage(cv::Mat image, std::string encoding) {
        // Gets CvImage and then converts it into the sensor_msgs::Image message
        return *this->convertCVImageToCVBridgeImage(image, encoding).toImageMsg();
    }
}