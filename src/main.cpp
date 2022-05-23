#include "ros/ros.h"

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <vector>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>

#include "image/ImageConverter.h"

cv::CascadeClassifier classifier;
Image::ImageConverter converter;
image_transport::Publisher imagePub;

void imageCallback(const sensor_msgs::ImageConstPtr &message) {
    cv::Mat image = converter.convertMessageToCVImage(message, message->encoding);

    std::vector<cv::Rect> detections;
    classifier.detectMultiScale(image, detections);

    for(size_t i = 0; i < detections.size(); i++) {
        // cv::Point center(detections[i].x + detections[i].width)
        cv::rectangle(image, detections.at(i), cv::Scalar(255,255,255));
    }

    sensor_msgs::Image output = converter.convertCVImageToMessage(image);
    imagePub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "edge_detector");
    ros::NodeHandle n;

    if(!classifier.load("/home/jon/development/haar/filled_ethernet/data/cascade.xml")) {
        ROS_ERROR("Error loading classifier");
        return -1;
    }

    image_transport::ImageTransport transport(n);
    image_transport::Subscriber imageSub = transport.subscribe("/camera/color/image_raw", 1, &imageCallback);
    imagePub = transport.advertise("task_detect/output_video", 10);


    ros::spin();
}