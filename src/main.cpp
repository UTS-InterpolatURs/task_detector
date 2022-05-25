#include "ros/ros.h"

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "image/ImageConverter.h"
#include "classifier/TaskFinder.h"

Image::ImageConverter converter;
image_transport::Publisher imagePub;

Classification::TaskFinder finder;

// void imageCallback(const sensor_msgs::ImageConstPtr &message) {
//     cv::Mat image = converter.convertMessageToCVImage(message, message->encoding);

//     std::vector<cv::Rect> detections;
//     classifier.detectMultiScale(image, detections);

//     for(size_t i = 0; i < detections.size(); i++) {
//         // cv::Point center(detections[i].x + detections[i].width)
//         cv::rectangle(image, detections.at(i), cv::Scalar(255,255,255));
//     }

//     // cv::Mat image = cv::imread("/home/jon/development/haar/empty_ethernet/info/out1.png");

//     // ROS_INFO_STREAM("Hello World" << image.empty());

//     // cv::Rect rect(cv::Point2i(1046, 363), cv::Size(79, 70));

//     // cv::rectangle(image, rect, cv::Scalar(255,0,0));

//     sensor_msgs::Image output = converter.convertCVImageToMessage(image, message->encoding);
//     imagePub.publish(output);
// }

void imageCallback(const sensor_msgs::ImageConstPtr depthMessage, const sensor_msgs::ImageConstPtr colorMessage, const sensor_msgs::CameraInfoConstPtr info) {

    cv::Mat colorImage = converter.convertMessageToCVImage(colorMessage, colorMessage->encoding);
    cv::Mat depthImage = converter.convertMessageToCVImage(depthMessage, depthMessage->encoding);

    Classification::Taskboard board = finder.findFeatures(colorImage, depthImage, info);

    Classification::Feature feature = board.keyHole;

    cv::rectangle(colorImage, feature.boundingRect, cv::Scalar(255,255,255));
    cv::Point centerPoint = feature.centerPoint.getImageCoordinates();
    cv::circle(colorImage, cv::Point(centerPoint.x, centerPoint.y), 10, cv::Scalar(255,0,255));

    geometry_msgs::Point32 globalPoint = feature.centerPoint.getGlobalCoordinates();

    ROS_INFO_STREAM("GX: " << globalPoint.x << " GY: " << globalPoint.y << " GZ: " << globalPoint.z);

    sensor_msgs::Image output = converter.convertCVImageToMessage(colorImage, colorMessage->encoding);
    imagePub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_detector");
    ros::NodeHandle n;

    image_transport::ImageTransport transport(n);
    // image_transport::Subscriber imageSub = transport.subscribe("/camera/color/image_raw", 1, &imageCallback);
    imagePub = transport.advertise("task_detect/output_video", 10);

    finder = Classification::TaskFinder(n);

    message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> imageSub(n, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub(n, "camera/color/camera_info", 1);

    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ImageSyncPolicy;
    message_filters::Synchronizer<ImageSyncPolicy> sync(ImageSyncPolicy(10), depthSub, imageSub, infoSub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::spin();
}