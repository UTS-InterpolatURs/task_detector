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

#include "task_detector/GetFeature.h"

#include "image/ImageConverter.h"
#include "classifier/TaskFinder.h"

#define POINT_COUNT_GOAL 15

Image::ImageConverter converter;
image_transport::Publisher imagePub;

Classification::TaskFinder finder;

std::vector<Spatial::Coordinate> points;

void imageCallback(const sensor_msgs::ImageConstPtr depthMessage, const sensor_msgs::ImageConstPtr colorMessage, const sensor_msgs::CameraInfoConstPtr info) {

    if(points.size() > POINT_COUNT_GOAL) {
        ROS_INFO("hello world");
        points.clear();
    }

    cv::Mat colorImage = converter.convertMessageToCVImage(colorMessage, colorMessage->encoding);
    cv::Mat depthImage = converter.convertMessageToCVImage(depthMessage, depthMessage->encoding);

    Classification::Taskboard board = finder.findFeatures(colorImage, depthImage, info);

    Classification::Feature feature = board.keyHole;

    cv::rectangle(colorImage, feature.boundingRect, cv::Scalar(255,255,255));
    cv::Point centerPoint = feature.centerPoint.getImageCoordinates();
    cv::circle(colorImage, cv::Point(centerPoint.x, centerPoint.y), 10, cv::Scalar(255,0,255));

    Spatial::Coordinate coord = feature.centerPoint;
    points.push_back(coord);

    geometry_msgs::Point32 globalPoint = coord.getGlobalCoordinates();

    ROS_INFO_STREAM("GX: " << globalPoint.x << " GY: " << globalPoint.y << " GZ: " << globalPoint.z);

    // sensor_msgs::Image output = converter.convertCVImageToMessage(colorImage, colorMessage->encoding);
    // imagePub.publish(output);
}

bool getFeature(task_detector::GetFeature::Request &request, task_detector::GetFeature::Response &response) {
    while(points.size() < POINT_COUNT_GOAL-1) {
        ROS_INFO("Waiting for desired number of images");
    }

    ROS_INFO("Reached goal");

    geometry_msgs::Point32 averagedPoint;

    float xSum = 0, ySum = 0, zSum = 0;

    for(auto coord : points) {
        geometry_msgs::Point32 global = coord.getGlobalCoordinates();
        xSum += global.x;
        ySum += global.y;
        zSum += global.z;
    }

    averagedPoint.x = xSum / points.size();
    averagedPoint.y = ySum / points.size();
    averagedPoint.z = zSum / points.size();

    std::vector<geometry_msgs::Point32> featurePoints;
    featurePoints.push_back(averagedPoint);

    response.points = featurePoints;
    return true;
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

    ros::ServiceServer service = n.advertiseService("get_features", getFeature);

    // ros::spin();8888

    ros::AsyncSpinner asyncSpinner(8);
    asyncSpinner.start();

    ros::waitForShutdown();
}