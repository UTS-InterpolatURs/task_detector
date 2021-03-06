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
#include "utils/CoordinateSmoother.h"

#define POINT_COUNT_GOAL 15

Image::ImageConverter converter;
image_transport::Publisher imagePub;

Classification::TaskFinder finder;

std::vector<Spatial::Coordinate> points;

ros::Publisher pointsPub;

void imageCallback(const sensor_msgs::ImageConstPtr depthMessage, const sensor_msgs::ImageConstPtr colorMessage, const sensor_msgs::CameraInfoConstPtr info) {
    // Gets color and depth images as cv::Mat image type
    cv::Mat colorImage = converter.convertMessageToCVImage(colorMessage, colorMessage->encoding);
    cv::Mat depthImage = converter.convertMessageToCVImage(depthMessage, depthMessage->encoding);

    // Gets all recognised features on taskboard
    Classification::Taskboard board = finder.findFeatures(colorImage, depthImage, info);

    // Gets keyhole feature
    Classification::Feature feature = board.keyHole;

    // Draws rectangle around the recognised features and draws circle around the center point of the feature
    cv::rectangle(colorImage, feature.boundingRect, cv::Scalar(255,255,255));
    cv::Point centerPoint = feature.centerPoint.getImageCoordinates();
    cv::circle(colorImage, cv::Point(centerPoint.x, centerPoint.y), 10, cv::Scalar(255,0,255));

    // Gets centerpoint and adds to point array for filtering
    Spatial::Coordinate coord = feature.centerPoint;
    points.push_back(coord);

    geometry_msgs::Point32 globalPoint = coord.getGlobalCoordinates();
    ROS_INFO_STREAM("GX: " << globalPoint.x << " GY: " << globalPoint.y << " GZ: " << globalPoint.z);

    // Checks to see if the point array has reached the required number of points to publish
    if(points.size() >= POINT_COUNT_GOAL - 1) {
        Utils::CoordinateSmoother smoother;
        pointsPub.publish(smoother.getSmoothedGlobalCoordinate(points)); // Publishes global coordinates to topic
        points.clear();
    }

    sensor_msgs::Image output = converter.convertCVImageToMessage(colorImage, colorMessage->encoding); // Converts processed image to message
    imagePub.publish(output); // Publishes processed image message
}

bool getFeature(task_detector::GetFeature::Request &request, task_detector::GetFeature::Response &response) {
    while(points.size() < POINT_COUNT_GOAL-1) {
        ROS_INFO("Waiting for desired number of images");
    }

    ROS_INFO("Reached goal");

    Utils::CoordinateSmoother smoother;

    std::vector<geometry_msgs::Point32> featurePoints;
    featurePoints.push_back(smoother.getSmoothedGlobalCoordinate(points));

    response.points = featurePoints;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_detector");
    ros::NodeHandle n;

    pointsPub = n.advertise<geometry_msgs::Point32>("/task_detect/detected_points", 1);
    
    // Creates image output stream topic
    image_transport::ImageTransport transport(n);
    imagePub = transport.advertise("task_detect/output_video", 10);

    // Initializes task finder
    finder = Classification::TaskFinder(n);

    // Creates synchronised subscribers
    message_filters::Subscriber<sensor_msgs::Image> depthSub(n, "/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> imageSub(n, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub(n, "camera/color/camera_info", 1);

    // Creates sync policy type definition (for readability)
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> ImageSyncPolicy;

    // Creates synchronizer
    message_filters::Synchronizer<ImageSyncPolicy> sync(ImageSyncPolicy(10), depthSub, imageSub, infoSub);

    // Register synchronous callback with ROS
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    // Defines Service for gettting all recognised features
    ros::ServiceServer service = n.advertiseService("get_features", getFeature);

    // ros::spin();8888

    // Creates async ros spinner so image processing and service handling can occur simulatenously
    ros::AsyncSpinner asyncSpinner(8); // Note: This does not have to be 8 threads and can instead be any number above 3
    asyncSpinner.start();

    ros::waitForShutdown();
}