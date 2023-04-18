#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "/home/maro/DEBI_ws/devel/include/ball_detection_msg/BallPosition.h"

// Ball detection parameters
const int H_MIN = 0;
const int H_MAX = 255;
const int S_MIN = 0;
const int S_MAX = 255;
const int V_MIN = 0;
const int V_MAX = 255;
const int MIN_RADIUS = 10;
const int MAX_RADIUS = 100;

// Fixed ball radius (in meters)
const float BALL_RADIUS = 0.0275;

// Publisher for ball position topic
ros::Publisher ball_pub;

// Camera image subscriber callback function
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    // Apply color thresholding to extract ball pixels
    cv::Mat ball_mask;
    cv::inRange(hsv_image, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX, S_MAX, V_MAX), ball_mask);

    // Find ball contours and fit a circle
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(ball_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point2f center;
    float radius;
    if (contours.size() > 0)
    {
        cv::minEnclosingCircle(contours[0], center, radius);
    }

    // Publish ball position on topic
    ball_detection_msg::BallPosition ball_msg;
    if (radius > MIN_RADIUS && radius < MAX_RADIUS)
    {
        ball_msg.x = center.x;
        ball_msg.y = center.y;
        ball_msg.radius = BALL_RADIUS;
        ball_pub.publish(ball_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detection_node");
    ros::NodeHandle nh;

    // Subscribe to camera image topic
    ros::Subscriber image_sub = nh.subscribe("camera/image_raw", 1, imageCallback);

    // Initialize publisher for ball position topic
    ball_pub = nh.advertise<ball_detection_msg::BallPosition>("ball_position", 10);

    ros::spin();
    return 0;
}