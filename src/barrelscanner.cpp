// ROS headers
#include <exception>
#include <string>
#include <ros/ros.h>
#include <ros/topic.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosnavigatePnP/barrelscan.h>
#include <rosnavigatePnP/utilities.h>

// Variable Declaration
const std::string window_name = "Cylinder Image";
cv::Mat img;
ros::Time latestImageStamp;

bool barrelScanResult(rosnavigatePnP::barrelscan::Request &req, rosnavigatePnP::barrelscan::Response &res);
void processImage(const sensor_msgs::ImageConstPtr& imgMsg);
void imageRegionProcess(const cv::Mat& region, std::vector<int>& colorOrder);
std::vector<int> detectColorOrder(const cv::Mat &img);

int main(int argc, char** argv) {
    ros::init(argc, argv, "objectscanner_node");
    ros::NodeHandle n;

	ROS_INFO("Starting scan_node...");

	#ifndef HEADLESS
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    #endif
    //cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    ros::ServiceServer service = n.advertiseService("/objectscanner_node", barrelScanResult);
	ROS_INFO("Scan service advertised.");

    image_transport::ImageTransport it(n);
    image_transport::TransportHints transportHint("compressed");
    image_transport::Subscriber subToImage = it.subscribe("/xtion/rgb/image_raw", 1, processImage, transportHint);

    ros::spin();
    return 0;
}

// Service callback to process laser scan data and find color order
bool barrelScanResult(rosnavigatePnP::barrelscan::Request &req, rosnavigatePnP::barrelscan::Response &res){
	ROS_INFO("barrelScanResult called...");
    const std::vector<float> ranges = req.msg.ranges;
    const float angle_increment = req.msg.angle_increment;
    const float angle_min = req.msg.angle_min;

    bool success = false;
    geometry_msgs::Pose pose;
    std::vector<geometry_msgs::Pose> poses;
    
    float th_x = 0.7;
    float th_y = 0.2; 
    

    std::vector<CartesianCoordinates> cartesianRanges = convertRanges(ranges, angle_min, angle_increment);
    std::vector<std::vector<CartesianCoordinates>> cartesianRangesClusters = clusterRanges(cartesianRanges, th_x, th_y);
    poses = findCylinders(cartesianRangesClusters, angle_min, angle_increment);

    std::vector<geometry_msgs::Pose> return_vec;

    tf::TransformListener tfListener;

    // Transform poses from base_laser_link to map frame
    for(int i = 0; i< poses.size(); i++){
        geometry_msgs::PoseStamped in_out_point;
        in_out_point.header.frame_id = "base_laser_link";
        in_out_point.pose = poses[i];
        in_out_point.pose.orientation.x = 0.0;
        in_out_point.pose.orientation.y = 0.0;
        in_out_point.pose.orientation.z = 1.0;
        in_out_point.pose.orientation.w = 0.0;

        try {
            tfListener.waitForTransform("/base_laser_link", "/map", ros::Time(0), ros::Duration(3.0));
            tfListener.transformPose("/map", in_out_point, in_out_point);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Failed to transform point to /map: %s", ex.what());
            return false;
        }

        return_vec.push_back(in_out_point.pose);
    }
    // Find the color order in the image
    std::vector<int> colorOrder = detectColorOrder(img);

    // Ensure the number of detected poses matches the number of detected colors
    if (!colorOrder.empty() && !poses.empty() && colorOrder.size() == poses.size()) {
        success = true;
    }

    // Set The Response
    res.cylinderposes = return_vec;
    res.cylcolor = colorOrder;

    return success;
}

// Fucntion is used to process the incoming images.
void processImage(const sensor_msgs::ImageConstPtr& imgMsg) {
    latestImageStamp = imgMsg->header.stamp;

    cv_bridge::CvImagePtr cvImgPtr;
    try {
        cvImgPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cvImgPtr->image.copyTo(img);
}

// Find the color order in the image by dividing it into regions
std::vector<int> detectColorOrder(const cv::Mat &img) {
    std::vector<int> colorOrder;

    int width = img.cols;
    int height = img.rows;

    cv::Rect leftRect(0, 0, width / 3, height);
    cv::Rect centerRect(width / 3, 0, width / 3, height);
    cv::Rect rightRect(2 * width / 3, 0, width / 3, height);

    imageRegionProcess(img(rightRect), colorOrder);
    imageRegionProcess(img(centerRect), colorOrder);
    imageRegionProcess(img(leftRect), colorOrder);

    return colorOrder;
}

// Process a specific region of the image to find the dominant color
void imageRegionProcess(const cv::Mat& region, std::vector<int>& colorOrder) {
    cv::Mat regionRed, regionGreen, regionBlue;
    cv::inRange(region, cv::Scalar(0, 0, 200), cv::Scalar(50, 50, 255), regionRed);
    cv::inRange(region, cv::Scalar(0, 200, 0), cv::Scalar(50, 255, 50), regionGreen);
    cv::inRange(region, cv::Scalar(200, 0, 0), cv::Scalar(255, 50, 50), regionBlue);

    int redCount = cv::countNonZero(regionRed);
    int greenCount = cv::countNonZero(regionGreen);
    int blueCount = cv::countNonZero(regionBlue);

    if (redCount > greenCount && redCount > blueCount)
        colorOrder.push_back(3); //red
    else if (greenCount > redCount && greenCount > blueCount)
        colorOrder.push_back(2); //green
    else
        colorOrder.push_back(1); //blue
}
