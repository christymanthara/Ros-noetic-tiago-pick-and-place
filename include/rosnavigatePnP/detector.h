#ifndef ROSNAVIGATEPNP_DETECTOR_H
#define ROSNAVIGATEPNP_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

class ObstacleDetector {
public:
    ObstacleDetector();
    ~ObstacleDetector();
    void obstacle_detection(const sensor_msgs::LaserScanConstPtr& aLaser_msg);
    int getObstacleCount() const;
    geometry_msgs::PoseArray getObstacles() const;
	bool isDetectionSuccessful() const;

private:
    struct PointSet {
        cv::Point2f first_point;
        cv::Point2f middle_point;
        cv::Point2f last_point;
        std::vector<cv::Point2f> points;
    };

    struct Circle {
        cv::Point2f center;
        float radius;
    };

    bool isObstacle(const Circle& aCircle) const;
    void groupPoints(const float& angle_increment);
    void detectPointSet(cv::Point2f& point_start, cv::Point2f& point_end, int& start_index, int& end_index, int& num_points);
    void detectCircles();
    bool is_a_Circle(Circle& aCircle, const PointSet& aPtSet);
    void findCircle(Circle& circle, const PointSet& pt_set);
    void publishObstacles(const sensor_msgs::LaserScanConstPtr& aLaser_msg);
    geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, const tf::StampedTransform& transform);
    float length(const cv::Point2f& aPoint) const;

    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher obstacle_pub_;
    std::vector<cv::Point2f> my_input_points;
    std::vector<PointSet> my_point_sets;
    std::vector<Circle> my_circles;

    // Parameters for obstacle detection
    const float max_radius = 0.5;
    const float min_radius = 0.1;
    const float max_group_distance = 0.2;
    const float min_range = 0.1;
    const float max_range = 10.0;
    const int min_group_points = 5;
	bool detection_success_;
};

#endif // ROSNAVIGATEPNP_DETECTOR_H

