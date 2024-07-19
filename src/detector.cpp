#include "rosnavigatePnP/detector.h"

ObstacleDetector::ObstacleDetector() {
    ros::NodeHandle nh;
    obstacle_pub_ = nh.advertise<geometry_msgs::PoseArray>("obstacles", 10);
}

ObstacleDetector::~ObstacleDetector() {
}

float ObstacleDetector::length(const cv::Point2f& aPoint) const {  // Marked as const
    return sqrt(pow(aPoint.x, 2.0) + pow(aPoint.y, 2.0));
}

bool ObstacleDetector::isObstacle(const Circle& aCircle) const {
    if (aCircle.radius > max_radius || aCircle.radius < min_radius) {
        return false;
    }
    if (length(aCircle.center) < min_range || length(aCircle.center) > max_range) {
        return false;
    }
    return true;
}
bool ObstacleDetector::isDetectionSuccessful() const
{
    return detection_success_;
}
void ObstacleDetector::obstacle_detection(const sensor_msgs::LaserScanConstPtr& aLaser_msg) {
    my_input_points.clear();
    my_point_sets.clear();
    my_circles.clear();
	detection_success_ = true;
	
	// Convert laser scan data to 2D points
    for (int i = 19; i < aLaser_msg->ranges.size() - 18; i++) {
        float r = aLaser_msg->ranges.at(i);
        if (r >= aLaser_msg->range_min && r <= aLaser_msg->range_max) {
            float phi = aLaser_msg->angle_min + (i * aLaser_msg->angle_increment);
            my_input_points.push_back(cv::Point2f(r * cos(phi), r * sin(phi)));
        }
    }

    groupPoints(aLaser_msg->angle_increment);
    detectCircles();
    publishObstacles(aLaser_msg);
}

void ObstacleDetector::groupPoints(const float& angle_increment) {
    int start_index = 0;
    int end_index = 0;
    cv::Point2f point_start = my_input_points.at(0);
    cv::Point2f point_end = my_input_points.at(0);
    int num_points = 1;
    float distance_proportion = angle_increment;

    for (int i = 1; i < my_input_points.size(); i++) {
        float range = length(my_input_points.at(i));
        float distance = length(my_input_points.at(i) - point_end);

        if (distance < max_group_distance + range * distance_proportion) {
            point_end = my_input_points.at(i);
            num_points++;
            end_index = i;
        } else {
            detectPointSet(point_start, point_end, start_index, end_index, num_points);
            start_index = i;
            end_index = i;
            point_start = my_input_points.at(i);
            point_end = my_input_points.at(i);
            num_points = 1;
        }
    }

    detectPointSet(point_start, point_end, start_index, end_index, num_points);
}

void ObstacleDetector::detectPointSet(cv::Point2f& point_start, cv::Point2f& point_end, int& start_index, int& end_index, int& num_points) {
    if (num_points < min_group_points) {
        return;
    }

    PointSet tmp_ptSet;
    tmp_ptSet.first_point = point_start;
    tmp_ptSet.middle_point = my_input_points.at(start_index + ((end_index - start_index) / 2));
    tmp_ptSet.last_point = point_end;
    for (int i = start_index; i < my_input_points.size() && i <= end_index; i++) {
        tmp_ptSet.points.push_back(my_input_points.at(i));
    }
    my_point_sets.push_back(tmp_ptSet);
}

void ObstacleDetector::detectCircles() {
    for (const PointSet& ptSet : my_point_sets) {
        Circle circle;
        findCircle(circle, ptSet);
        if (is_a_Circle(circle, ptSet) && isObstacle(circle)) {
            my_circles.push_back(circle);
        }
    }
}

bool ObstacleDetector::is_a_Circle(Circle& aCircle, const PointSet& aPtSet) {
    if (aCircle.radius > max_radius) {
        return false;
    }

    const float ths = 0.01;
    for (int i = 0; i < aPtSet.points.size(); i++) {
        float dist = length(aPtSet.points.at(i) - aCircle.center);
        if (dist > (aCircle.radius + ths) || dist < (aCircle.radius - ths)) {
            return false;
        }
    }
    return true;
}

void ObstacleDetector::findCircle(Circle& circle, const PointSet& pt_set) {
    cv::Point2f p1 = pt_set.first_point;
    cv::Point2f p2 = pt_set.middle_point;
    cv::Point2f p3 = pt_set.last_point;
    const float TOL = 0.0000001;

    float yDelta_a = p2.y - p1.y;
    float xDelta_a = p2.x - p1.x;
    float yDelta_b = p3.y - p2.y;
    float xDelta_b = p3.x - p2.x;

    float aSlope = yDelta_a / xDelta_a;
    float bSlope = yDelta_b / xDelta_b;
    if (fabs(bSlope - aSlope) < TOL) {
        circle.center.x = 0.0;
        circle.center.y = 0.0;
        circle.radius = std::numeric_limits<float>::infinity();
        return;
    }
    circle.center.x = (aSlope * bSlope * (p1.y - p3.y) + bSlope * (p1.x + p2.x) - aSlope * (p2.x + p3.x)) / (2.0 * (bSlope - aSlope));
    circle.center.y = -1.0 * (circle.center.x - (p1.x + p2.x) / 2.0) / aSlope + (p1.y + p2.y) / 2.0;
    circle.radius = length(p2 - circle.center);
}

void ObstacleDetector::publishObstacles(const sensor_msgs::LaserScanConstPtr& aLaser_msg) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("base_link", aLaser_msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("base_link", aLaser_msg->header.frame_id.c_str(), ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
        return;
    }

    geometry_msgs::PoseArray obstacles;
    obstacles.header.stamp = ros::Time::now();
    obstacles.header.frame_id = "base_link";

    for (const Circle& aCircle : my_circles) {
        if (isObstacle(aCircle)) {
            geometry_msgs::Point p;
            p.x = aCircle.center.x;
            p.y = aCircle.center.y;
            p.z = 0.0;
            geometry_msgs::Point transformed_point = transformPoint(p, transform);
            geometry_msgs::Pose pose;
            pose.position = transformed_point;
            pose.orientation.w = 1.0;
            obstacles.poses.push_back(pose);
        }
    }
    obstacle_pub_.publish(obstacles);
}

geometry_msgs::Point ObstacleDetector::transformPoint(const geometry_msgs::Point& point, const tf::StampedTransform& transform) {
    tf::Point tf_point(point.x, point.y, point.z);
    tf::Point transformed_tf_point = transform * tf_point;
    geometry_msgs::Point transformed_point;
    transformed_point.x = transformed_tf_point.x();
    transformed_point.y = transformed_tf_point.y();
    transformed_point.z = transformed_tf_point.z();
    return transformed_point;
}

int ObstacleDetector::getObstacleCount() const {
    return my_circles.size();
}

geometry_msgs::PoseArray ObstacleDetector::getObstacles() const {
    geometry_msgs::PoseArray obstacles;
    for (const Circle& aCircle : my_circles) {
        if (isObstacle(aCircle)) {
            geometry_msgs::Pose pose;
            pose.position.x = aCircle.center.x;
            pose.position.y = aCircle.center.y;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            obstacles.poses.push_back(pose);
        }
    }
    return obstacles;
}

