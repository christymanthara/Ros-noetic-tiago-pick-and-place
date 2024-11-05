#include <rosnavigatePnP/utilities.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>

float degreesToRadians(float degrees) {
    return degrees * (M_PI / 180.0);
}

//Struct object for cartesiancoordinates
CartesianCoordinates polarToCartesian(float r, float theta){
    float x = r * cos(theta);
    float y = r * sin(theta);
    return {x, y};
}


//Conversion of the coordinates from the polar to cartesian.
std::vector<CartesianCoordinates> convertRanges(const std::vector<float>& ranges, float angle_min, float angle_increment) {
    std::vector<CartesianCoordinates> resultRanges;
    for (int i = 0; i < ranges.size(); i++) {
        CartesianCoordinates currCord = polarToCartesian(ranges[i], angle_min + angle_increment * i);
        resultRanges.push_back(currCord);
    }
    return resultRanges;
}

// Calculate the cluster from the range fields.
std::vector<std::vector<CartesianCoordinates>> clusterRanges(const std::vector<CartesianCoordinates> &ranges, float th1, float th2) {
    std::vector<std::vector<CartesianCoordinates>> clusters;
    std::vector<CartesianCoordinates> currentCluster;

    for (int i = 0; i < ranges.size(); i++) {
        float x_diff = 0;
        float y_diff = 0;
        if (!currentCluster.empty()) {
            x_diff = std::abs(ranges[i].x - currentCluster.back().x);
            y_diff = std::abs(ranges[i].y - currentCluster.back().y);
        }

        if (currentCluster.empty() || (x_diff <= th1 && y_diff <= th2)) {
            currentCluster.push_back(ranges[i]);
        } else {
            clusters.push_back(currentCluster);
            currentCluster = { ranges[i] };
        }
    }
    
    if (!currentCluster.empty())
        clusters.push_back(currentCluster);

    return clusters;
}

// Finding the poses for the three cylinders
std::vector<geometry_msgs::Pose> findCylinders(const std::vector<std::vector<CartesianCoordinates>>& rangeClusters, float angle_min, float angle_increment) {
    std::vector<geometry_msgs::Pose> poses;
    std::vector<float> step_sizes;
    for (int i = 0; i < rangeClusters.size()-1; i++) {
        float step = rangeClusters[i].back().x - rangeClusters[i+1][0].x;
        step_sizes.push_back(step);
    }

    int angle_counter = 0;
    for (int i = 0; i < rangeClusters.size(); i++) {
        std::vector<CartesianCoordinates> currentRange = rangeClusters[i];
        geometry_msgs::Pose pp;
        
        if (step_sizes[i] > 0 || currentRange.size() < 30 || currentRange.size() > 50) {
            angle_counter += currentRange.size();
        } else {
            float x_start = currentRange[0].x;
            float y_start = currentRange[0].y;
            angle_counter += currentRange.size();
            float x_end = currentRange.back().x;
            float y_end = currentRange.back().y;
            float x = (x_start + x_end) / 2;
            float y = (y_start + y_end) / 2;
            pp.position.x = x;
            pp.position.y = y;
            pp.position.z = 0.00;
            poses.push_back(pp);
        }
    }
    return poses;
}

