#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <utility>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "rosnavigatePnP/Aprildetect.h"

enum class Status {
    PickStarted = 0,
    PlaceStarted = 1,
    HandOpen = 2,
    HandClosed = 3,
    ArmHigh = 4,
    ArmLow = 5,
    ActionEnded = 6
};

struct CartesianCoordinates {
    float x;
    float y;
};

struct PoseID {
    CartesianCoordinates pose;
    int id;
};

float degreesToRadians(float degrees);
CartesianCoordinates polarToCartesian(float r, float theta);

// Scan Methods
std::vector<std::vector<CartesianCoordinates>> clusterRanges(const std::vector<CartesianCoordinates> &ranges,
                                                             float th1, float th2);

std::vector<CartesianCoordinates> convertRanges (const std::vector<float> &ranges,
                                                 float angle_min, float angle_increment);

std::vector<geometry_msgs::Pose> findCylinders(const std::vector<std::vector<CartesianCoordinates>> &rangeClusters,
                                               float angle_min, float angle_increment);

// Navigation Methods
struct Position {
    float x, y, z;
    float roll, pitch, yaw;
};

enum class StatusRobot : int {
    STOPPED = 0,
    MOVING = 1,
    REACHED_GOAL = 2,
    STARTED_SCAN = 3,
    ENDED_SCAN = 4
};

bool navigateRobotToGoal(const geometry_msgs::Pose& goalPosition);
bool navigateRobotToGoal(float x, float y, float z, float theta_z);

#endif // UTILS_H

