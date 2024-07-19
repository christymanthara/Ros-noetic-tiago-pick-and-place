#ifndef NAVIGATION_METHODS_H
#define NAVIGATION_METHODS_H

#include <vector>
#include "utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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

// declarations
bool navigateRobotToGoal(const geometry_msgs::Pose& goalPosition);
bool navigateRobotToGoal(float x, float y, float z, float theta_z);

#endif // NAVIGATION_METHODS_H
