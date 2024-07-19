#ifndef TIAGO_SERVER_H
#define TIAGO_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "rosnavigatePnP/TiagoMoveAction.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include "detector.h"

class TiagoServer
{
public:
    TiagoServer(std::string name);

private:
    void navAndDetectCallback(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal);
    void doNavigation(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal);
    // void doDetection();
    bool auto_moving_routine(const move_base_msgs::MoveBaseGoal &a_goal_pose);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<rosnavigatePnP::TiagoMoveAction> server;
    rosnavigatePnP::TiagoMoveFeedback feedback;
    rosnavigatePnP::TiagoMoveResult result;

    ObstacleDetector detector_;
};

#endif // TIAGO_SERVER_H

