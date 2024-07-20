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
#include <apriltag_ros/AprilTagDetection.h>

class TiagoServer
{
public:
    TiagoServer(std::string name);

private:
    bool executionDone = false;
    void navAndDetectCallback(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal);
    bool doNavigation(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal);
    // void doDetection();
    bool autoNavigate(const move_base_msgs::MoveBaseGoal &a_goal_pose);

    bool goToTable(int id);
    bool goToScanPosition(int id);
    bool goToCylinder(geometry_msgs::Pose pose);
    bool goHome(int id);

    rosnavigatePnP::TiagoMoveGoal createGoal(double x, double y, double z, double orx, double ory, double orz, double orw);

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<rosnavigatePnP::TiagoMoveAction> server;
    rosnavigatePnP::TiagoMoveFeedback feedback;
    rosnavigatePnP::TiagoMoveResult result;

    ObstacleDetector detector_;
};

#endif // TIAGO_SERVER_H
