#include <rosnavigatePnP/tiago_server.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

TiagoServer::TiagoServer(std::string name)
    : server(nh, name, boost::bind(&TiagoServer::navAndDetectCallback, this, _1), false)
{
    server.start();
    ROS_INFO_STREAM("(Server) TIAGO SERVER STARTED");
}

void TiagoServer::doDetection()
{
    ROS_INFO_STREAM("(Server) ROBOT IS STARTING THE DETECTION");
    feedback.state = 2; 
    server.publishFeedback(feedback);

    sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

    if (!msg)
    {
        ROS_WARN("(Server) Failed to get laser scan data within timeout period");
        feedback.state = 4;
        server.publishFeedback(feedback);
        server.setAborted(result);
        return;
    }

    detector_.obstacle_detection(msg);

    if (!detector_.isDetectionSuccessful()) 
    {
        ROS_WARN("(Server) Obstacle detection failed");
        feedback.state = 4; 
        server.publishFeedback(feedback);
        server.setAborted(result);
        return;
    }

    int obstacle_count = detector_.getObstacleCount();
    ROS_INFO_STREAM("(Server) NUMBER OF OBSTACLES DETECTED: " << obstacle_count);

    geometry_msgs::PoseArray obstacles = detector_.getObstacles();
    for (const auto& pose : obstacles.poses) {
        geometry_msgs::Point obstaclepoint;
        obstaclepoint.x = pose.position.x;
        obstaclepoint.y = pose.position.y;
        obstaclepoint.z = pose.position.z;
        result.obstacles.push_back(obstaclepoint);
    }

    ROS_INFO_STREAM("(Server) DETECTION IS FINISHED");
    server.setSucceeded(result);
}


bool TiagoServer::auto_moving_routine(const move_base_msgs::MoveBaseGoal &a_goal_pose)
{
    // Create the action client
    // We also tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_("move_base", true);
    // Wait for the action server to come up
    while (!move_base_ac_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_ac_.sendGoal(a_goal_pose);

    // Wait for the action to return
    move_base_ac_.waitForResult(); // Will wait for infinite time

    if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void TiagoServer::doNavigation(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) Received navigation goal: (" << goal->x << ", " << goal->y << ", " << goal->orZ << ")");
    // Create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    // Set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.orientation.z = goal->orZ;

    // All is wrt. world (map) ref frame
    goalMsg.target_pose.pose.orientation.w = 1.0;

    // Use auto_moving_routine to send the goal and wait for the result
    bool success = auto_moving_routine(goalMsg);

    if (success)
    {
        ROS_INFO_STREAM("(Server) ROBOT IS AT THE FINAL GOAL POSITION");
        feedback.state = 1; //(Client) ROBOT IS ARRIVED TO THE FINAL POSE.
        server.publishFeedback(feedback);

        // Delay of 20 seconds
        ros::Duration(10.0).sleep();

        // Start detection only if navigation is successful
        doDetection(); 
    }
    else
    {
        ROS_WARN_STREAM("(Server) ROBOT FAILED TO REACH THE FINAL GOAL POSITION");
        feedback.state = 3; //(Client) ROBOT FAILED TO REACH THE FINAL POSE.
        server.publishFeedback(feedback);
        server.setAborted(result);
    }
}

void TiagoServer::navAndDetectCallback(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal)
{
    // Calling the navigation function to navigate to the final pose
    doNavigation(goal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_server");

    TiagoServer server("TiagoServer");

    ros::spin();
    return 0;
}

