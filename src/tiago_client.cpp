#include <ros/ros.h>
#include <rosnavigatePnP/tiago_client.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <rosnavigatePnP/TiagoMoveAction.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <assignment2/PoseAction.h>
#include <assignment2/ArmAction.h>
#include <assignment2/Scan.h>
#include "headers/utils.h"
#include "headers/navigation_methods.h"

const float x_offset = 0.1;
const float y_offset = 0.5;

template <typename Action>
bool isServerAvailable(const actionlib::SimpleActionClient<Action>& client, const std::string& serverName);

void feedbackNavigation(const assignment2::PoseFeedbackConstPtr& feedback);
void feedbackManipulation(const assignment2::ArmFeedbackConstPtr& feedback);

int doNavigation(int goalChoice, int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation, const apriltag_ros::AprilTagDetection &scanResponse);
int doPick(int object_order, ros::ServiceClient &detectionClient , actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation);
int doScan(ros::ServiceClient &scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg);
int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation);
bool doRecoveryNavigation(int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation);

TiagoClient::TiagoClient(float x, float y, float z, std::string clientName): client(clientName, true)
{
    ROS_INFO("(Client) Tiago Client started");
    this->x = x;
    this->y = y;
    this->z = z;
}

void TiagoClient::activeCb()
{
    ROS_INFO("(Client) GOAL SENT TO THE ROBOT.");
}

void TiagoClient::feedbackCb(const rosnavigatePnP::TiagoMoveFeedbackConstPtr &feedback)
{
    int state = feedback->state;
    switch (state) {
        case 0:
            ROS_INFO("Received status: STOPPED");
            break;
        case 1:
            ROS_INFO("Received status: MOVING");
            break;
        case 2:
            ROS_INFO("Received status: REACHED_GOAL");
            break;
        case 3:
            ROS_INFO("Received status: STARTED_SCAN");
            break;
        case 4:
            ROS_INFO("Received status: ENDED_SCAN");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

void TiagoClient::sendGoal()
{
    ROS_INFO("(Client) Waiting for the TiagoServerCommands to come up.");
    client.waitForServer();

    rosnavigatePnP::TiagoMoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.z = z;

    ROS_INFO("(Client) Sending goal");
    client.sendGoal(goal, boost::bind(&TiagoClient::doneCb, this, _1, _2),
                          boost::bind(&TiagoClient::activeCb, this),
                          boost::bind(&TiagoClient::feedbackCb, this, _1));
}

void TiagoClient::doneCb(const actionlib::SimpleClientGoalState &state, const rosnavigatePnP::TiagoMoveResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("(Client) ROBOT HAS FINISHED: NAVIGATION AND DETECTION ARE DONE");
    }
    else
    {
        ROS_WARN("(Client) ROBOT HAS FAILED");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "start1");
    ros::NodeHandle nh;

    ros::ServiceClient human_client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    tiago_iaslab_simulation::Objs human_srv;
    human_srv.request.ready = true;
    human_srv.request.all_objs = true;

    std::vector<int> object_order;

    if(human_client.call(human_srv)){
        for(int i = 0; i < human_srv.response.ids.size(); i++)
        {
            object_order.push_back(human_srv.response.ids[i]);
            ROS_INFO("Object ID: %d", (int)human_srv.response.ids[i]);
        }
    }
    else{
        ROS_ERROR("Failed to call service to get pick object order");
        ros::shutdown();
        return 1;
    }

    actionlib::SimpleActionClient<assignment2::PoseAction> acNavigation("poseRevisited", true);
    if (!isServerAvailable(acNavigation, "Navigation")) return 1;

    actionlib::SimpleActionClient<assignment2::ArmAction> acManipulation("manipulationNode", true);
    if (!isServerAvailable(acManipulation, "Manipulation")) return 1;

    ros::ServiceClient detectionClient = nh.serviceClient<assignment2::Detection>("/object_detection");
    ros::ServiceClient scan_client = nh.serviceClient<assignment2::Scan>("/scan_node");

    boost::shared_ptr<const sensor_msgs::LaserScan> msg;
    apriltag_ros::AprilTagDetection nullAprilTag;
    std::vector<int> ids;
    int returnVal;
    std::vector<apriltag_ros::AprilTagDetection> scanResponse;

    returnVal = doNavigation(1, object_order[0], acNavigation, nullAprilTag);
    if(returnVal == 1) return 1;
    else returnVal = 0;

    for(int i = 0; i < object_order.size(); i++){

        int order = object_order[i];
        int nextOrder = object_order[i+1];

        returnVal = doPick(object_order[i], detectionClient, acManipulation);
        if(returnVal == 1) return 1;
        else returnVal = 0;

        returnVal = doNavigation(2, order, acNavigation, nullAprilTag);
        if(returnVal == 1) return 1;
        else returnVal = 0;

        if(i == 0){
            msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);
            returnVal = doScan(scan_client, scanResponse, msg);
            if(returnVal == 1) return 1;
            else returnVal = 0;

            for(int i = 0; i < scanResponse.size(); i++){
                ids.push_back(scanResponse[i].id[0]);
            }
        }

        int correct_index;
        for(int k = 0; k < ids.size(); k++)
        {
            if(ids[k] == order)
                correct_index = k;
        }

        apriltag_ros::AprilTagDetection tempResponse;

        if(ids.size() == 3 && scanResponse.size() == 3)
        {
            tempResponse.pose.pose.pose.position.x = scanResponse[correct_index].pose.pose.pose.position.x - x_offset;
            tempResponse.pose.pose.pose.position.y = scanResponse[correct_index].pose.pose.pose.position.y - y_offset;
            tempResponse.pose.pose.pose.position.z = 0.00;
            tempResponse.id.push_back(correct_index);

            returnVal = doNavigation(3, order, acNavigation, tempResponse);
            if(returnVal == 1) return 1;
            else returnVal = 0;
        }
        else
        {
            returnVal = doRecoveryNavigation(order, acNavigation);
            if(returnVal == 1) return 1;
            else returnVal = 0;
        }

        returnVal = doPlace(correct_index, scanResponse, acManipulation);
        if(returnVal == 1) return 1;
        else returnVal = 0;

        if(i < 2){
            returnVal = doNavigation(4, nextOrder, acNavigation,  nullAprilTag);
            if(returnVal == 1) return 1;
            else returnVal = 0;
        }

        if (i == 2){
            returnVal = doNavigation(5, order, acNavigation,  nullAprilTag);
            if(returnVal == 1) return 1;
            else returnVal = 0;
        }
    }

    return 0;
}

template <typename Action>
bool isServerAvailable(const actionlib::SimpleActionClient<Action>& client, const std::string& serverName){
    ROS_INFO("Waiting for %s server to start.", serverName.c_str());

    if (!client.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("%s server not available, shutting down", serverName.c_str());
        ros::shutdown();
        return false;
    }

    ROS_INFO("%s server started", serverName.c_str());
    return true;
}

void feedbackNavigation(const assignment2::PoseFeedbackConstPtr& feedback) {
    int status = feedback->status;

    switch (status) {
        case 0:
            ROS_INFO("Received status: STOPPED");
            break;
        case 1:
            ROS_INFO("Received status: MOVING");
            break;
        case 2:
            ROS_INFO("Received status: REACHED_GOAL");
            break;
        case 3:
            ROS_INFO("Received status: STARTED_SCAN");
            break;
        case 4:
            ROS_INFO("Received status: ENDED_SCAN");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

void feedbackManipulation(const assignment2::ArmFeedbackConstPtr& feedback) {
    int status = feedback->status;

    switch (status) {
        case 0:
            ROS_INFO("Received status: Pick Started");
            break;
        case 1:
            ROS_INFO("Received status: Place Started");
            break;
        case 2:
            ROS_INFO("Received status: Gripper is Open");
            break;
        case 3:
            ROS_INFO("Received status: Gripper is Closed");
            break;
        case 4:
            ROS_INFO("Received status: Arm is moving");
            break;
        case 5:
            ROS_INFO("Received status: Object picked");
            break;
        case 6:
            ROS_INFO("Received status: Object placed");
            break;
        case 7:
            ROS_INFO("Received status: Arm is closed");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

int doNavigation(int goalChoice, int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation, const apriltag_ros::AprilTagDetection &scanResponse)
{
    assignment2::PoseGoal navigation_goal;
    navigation_goal.operation = goalChoice;
    navigation_goal.id = object_order;

    if(goalChoice == 3 ){
        navigation_goal.detection = scanResponse;
    }

    acNavigation.sendGoal(navigation_goal, NULL, NULL, &feedbackNavigation);
    bool finished_before_timeout = acNavigation.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state_final = acNavigation.getState();
        ROS_INFO("Action finished: %s", state_final.toString().c_str());
        if (state_final == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Navigation to goal finished before timeout");
        }
    } else {
        ROS_ERROR("Navigation to goal did not finish before the timeout.");
        acNavigation.cancelGoal();
        ROS_ERROR("Goal has been cancelled");
        return 1;
    }

    return 0;
}

int doScan(ros::ServiceClient &scan_client, std::vector<apriltag_ros::AprilTagDetection> &scanResponse, boost::shared_ptr<const sensor_msgs::LaserScan> msg)
{
    assignment2::Scan srv2;
    srv2.request.msg = *msg;
    srv2.request.ready = true;

    if(scan_client.call(srv2))
    {
        for(int i=0; i<srv2.response.poses.size(); i++)
        {
            apriltag_ros::AprilTagDetection cylinder_pose;

            cylinder_pose.pose.pose.pose = srv2.response.poses[i];
            cylinder_pose.id.push_back(srv2.response.ids_associated_colors[i]);

            scanResponse.push_back(cylinder_pose);
        }
    }
    else
    {
        ROS_ERROR("Failed to call Scan Service");
        return 1;
    }

    return 0;
}

int doPick(int object_order, ros::ServiceClient &detectionClient , actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{
    assignment2::Detection detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.requested_id = object_order;

    assignment2::ArmGoal armGoal;

    if(detectionClient.call(detection_srv)){
        armGoal.request = 1;
        armGoal.id = object_order;
        armGoal.detections = detection_srv.response.detections;

        acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);

        bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));

        if (manipulation_finished_before_timeout) {
            actionlib::SimpleClientGoalState state = acManipulation.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Pick done");
            }
        } else {
            ROS_ERROR("Pick action did not finish before the timeout.");
            acManipulation.cancelGoal();
            ROS_ERROR("Pick goal has been cancelled");
        }

    }else{
        ROS_ERROR("Detection service failed");
        ros::shutdown();
        return 1;
    }
    return 0;
}

int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<assignment2::ArmAction> &acManipulation)
{
    assignment2::ArmGoal armGoal;

    armGoal.request = 2;
    armGoal.id = object_order;
    armGoal.detections = tempResponses;

    acManipulation.sendGoal(armGoal, NULL, NULL, &feedbackManipulation);

    bool manipulation_finished_before_timeout = acManipulation.waitForResult(ros::Duration(60.0));

    if (manipulation_finished_before_timeout) {
        actionlib::SimpleClientGoalState state = acManipulation.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Place done");
        }
    } else {
        ROS_ERROR("Place action did not finish before the timeout.");
        acManipulation.cancelGoal();
        ROS_ERROR("Place goal has been cancelled");
    }

    return 0;
}

bool doRecoveryNavigation(int object_order, actionlib::SimpleActionClient<assignment2::PoseAction> &acNavigation){
    ROS_WARN("RECOVERY PLAN IN USE FOR NAVIGATION!");

    geometry_msgs::Pose bluePose;    geometry_msgs::Pose greenPose;     geometry_msgs::Pose redPose;
    bluePose.position.x = 12.4;      bluePose.position.y = -0.85;       bluePose.position.z = 0;
    greenPose.position.x = 11.4;     greenPose.position.y = -0.85;      greenPose.position.z = 0;
    redPose.position.x = 10.4;       redPose.position.y = -0.85;        redPose.position.z = 0;

    apriltag_ros::AprilTagDetection tempResponse;

    switch(object_order){
        case 1:
            tempResponse.pose.pose.pose = bluePose;
            break;
        case 2:
            tempResponse.pose.pose.pose = greenPose;
            break;
        case 3:
            tempResponse.pose.pose.pose = redPose;
            break;
        default:
            ROS_ERROR("ERROR IN RECOVERY NAVIGATION TO CYLINDERS. ORDER = %d", object_order);
            return false;
    }

    tempResponse.id.push_back(object_order);

    return doNavigation(3, object_order, acNavigation, tempResponse );
}