#ifndef ARMFUNCTIONS_H
#define ARMFUNCTIONS_H

#include <ros/ros.h>
#include <rosnavigatePnP/ArmMoveAction.h>
#include <rosnavigatePnP/Aprildetect.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <apriltag_ros/AprilTagDetection.h>

int doPick(int object_order, ros::ServiceClient &detectionClient, actionlib::SimpleActionClient<rosnavigatePnP::ArmMoveAction> &acManipulation);
int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<rosnavigatePnP::ArmMoveAction> &acManipulation);
void feedbackManipulation(const rosnavigatePnP::ArmMoveFeedbackConstPtr& feedback);

#endif // ARMFUNCTIONS_H

