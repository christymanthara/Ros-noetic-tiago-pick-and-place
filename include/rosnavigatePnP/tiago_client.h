#ifndef TIAGOCLIENT_H
#define TIAGOCLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <rosnavigatePnP/TiagoMoveAction.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

class TiagoClient {
    private:
        actionlib::SimpleActionClient<rosnavigatePnP::TiagoMoveAction> client;       
        float x;
        float y;
        float z;

        float orx;
        float ory;
        float orz;
        float orw;

        void doneCb(const actionlib::SimpleClientGoalState &state, const rosnavigatePnP::TiagoMoveResultConstPtr &result);
        void activeCb();
        void feedbackCb(const rosnavigatePnP::TiagoMoveFeedbackConstPtr &feedback);

    public:
        TiagoClient(float x, float y, float z, std::string serverName);
        void sendGoal();
};

#endif // TIAGOCLIENT_H
