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

class TiagoClient{

    private: 
        actionlib::SimpleActionClient<rosnavigatePnP::TiagoMoveAction> client;       
        float x;
        float y;
        float z; 

        float orx;
        float ory;
        float orz;
        float orw;

        /** callback for action done
         * @param &state final state 
         * @param &result_ptr Boost Pointer to the final result of the move and detect action
        */
        void doneCb(const actionlib::SimpleClientGoalState &state,const rosnavigatePnP::TiagoMoveResultConstPtr &result_ptr);

        //callback for when robot starts the task
        void activeCb();

        /** callback for feedback messages
         * @param &feedback_ptr pointer to the MoveDetect Feedback messages
        */
        void feedbackCb(const rosnavigatePnP::TiagoMoveFeedbackConstPtr &feedback_ptr);

    public: 

        /*  
        @param x coordinate x of the final pose (w.r.t. map ref. frame)
        @param y coordinate y of the final pose (w.r.t. map ref. frame)
        @param orZ orientation in the z axis of the final pose (w.r.t. map ref. frame)
        */
        TiagoClient(float x, float y, float orZ, std::string serverName);
        
        /* method for sending the goal and start the task */
        void sendGoal();

}; 

#endif // TIAGOSERVER
