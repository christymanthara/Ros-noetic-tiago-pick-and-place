#include <rosnavigatePnP/armfunctions.h>

void feedbackManipulation(const rosnavigatePnP::ArmMoveFeedbackConstPtr& feedback) {
    int state = feedback->state;

    switch (state) {
        case 0:
            ROS_INFO("Received manipulator state: Pick Started");
            break;
        case 1:
            ROS_INFO("Received manipulator state: Place Started");
            break;
        case 2:
            ROS_INFO("Received manipulator state: Gripper is Open");
            break;
        case 3:
            ROS_INFO("Received manipulator state: Gripper is Closed");
            break;
        case 4:
            ROS_INFO("Received manipulator states: Arm is moving");
            break;
        case 5:
            ROS_INFO("Received manipulator state: Object picked");
            break;
        case 6:
            ROS_INFO("Received manipulator state: Object placed");
            break;
        case 7:
            ROS_INFO("Received manipulator state: Arm is closed");
            break;
        default:
            ROS_INFO("Received unknown status");
            break;
    }
}

int doPick(int object_order, ros::ServiceClient &detectionClient, actionlib::SimpleActionClient<rosnavigatePnP::ArmMoveAction> &acManipulation)
{
    rosnavigatePnP::Aprildetect detection_srv;
    detection_srv.request.ready = true;
    detection_srv.request.april_id = object_order;

    rosnavigatePnP::ArmMoveGoal armGoal;

    if(detectionClient.call(detection_srv)){
        armGoal.request = 1;
        armGoal.pickid = object_order;
        armGoal.detected = detection_srv.response.detections;

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

    } else {
        ROS_ERROR("Detection service failed");
        ros::shutdown();
        return 1;
    }
    return 0;
}

int doPlace(int object_order, std::vector<apriltag_ros::AprilTagDetection> tempResponses, actionlib::SimpleActionClient<rosnavigatePnP::ArmMoveAction> &acManipulation)
{
    rosnavigatePnP::ArmMoveGoal armGoal;

    armGoal.request = 2;
    armGoal.pickid = object_order;
    armGoal.detected = tempResponses;

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

