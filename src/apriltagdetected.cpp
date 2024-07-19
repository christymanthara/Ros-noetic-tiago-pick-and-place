#include "headers/utils.h"
#include <exception>
#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <control_msgs/PointHeadAction.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/topic.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


/**
 * @brief This method detects the tags of the objects on the table and returns them through the Detection.srv's response
 * @param req represents the request of Detection.srv
 * @param res represents the response of Detection.srv
 * @return TRUE if executes correctly and FALSE otherwise.
 */
bool detectTags(rosnavigatePnP::Detection::Request &req, rosnavigatePnP::Detection::Response &res){

	ROS_INFO("Incoming request: %s", req.ready ? "true" : "false");
	
	//simple action client to interact with tiago's head
	actionlib::SimpleActionClient<control_msgs::PointHeadAction> pointHeadClient("/head_controller/point_head_action", true); 
	
	int iterations = 0, max_iterations = 3;
	// Wait for head controller action server to come up
	while(!pointHeadClient.waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
	{
		ROS_INFO("Waiting for the point_head_action server to come up");
		++iterations;
	}
	if(iterations == max_iterations)
	    ROS_ERROR("Error in create PointHeadClient: head controller action server not available");

	
	//define a target point to make tiago point at (center of table to detect tags)
	geometry_msgs::PointStamped pointDown; 
	pointDown.header.frame_id = "/xtion_rgb_optical_frame"; 
	//pointStamped.header.stamp = latestImageStamp;
	pointDown.point.x = 0.00;
	pointDown.point.y = 0.80;
	pointDown.point.z = 1.00;  
	
	//define a target point to make tiago point at (in front of him to the next cylinder detection)
	geometry_msgs::PointStamped pointUp; 
	pointUp.header.frame_id = "/xtion_rgb_optical_frame"; 
	//pointStamped.header.stamp = latestImageStamp;
	pointUp.point.x = 0.00;
	pointUp.point.y = -0.80;
	pointUp.point.z = 1.00;  
	
	//the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
	control_msgs::PointHeadGoal goal;
	
	goal.pointing_frame = "/xtion_rgb_optical_frame";
	goal.pointing_axis.x = 0.0;
	goal.pointing_axis.y = 0.0;
	goal.pointing_axis.z = 1.0;
	goal.min_duration = ros::Duration(1.0);
	goal.max_velocity = 0.25;
	goal.target = pointDown;

	pointHeadClient.sendGoal(goal);
	
	bool finished_before_timeout = pointHeadClient.waitForResult(ros::Duration(20.0));
 
	//print result
    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = pointHeadClient.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Point down to the table");  
        }
    }else{
        ROS_ERROR("Point down did not finish before the timeout.");
        pointHeadClient.cancelGoal();
        ROS_ERROR("Point down goal has been cancelled");
    }
	
	apriltag_ros::AprilTagDetectionArray::ConstPtr apriltag_msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", ros::Duration(10.0));
	
	geometry_msgs::PoseWithCovarianceStamped poseCovarianceStamped;
	geometry_msgs::PoseWithCovariance poseCovariance;
	geometry_msgs::Pose pose;		// contains Point and Quaternion
	geometry_msgs::Point position;	// float x, y, z
	geometry_msgs::Quaternion orientation; // float x, y, z, w
	
	std::vector<apriltag_ros::AprilTagDetection> transformed_detections;
	
	for(int i = 0; i < apriltag_msg->detections.size() ; i++ )
	{
		poseCovarianceStamped = apriltag_msg->detections[i].pose;
		poseCovariance = poseCovarianceStamped.pose;
		pose = poseCovariance.pose;
		
		geometry_msgs::PoseStamped in_out_point;
		in_out_point.header.frame_id = "/xtion_rgb_optical_frame";
		in_out_point.pose = pose;
		
		try
		{
			tf::TransformListener tfListener;
		    tfListener.waitForTransform("/xtion_rgb_optical_frame", "/map", ros::Time(0), ros::Duration(3.0));
			tfListener.transformPose("/map", in_out_point, in_out_point);
		}
		catch (tf::TransformException& ex)
		{
		    ROS_ERROR("Failed to transform point to /map: %s", ex.what());
		    return false;
		} 
		
		apriltag_ros::AprilTagDetection transformed_detection;
		transformed_detection.pose.pose.pose = in_out_point.pose;
		transformed_detection.id = apriltag_msg->detections[i].id;
		transformed_detection.size = apriltag_msg->detections[i].size;
		transformed_detections.push_back(transformed_detection);
		
	}
	
	//make tiago point in up again
	goal.pointing_frame = "/xtion_rgb_optical_frame";
	goal.pointing_axis.x = 0.0;
	goal.pointing_axis.y = 0.0;
	goal.pointing_axis.z = 1.0;
	goal.min_duration = ros::Duration(1.0);
	goal.max_velocity = 0.25;
	goal.target = pointUp;

	pointHeadClient.sendGoal(goal);
	
	finished_before_timeout = pointHeadClient.waitForResult(ros::Duration(20.0));
 
	//print result
    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = pointHeadClient.getState();
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Point up");  
        }
    }else{
        ROS_ERROR("Point up did not finish before the timeout.");
        pointHeadClient.cancelGoal();
        ROS_ERROR("Point up goal has been cancelled");
		return false;
    }
    
	res.detections = transformed_detections;
	return true;
}


// Entry point
int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "detection_node");
	ros::NodeHandle n;
	if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
    	ROS_FATAL("Timed-out waiting for valid time.");
    	return EXIT_FAILURE;
  	}
  	
	ROS_INFO("Starting QR pose detection application ...");
	
	ros::ServiceServer service = n.advertiseService("/object_detection", detectTags);
	
	ros::spin();
	
	return 0;
}
