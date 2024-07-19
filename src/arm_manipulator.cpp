// ROS headers
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <control_msgs/PointHeadAction.h>
#include <ros/topic.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


// Our headers
#include <rosnavigatePnP/ArmAction.h>
// #include "headers/utils.h"

// Std C++ headers
#include <exception>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <unistd.h>


class ArmAction {
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<rosnavigatePnP::ArmMoveAction> server;
    std::string action_name_;
    rosnavigatePnP::ArmMoveFeedback Armfeedback;
    rosnavigatePnP::ArmMoveResult Armresult;
    moveit::planning_interface::PlanningSceneInterface plan_scene_interface;
	moveit::planning_interface::MoveGroupInterface arm_group;
	moveit::planning_interface::MoveGroupInterface gripper_group;

public:

    ArmMoveAction(std::string name) : server(nh_, name, boost::bind(&ArmMoveAction::executeArmMove, this, _1), false), action_name_(name),
    plan_scene_interface(),
    arm_group("arm_torso"),
    gripper_group("gripper")
    {
    	server.start();
    }
    
    ~ArmMoveAction(void){}
    

	/**
	 * @brief Creates and adds the collision objects on the table to perform the pick, called from the method "pick"
	 * @param detections vector of detections of the tags
	 */
     void ArmAction::addCollisionObjects(std::vector<apriltag_ros::AprilTagDetection> detections)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Adding the table collision object
    moveit_msgs::CollisionObject table_object;
    table_object.id = "table";
    table_object.header.frame_id = "map";

    // Define the shape of the table collision object
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = shape_msgs::SolidPrimitive::BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 1.0;  // x dimension
    table_primitive.dimensions[1] = 1.0;  // y dimension
    table_primitive.dimensions[2] = 0.755;  // z dimension

    // Set the pose of the table collision object (map frame)
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 7.82;
    table_pose.position.y = -2.98;
    table_pose.position.z = 0.375;

    table_object.primitives.push_back(table_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = moveit_msgs::CollisionObject::ADD;

    collision_objects.push_back(table_object);

    // Adding obstacle collision objects
    for (const auto& detection : detections)
    {
        moveit_msgs::CollisionObject obstacle_object;
        shape_msgs::SolidPrimitive obj_primitive;
        geometry_msgs::Pose object_pose;

        obstacle_object.id = std::to_string(detection.id[0]);
        obstacle_object.header.frame_id = "map";

        tf2::Quaternion obj2_quaternion(
            detection.pose.pose.pose.orientation.x,
            detection.pose.pose.pose.orientation.y,
            detection.pose.pose.pose.orientation.z,
            detection.pose.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(obj2_quaternion);
        tf2::Quaternion obj2_orient;

        switch (static_cast<int>(detection.id[0]))
        {
            case 1: // Cylinder case
                obj_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
                obj_primitive.dimensions.resize(2);
                obj_primitive.dimensions[0] = detection.pose.pose.pose.position.z - 0.755;  // height
                obj_primitive.dimensions[1] = sqrt(2 * (detection.size[0] * detection.size[0])) / 2 + 0.005;  // radius

                object_pose.position.x = detection.pose.pose.pose.position.x;
                object_pose.position.y = detection.pose.pose.pose.position.y;
                object_pose.position.z = detection.pose.pose.pose.position.z - obj_primitive.dimensions[0] / 2;
                object_pose.orientation = detection.pose.pose.pose.orientation;
                break;

            case 2: // Box case
                obj_primitive.type = shape_msgs::SolidPrimitive::BOX;
                obj_primitive.dimensions.resize(3);
                obj_primitive.dimensions[0] = detection.size[0] + 0.015;  // x dimension
                obj_primitive.dimensions[1] = 2 * (detection.size[0] + 0.015) * sqrt(2) / 2;  // y dimension
                obj_primitive.dimensions[2] = (detection.size[0] + 0.015) * sqrt(2) / 2;  // z dimension

                object_pose.position.x = detection.pose.pose.pose.position.x + 0.015;
                object_pose.position.y = detection.pose.pose.pose.position.y + 0.015;
                object_pose.position.z = detection.pose.pose.pose.position.z;

                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                obj2_orient.setRPY(0.0, 0.0, yaw);
                obj2_orient.normalize();
                object_pose.orientation = tf2::toMsg(obj2_orient);
                break;

            case 3: // Small box case
                obj_primitive.type = shape_msgs::SolidPrimitive::BOX;
                obj_primitive.dimensions.resize(3);
                obj_primitive.dimensions[0] = detection.size[0] + 0.015;  // x dimension
                obj_primitive.dimensions[1] = detection.size[0] + 0.015;  // y dimension
                obj_primitive.dimensions[2] = detection.size[0] + 0.015;  // z dimension

                object_pose.position.x = detection.pose.pose.pose.position.x;
                object_pose.position.y = detection.pose.pose.pose.position.y;
                object_pose.position.z = detection.pose.pose.pose.position.z - obj_primitive.dimensions[2] / 2;
                object_pose.orientation = detection.pose.pose.pose.orientation;
                break;

            default: // Default cylinder case
                obj_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
                obj_primitive.dimensions.resize(2);
                obj_primitive.dimensions[0] = detection.pose.pose.pose.position.z - 0.755 + 0.05;  // height
                obj_primitive.dimensions[1] = sqrt(2 * (detection.size[0] * detection.size[0])) / 2 + 0.02;  // radius

                object_pose.position.x = detection.pose.pose.pose.position.x;
                object_pose.position.y = detection.pose.pose.pose.position.y;
                object_pose.position.z = detection.pose.pose.pose.position.z - obj_primitive.dimensions[0] / 2;
                object_pose.orientation = detection.pose.pose.pose.orientation;
                break;
        }

        obstacle_object.operation = moveit_msgs::CollisionObject::ADD;
        obstacle_object.primitives.push_back(obj_primitive);
        obstacle_object.primitive_poses.push_back(object_pose);

        collision_objects.push_back(obstacle_object);
    }

    // Apply the collision objects to the planning scene
    plan_scene_interface.applyCollisionObjects(collision_objects);
}


		
	/**
	 * @brief picks the object identified by requestedID
	 * @param detections array output of AprilTagDetection, contains all poses and dimensions of the tags found during scan
	 * @param requestedID id of the object required to pick
	 * @return TRUE if executes correctly and FALSE otherwise.
	 */
    bool pick(std::vector<apriltag_ros::AprilTagDetection> detections, int requestedID){
    	
		Armfeedback.status = 0; // Pick Started
		server.publishFeedback(Armfeedback);

		int detection_index = 0;
		  while(detections[detection_index].id[0] != requestedID){
		  	detection_index++;
		}
		
    	// Creating approach/depart poses
        geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = "map";
        tf2::Quaternion original_quaternion(detections[detection_index].pose.pose.pose.orientation.x, detections[detection_index].pose.pose.pose.orientation.y, detections[detection_index].pose.pose.pose.orientation.z, detections[detection_index].pose.pose.pose.orientation.w);
        tf2::Matrix3x3 m(original_quaternion);
		double roll, pitch, yaw;
		tf2::Quaternion rotation_about_y;
		
		
		// Creating goal pose 
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        
        switch(detections[detection_index].id[0]){
        	case 1:
        		approach_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
				approach_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
				approach_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;
				
				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				approach_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        		goal_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.01; 
        		goal_pose.pose.orientation = approach_pose.pose.orientation;
        		
        		break;
        	case 2:
        		approach_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x + 0.015;
				approach_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y + 0.015;
				approach_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;
				roll = 0.0;
				yaw += M_PI / 2;
				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				approach_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x + 0.015;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y + 0.015;
        		goal_pose.pose.position.z =  0.755 + 0.12;
        		goal_pose.pose.orientation = approach_pose.pose.orientation;
        		
        		break;
        	case 3:
        		approach_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
				approach_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
				approach_pose.pose.position.z = detections[detection_index].pose.pose.pose.position.z + 0.20;  
				
				m.getRPY(roll, pitch, yaw);
				pitch = M_PI / 2;

				rotation_about_y.setRPY(roll, pitch, yaw);
				rotation_about_y.normalize();
				approach_pose.pose.orientation = tf2::toMsg(rotation_about_y);
				
        		goal_pose.pose.position.x = detections[detection_index].pose.pose.pose.position.x;
        		goal_pose.pose.position.y = detections[detection_index].pose.pose.pose.position.y;
        		goal_pose.pose.position.z = 0.755 + 0.12; 
        		goal_pose.pose.orientation = approach_pose.pose.orientation;
        		break;
        	default:
        		ROS_ERROR("Error creating appro and goal pose");
        		break;
        }
        
        
        //open gripper (safety measure)
		moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
		
		gripper_group.setStartStateToCurrentState();
		gripper_group.setPlanningTime(5.0);
		
		std::vector<double> open_gripper_values;
		open_gripper_values.push_back(0.044);
		open_gripper_values.push_back(0.044);
		gripper_group.setJointValueTarget(open_gripper_values);
		
    	bool success = bool(gripper_group.plan(gripper_plan));

	    if(!success)
	        ROS_ERROR("No plan found for opening gripper");
		else{
	    	ROS_INFO_STREAM("Plan found for opening gripper in " << gripper_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = gripper_group.move();
			sleep(2);

			Armfeedback.status = 2; // Gripper is open
			server.publishFeedback(Armfeedback);

			if (!bool(e))
			    ROS_ERROR("Error executing plan opening gripper");
			else
				ROS_INFO_STREAM("Motion to opening gripper ended, motion duration: " << (ros::Time::now() - start).toSec());
		}

		// Creating plan for appro
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	    
		//get home position to return in after picking
		std::vector<double> home_pose;
		home_pose = arm_group.getCurrentJointValues();
		
		//add ollision objects to make tiago avoid them while reaching approach_pose
		addCollisionObjects(detections);

		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(approach_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(7.0);
	    
	    success = bool(arm_group.plan(my_plan));

		Armfeedback.status = 4; // Arm is moving
		server.publishFeedback(Armfeedback);

	    if(!success)
	        ROS_ERROR("No plan found for approach_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for approach_pose in " << my_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();
			// Execute the Movement
			moveit::core::MoveItErrorCode error = arm_group.move();
			
			if (!bool(error))
			    ROS_ERROR("Error executing plan approach_pose");
			else
				ROS_INFO_STREAM("Motion to approach_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//Creating plan for goal
	    moveit::planning_interface::MoveGroupInterface::Plan my_plan_goal;
	   	
	   	arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(goal_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    success = bool(arm_group.plan(my_plan_goal));

	    if(!success)
	        ROS_ERROR("No plan found for goal_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for goal_pose in " << my_plan_goal.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan goal_pose");
			else
				ROS_INFO_STREAM("Motion to goal_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//attach the object to gripper_grasping_frame and allow fingers to touch collision object
		std::vector<std::string> touch_links;
		touch_links.push_back("gripper_left_finger_link");
		touch_links.push_back("gripper_right_finger_link");
		std::string object_id = std::to_string(detections[detection_index].id[0]);
		gripper_group.attachObject(object_id, "gripper_grasping_frame", touch_links);
    	
    	
		//close the gripper 
		//moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		gripper_group.setStartStateToCurrentState();
		gripper_group.setPlanningTime(2.0);
		
		std::vector<double> close_gripper_values;
		
		switch(detections[detection_index].id[0]){
			case 1:
				close_gripper_values.push_back(sqrt(2*(detections[detection_index].size[0]*detections[detection_index].size[0]))/2 - 0.001);
				close_gripper_values.push_back(sqrt(2*(detections[detection_index].size[0]*detections[detection_index].size[0]))/2 - 0.001);
				break;
			case 2:
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.001);
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.001);
				break;
			case 3:
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.001);
				close_gripper_values.push_back((detections[detection_index].size[0])/2 - 0.001);
    			break;
    		default:
    			ROS_ERROR("received object id to be attached represents an obstacle object, cannot attach it");
    			return false;
    	}
    	
    	gripper_group.setJointValueTarget(close_gripper_values);
		
    	success = bool(gripper_group.plan(gripper_plan));

	    if(!success)
	        ROS_ERROR("No plan found for closing gripper");
		else{
	    	ROS_INFO_STREAM("Plan found for closing gripper in " << gripper_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = gripper_group.move();

			Armfeedback.status = 3; // Gripper is closed
			server.publishFeedback(Armfeedback);

			if (!bool(e))
			    ROS_ERROR("Error executing plan closing gripper");
			else
				ROS_INFO_STREAM("Motion to closing gripper ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
    	
		Armfeedback.status = 5; // Object picked
		server.publishFeedback(Armfeedback);

    	//departs
    	moveit::planning_interface::MoveGroupInterface::Plan my_plan_departs;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    arm_group.setPoseTarget(approach_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(7.0);
	    
	    success = bool(arm_group.plan(my_plan_departs));

	    if(!success)
	        ROS_ERROR("No plan found for departs");
		else{
	    	ROS_INFO_STREAM("Plan found for departs in " << my_plan_departs.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan departs");
			else
				ROS_INFO_STREAM("Motion to departs ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		
		//plan to come back home
		moveit::planning_interface::MoveGroupInterface::Plan home_plan;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setPlanningTime(5.0);
		
		arm_group.setJointValueTarget(home_pose);
		
    	success = bool(arm_group.plan(home_plan));

	    if(!success)
	        ROS_ERROR("No plan found for returning to home configuration");
		else{
	    	ROS_INFO_STREAM("Plan found to return to home configuration in " << home_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();

			Armfeedback.status = 7; // Arm is closed
			server.publishFeedback(Armfeedback);

			if (!bool(e))
			    ROS_ERROR("Error executing plan return to home configuration");
			else
				ROS_INFO_STREAM("Motion to return to home configuration ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
    	
        
        return true;
    }
    
	/**
	 * @brief Adds the cylindes as collision objects
	 * @param detections array containing all necessary info to create the collision objects, as poses and dimensions
	 * @param correct_index index of the correct cylinder where we want to place the object 
	 */
    void addPlaceCollisionCylinder(std::vector<apriltag_ros::AprilTagDetection> detections, int correct_index){
    	
    	std::vector<moveit_msgs::CollisionObject> collision_objects;
		
		moveit_msgs::CollisionObject place_cylinder;
		shape_msgs::SolidPrimitive primitive;
		geometry_msgs::Pose cylinder_pose;
		
		std::string place_id = "place_cylinder_" + std::to_string(detections[correct_index].id[0]);
		place_cylinder.id = place_id; //object_order
		primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
		place_cylinder.header.frame_id = "map";
		
		primitive.dimensions.resize(2);
		primitive.dimensions[0] = 0.70;  // height
		primitive.dimensions[1] = 0.22;  // radius
		
		cylinder_pose.position.x = detections[correct_index].pose.pose.pose.position.x;
		cylinder_pose.position.y = detections[correct_index].pose.pose.pose.position.y;
		cylinder_pose.position.z = 0.35;
		cylinder_pose.orientation.x = 0.0;
		cylinder_pose.orientation.y = 0.0;
		cylinder_pose.orientation.z = 1.0;
		cylinder_pose.orientation.w = 0.0;

		place_cylinder.operation = place_cylinder.ADD;
		
		place_cylinder.primitives.push_back(primitive);
	    place_cylinder.primitive_poses.push_back(cylinder_pose);
	
	    collision_objects.push_back(place_cylinder);
		
		plan_scene_interface.applyCollisionObjects(collision_objects);
	}
    

	/**
	 * @brief places the object that tiago is holding in its gripper
	 * @param detections array containing all necessary info to place the object, as poses and dimensions
	 * @param correct_index index of the correct cylinder where we want to place the object
	 * @return TRUE if executes correctly and FALSE otherwise.
	 */
    bool place(std::vector<apriltag_ros::AprilTagDetection> detections, int correct_index){
		
		Armfeedback.status = 1; // Place Started
		server.publishFeedback(Armfeedback);


		float table_height = 0.69;		
		
		geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = "map";
        approach_pose.pose.position.x = detections[correct_index].pose.pose.pose.position.x;
        approach_pose.pose.position.y = detections[correct_index].pose.pose.pose.position.y;
        approach_pose.pose.position.z = table_height + 0.20;
		
		geometry_msgs::PoseStamped place_pose;
        place_pose.header.frame_id = "map";
        place_pose.pose.position.x = detections[correct_index].pose.pose.pose.position.x;
        place_pose.pose.position.y = detections[correct_index].pose.pose.pose.position.y;
        
		double roll = 0.0;
		double pitch = M_PI / 2;
		double yaw = 0.0;
		tf2::Quaternion rotation_about_y;
		rotation_about_y.setRPY(roll, pitch, yaw);
		rotation_about_y.normalize();
		
		place_pose.pose.orientation = tf2::toMsg(rotation_about_y);
		approach_pose.pose.orientation = tf2::toMsg(rotation_about_y);
        
        
        switch(detections[correct_index].id[0]){
        	case 1:
        		place_pose.pose.position.z = table_height + 0.17; 
        		break;
        	default:
        		place_pose.pose.position.z = table_height + 0.12; 
        		break;
        }
        
        
        //create place plan and execute associated motion
        moveit::planning_interface::MoveGroupInterface::Plan appro_plan;
        moveit::planning_interface::MoveGroupInterface::Plan place_plan;
	  	
		//create place_cylinder collision object to be avoided during place motion
		addPlaceCollisionCylinder(detections, correct_index);
		
		//place the object
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
	    
	    //get home_pose to return after place action
	    std::vector<double> home_pose;
		home_pose = arm_group.getCurrentJointValues();
		
		//set approach pose
	    arm_group.setPoseTarget(approach_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    bool success = bool(arm_group.plan(appro_plan));

		Armfeedback.status = 4; // Arm is moving
		server.publishFeedback(Armfeedback);

	    if(!success)
	        ROS_ERROR("No plan found for approach_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for approach_pose in " << appro_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan approach_pose");
			else
				ROS_INFO_STREAM("Motion to approach_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//set goal target
	    arm_group.setPoseTarget(place_pose, "gripper_grasping_frame"); //appro
	    arm_group.setPlanningTime(5.0);
	    
	    arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setMaxVelocityScalingFactor(1.0);
		
	    success = bool(arm_group.plan(place_plan));

	    if(!success)
	        ROS_ERROR("No plan found for place_pose");
		else{
	    	ROS_INFO_STREAM("Plan found for place_pose in " << place_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan place_pose");
			else
				ROS_INFO_STREAM("Motion to place_pose ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		//open gripper
		moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
		
		gripper_group.setStartStateToCurrentState();
		gripper_group.setPlanningTime(5.0);
		
		std::vector<double> open_gripper_values;
		open_gripper_values.push_back(0.044);
		open_gripper_values.push_back(0.044);
		gripper_group.setJointValueTarget(open_gripper_values);
		
    	success = bool(gripper_group.plan(gripper_plan));

	    if(!success)
	        ROS_ERROR("No plan found for opening gripper");
		else{
	    	ROS_INFO_STREAM("Plan found for opening gripper in " << gripper_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = gripper_group.move();
			sleep(2);

			Armfeedback.status = 2; // Gripper is open
			server.publishFeedback(Armfeedback);

			if (!bool(e))
			    ROS_ERROR("Error executing plan opening gripper");
			else
				ROS_INFO_STREAM("Motion to opening gripper ended, motion duration: " << (ros::Time::now() - start).toSec());
		}
		
		gripper_group.detachObject(std::to_string(detections[correct_index].id[0]));
		
		Armfeedback.status = 6; // Object placed
		server.publishFeedback(Armfeedback);

		//come back to home_pose
		moveit::planning_interface::MoveGroupInterface::Plan home_plan;
		
		arm_group.setPlannerId("SBLkConfigDefault");
		arm_group.setStartStateToCurrentState();
		arm_group.setPlanningTime(5.0);
		
		arm_group.setJointValueTarget(home_pose); //appro
		
    	success = bool(arm_group.plan(home_plan));

	    if(!success)
	        ROS_ERROR("No plan found for home_plan");
		else{
	    	ROS_INFO_STREAM("Plan found to return to home configuration in " << home_plan.planning_time_ << " seconds");
	    
			ros::Time start = ros::Time::now();

			// Execute the Movement
			moveit::core::MoveItErrorCode e = arm_group.move();
			if (!bool(e))
			    ROS_ERROR("Error executing plan return to home configuration");
			else
				ROS_INFO_STREAM("Motion to return to home configuration ended, motion duration: " << (ros::Time::now() - start).toSec());
		}

		Armfeedback.status = 7; // Arm is closed
		server.publishFeedback(Armfeedback);
	  	
        return true;
    }


    /**
     * @brief Callback function for executing the manipulation action.
     * @param goal The goal for the pose action.
     */
    void executeArmMove(const rosnavigatePnP::ArmMoveGoalConstPtr &goal) { 
				
		bool objectPicked = false;
		bool objectPlaced = false;
		
        switch(goal->request){
            case 1:
                objectPicked = pick(goal->detections, goal->id);
                if (objectPicked){
                    Armresult.objectPicked = objectPicked;
                    server.setSucceeded(Armresult);
                } else {
                    Armresult.objectPicked = objectPicked;
                    server.setAborted(Armresult);
                }
                break;
            case 2:
                objectPlaced = place(goal->detections, goal->id);
                if (objectPlaced){
                    Armresult.objectPlaced = objectPlaced;
                    server.setSucceeded(Armresult);
                } else {
                    Armresult.objectPlaced = objectPlaced;
                    server.setAborted(Armresult);
                }
                break;
            default:
                ROS_ERROR("Not a possible choice.");
                break;
        }
    }
};


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "manipulationNode");
    ArmMoveAction server("manipulationNode");
    ROS_INFO("Manipulation Server is running...");
 
    ros::spin(); 

	return 0;
}
