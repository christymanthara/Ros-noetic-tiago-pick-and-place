#include "../headers/navigation_methods.h"

/**
 * @brief This method navigates the robot to a specified goal pose (location + direction angle)
 * @param goalPosition which is the target position to navigate towards.
 * @return TRUE if the robot reaches the goal within the timeout and FALSE otherwise.
 */
bool navigateRobotToGoal(const geometry_msgs::Pose& goalPosition) 
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(20.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //define goal
    ros::Time startTime = ros::Time::now();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = startTime;
    
    //set goal position
    goal.target_pose.pose = goalPosition;
    
    // Set goal orientation
    goal.target_pose.pose.orientation.x = 0;	
    goal.target_pose.pose.orientation.y = 0;   
    goal.target_pose.pose.orientation.z = 1;    
    goal.target_pose.pose.orientation.w = 0;
    
	//send goal to ActionServer
    ac.sendGoal(goal);

    // Wait for the robot to reach the goal before a fixed timeout
    bool goalReached = ac.waitForResult(ros::Duration(60.0));
	if(!goalReached) ac.cancelGoal();
    return goalReached;
}
/**
 * @brief This method navigates the robot to a specified goal pose (location + direction angle)
 * @param x x coord 
 * @param y y coord 
 * @param z z coord 
 * @param theta_z rotation around z-axis
 * @return TRUE if the robot reaches the goal within the timeout and FALSE otherwise.
 */
bool navigateRobotToGoal(float x, float y, float z, float theta_z)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(20.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //define goal
    ros::Time startTime = ros::Time::now();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = startTime;
    
    //set goal position
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;
    
    float theta_z1 = degreesToRadians(theta_z);
    
    // Set goal orientation
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( 0, 0, theta_z1);
    myQuaternion.normalize();
    goal.target_pose.pose.orientation = tf2::toMsg(myQuaternion);

	//send goal to ActionServer
    ac.sendGoal(goal);

    // Wait for the robot to reach the goal before a fixed timeout
    bool goalReached = ac.waitForResult(ros::Duration(60.0));
	if(!goalReached) ac.cancelGoal();
    return goalReached;
}


