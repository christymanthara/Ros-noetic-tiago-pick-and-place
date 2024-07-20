#include <rosnavigatePnP/tiago_server.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

TiagoServer::TiagoServer(std::string name)
    : server(nh, name, boost::bind(&TiagoServer::navAndDetectCallback, this, _1), false)
{
    server.start();
    ROS_INFO_STREAM("(Server) TIAGO SERVER STARTED");
}

// void TiagoServer::doDetection()
// {
//     ROS_INFO_STREAM("(Server) ROBOT IS STARTING THE DETECTION");
//     feedback.state = 2; 
//     server.publishFeedback(feedback);

//     sensor_msgs::LaserScanConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

//     if (!msg)
//     {
//         ROS_WARN("(Server) Failed to get laser scan data within timeout period");
//         feedback.state = 4;
//         server.publishFeedback(feedback);
//         server.setAborted(result);
//         return;
//     }

//     detector_.obstacle_detection(msg);

//     if (!detector_.isDetectionSuccessful()) 
//     {
//         ROS_WARN("(Server) Obstacle detection failed");
//         feedback.state = 4; 
//         server.publishFeedback(feedback);
//         server.setAborted(result);
//         return;
//     }

//     int obstacle_count = detector_.getObstacleCount();
//     ROS_INFO_STREAM("(Server) NUMBER OF OBSTACLES DETECTED: " << obstacle_count);

//     geometry_msgs::PoseArray obstacles = detector_.getObstacles();
//     for (const auto& pose : obstacles.poses) {
//         geometry_msgs::Point obstaclepoint;
//         obstaclepoint.x = pose.position.x;
//         obstaclepoint.y = pose.position.y;
//         obstaclepoint.z = pose.position.z;
//         result.obstacles.push_back(obstaclepoint);
//     }

//     ROS_INFO_STREAM("(Server) DETECTION IS FINISHED");
//     server.setSucceeded(result);
// }


bool TiagoServer::autoNavigate(const move_base_msgs::MoveBaseGoal &a_goal_pose)
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
    // move_base_ac_.waitForResult(); // Will wait for infinite time
    // Wait for the robot to reach the goal before a fixed timeout
    bool goalReached = move_base_ac_.waitForResult(ros::Duration(60.0));
	


    if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return true;
    }
    else
    {
        return false;
    }
    if(!goalReached) move_base_ac_.cancelGoal();
    return goalReached;

}

void TiagoServer::doNavigation(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal)
{
    ROS_INFO_STREAM("(Server) Received navigation goal: (" << goal->x << ", " << goal->y << ", " << goal->z << ", " << goal->orx << ", " << goal->ory << ", " << goal->orz << ", " << goal->orw << ")");
    // Create the MoveBase message
    move_base_msgs::MoveBaseGoal goalMsg;

    // Set the goal position
    goalMsg.target_pose.header.frame_id = "map";
    goalMsg.target_pose.header.stamp = ros::Time::now();

    goalMsg.target_pose.pose.position.x = goal->x;
    goalMsg.target_pose.pose.position.y = goal->y;
    goalMsg.target_pose.pose.position.z = goal->z;

     // All is wrt. world (map) ref frame
    goalMsg.target_pose.pose.orientation.x = goal->orx;
    goalMsg.target_pose.pose.orientation.y = goal->ory;
    goalMsg.target_pose.pose.orientation.z = goal->orz;  
    goalMsg.target_pose.pose.orientation.w = goal->orw;

    // Use autoNavigate to send the goal and wait for the result
    bool success = autoNavigate(goalMsg);

    if (success)
    {
        ROS_INFO_STREAM("(Server) ROBOT IS AT THE GOAL POSITION");
        feedback.state = 1; //(Client) ROBOT IS ARRIVED TO THE FINAL POSE.
        server.publishFeedback(feedback);

        // Delay of 10 seconds
        ros::Duration(10.0).sleep();

        // Start detection only if navigation is successful
        // doDetection(); 
    }
    else
    {
        ROS_WARN_STREAM("(Server) ROBOT FAILED TO REACH THE GOAL POSITION");
        feedback.state = 3; //(Client) ROBOT FAILED TO REACH THE FINAL POSE.
        server.publishFeedback(feedback);
        server.setAborted(result);
    }
}

void TiagoServer::navAndDetectCallback(const rosnavigatePnP::TiagoMoveGoalConstPtr &goal)
{
    // Calling the navigation function to navigate to the final pose
    // doNavigation(goal);
        feedback.state = 0;
        server.publishFeedback(feedback);
        
        //Initializing data for goalPosition
        int id = goal->id;
        geometry_msgs::Pose pose;
        
	// MOVING
     //   feedback.state = 1;
     //   server.publishFeedback(feedback);
        
        switch (goal->operation) 
        {        
            case 1:
                ROS_INFO("Operation 1, reaching the table passing through home position");
                feedback.state = 1;
                server.publishFeedback(feedback);
                
                executionDone = doNavigation(createGoal(8.4, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
                if (executionDone) 
                	executionDone = goToTable(id);
                             
                break;

            case 2:
                ROS_INFO("Operation 2, reaching the scan position");
                executionDone = goToScanPosition(id); 
                break;
                
            case 3:
            	ROS_INFO("Operation 3, reaching the place position");

				pose = goal->detection.pose.pose.pose;
            	executionDone = goToCylinder(pose);
            	
            	break;
       
       		case 4:
       		   	ROS_INFO("Operation 4, reaching home position, then table position");
       			executionDone = goHome(id);
       			if (executionDone) 
       				executionDone = goToTable(id);
       				
                break;
                
            case 5:
               	ROS_INFO("Operation 5, reaching HOME position.");
               	executionDone = goHome(id);
               	
               	break;
               	
            default:
                ROS_ERROR("Error in Navigation CallBack.");
                break;
        }

	// ARRIVED
        if (executionDone) {
        //    feedback.state = 2; // Sending REACHED_GOAL to feedback
        //    server.publishFeedback(feedback);
            result.arrivedstatus = executionDone;
            server.setSucceeded(result);
        } else {
            ROS_INFO("Navigation aborted - Timeout reached");
            result.arrivedstatus = executionDone;
            server.setAborted(result_);
        }
    }




rosnavigatePnP::TiagoMoveGoal TiagoServer::createGoal(double x, double y, double z, double orx, double ory, double orz, double orw)
{
    rosnavigatePnP::TiagoMoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.z = z;
    goal.orx = orx;
    goal.ory = ory;
    goal.orz = orz;
    goal.orw = orw;
    return goal;
}

bool TiagoServer::goToTable(int id) 
{
    feedback.state = 1;
    server.publishFeedback(feedback);

    // Create the goal object
    rosnavigatePnP::TiagoMoveGoal goal;
    
    // Set the goal based on the table ID
     switch (id) {
        case 1:
            // Final position for BLUE
            goal = createGoal(8.15, -2.1, 0.0, 0.0, 0.0, -0.819152, -0.573576);
            break;
        case 2:
            //second waypoint
            goal = createGoal(8.40, -4.2, 0.0, 0.0, 0.0, 1.0, 0.0);
            doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>(goal));
            // Final position for GREEN
            createGoal(7.50, -4.00, 0.0, 0.0, 0.0, 0.573576, 0.819152);


            break;
        case 3:
            // Final position for RED
            goal = createGoal(7.20, -2.1, 0.0, 0.0, 0.0, -0.422618, 0.906308);
;
            break;
        default:
            ROS_ERROR("Error with object ordering");
            return false;
    }

    // Call the existing doNavigation method with the goal object
    doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>(goal));

    feedback.state = 2;
    server.publishFeedback(feedback);
            
    feedback.state = 0;
    server.publishFeedback(feedback);

    return true;
}

bool TiagoServer::goToScanPosition(int id) {
        if (id == 1 || id == 3) 
        {

			feedback.state = 1;
			server.publishFeedback(feedback);
            
            doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(8.4, -2, 0.0, 0.0, 0.0, 0.0, 1.0));
            
            feedback.state = 2;
            server.publishFeedback(feedback);
            
            feedback.state = 0;
			server.publishFeedback(feedback);
		
            
        }else
        {
        	feedback.state = 1;
			server.publishFeedback(feedback);
            
            // tiago started crashing into the table after taking the 1st video, so we added this waypoint
            doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(8.0, -4.3, 0.0, 0.0, 0.0, 0.0, 1.0));
            
            feedback.state = 2;
            server.publishFeedback(feedback);
            
            feedback.state = 0;
			server.publishFeedback(feedback);
        }
        

        // 2nd Waypoint to look at the cylinders from the center
		feedback.state = 1;
		server.publishFeedback(feedback);
		
        doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(10.3, -4.3, 0.0, 0.0, 0.0, 0.0, 1.0));
        
        bool returned = doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(11.33, -2.5, 0.0, 0.0, 0.0, 0.731354, 0.681998));
        if(returned){
        	feedback.state = 2;
            server.publishFeedback(feedback);
        }
        
        feedback.state = 0;
		server.publishFeedback(feedback);
		
        
        return returned;
    }
    
     bool TiagoServer::goToCylinder(geometry_msgs::Pose pose) {

		feedback.state = 1;
		server.publishFeedback(feedback);
        
        return doNavigation(pose.position.x, pose.position.y, pose.position.z, 90.0);
    }

bool TiagoServer::goHome(int id){
       ROS_INFO("Operation 0, going HOME");
       
       doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(10.3, -4.3, 0.0, 0.0, 0.0, 1.0, 0.0));
       
       bool returned = doNavigation(boost::make_shared<rosnavigatePnP::TiagoMoveGoal>createGoal(8.4, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0));	 // HOME
       if(returned){
        	feedback.state = 2;
            server.publishFeedback(feedback);
        }
        
        feedback.state = 0;
		server.publishFeedback(feedback);
		
        
        return returned;
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tiago_server");

    TiagoServer server("TiagoServer");
    ROS_INFO("Server is running...");

    ros::spin();
    return 0;
}

