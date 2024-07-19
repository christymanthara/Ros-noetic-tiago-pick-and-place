#include <ros/ros.h>
#include <rosnavigatePnP/tiago_client.h>

// Constructor for the TiagoClient class also handles the poses
TiagoClient::TiagoClient(float x, float y, float z, std::string clientName): client(clientName, true)
{
    ROS_INFO("(Client) Tiago Client started");
    this->x = x;
    this->y = y;
    this->z = z;

}

// Callback for when robot starts the navigation
void TiagoClient::activeCb()
{
    ROS_INFO("(Client) GOAL SENT TO THE ROBOT.");
} 

// Callback for feedback messages
void TiagoClient::feedbackCb(const rosnavigatePnP::TiagoMoveFeedbackConstPtr &feedback)
{
    int state = feedback->state;
    if (state == 0)
        ROS_INFO("(Client) ROBOT IS MOVING.");
    else if (state == 1)
        ROS_INFO("(Client) ROBOT IS ARRIVED TO THE FINAL POSE.");
    else if (state == 2)
        ROS_INFO("(Client) ROBOT IS SCANNING THE ENVIRONMENT.");
    else if (state == 3)
        ROS_INFO("(Client) ROBOT ERROR IN NAVIGATION.");
    else if (state == 4)
        ROS_INFO("(Client) ROBOT ERROR IN DETECTION.");
}   
        
//function that send the goal to our Tiago Server
void TiagoClient::sendGoal()
{
    // Wait for the action server to come up so that we can begin processing goals.
    ROS_INFO("(Client) Waiting for the TiagoServerCommands to come up.");
    client.waitForServer();

    rosnavigatePnP::TiagoMoveGoal goal;

    // set the goal position
    goal.x = x;
    goal.y = y;
    goal.z = z;

    ROS_INFO("(Client) Sending goal");
    client.sendGoal(goal, boost::bind(&TiagoClient::doneCb, this, _1, _2),
                          boost::bind(&TiagoClient::activeCb, this),
                          boost::bind(&TiagoClient::feedbackCb, this, _1));
}

void TiagoClient::doneCb(const actionlib::SimpleClientGoalState &state,const rosnavigatePnP::TiagoMoveResultConstPtr &result)
{
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {	
		ROS_INFO("(Client) ROBOT HAS FINISHED: NAVIGATION AND DETECTION ARE DONE"); // success of the code
		// for (int i = 0; i < result->obstacles.size(); i++)
		// 	{
        //     ROS_INFO_STREAM("OBJECT DETECTED X : " << result->obstacles[i].x << " OBJECT DETECTED Y : " << result->obstacles[i].y);
		// 	}
        
        
    }
    else
    {
        ROS_WARN("(Client) ROBOT HAS FAILED: "); // failed
    }                                            // if-else 
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "start1");

    //read the command lines argument 
    if (argc != 4) {
        ROS_INFO("Please provide x, y and orientazione wrt. z axis of the goal position!");
        return 1;  
    } 

    float x,y,z; 
        float orx;
        float ory;
        float orz;
        float orw;
    char* endPtr; 
    // x = strtof(argv[1], &endPtr);
    // y = strtof(argv[2], &endPtr);
    // orZ = strtof(argv[3], &endPtr);

    //construct the client
    TiagoClient client(x,y,orZ,"TiagoServer"); 

    //send the goal
    client.sendGoal(); 

    ros::spin();
    return 0;
}

