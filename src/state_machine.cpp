#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/RcAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;


bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   actionlib::SimpleActionClient <rt2_assignment1::RcAction> pos("/go_to_point", true);
   
   rt2_assignment1::RcGoal gp;
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
        
        gp.x=rp.response.x; 
        gp.y=rp.response.y;  
        gp.theta=rp.response.theta;

        std::cout << "\nGoing to the position: x= " << gp.x << "\nGoing to the position: y= " <<gp.y << " \nGoing to the position:theta = " <<gp.theta << std::endl;
        pos.sendGoal(gp);
        
        while (true)
        {
           ros::spinOnce();
           if(start==false)
           {
                pos.cancelGoal();
                    std::cout << "\n currently goal cancelled" << std::endl;
                    break;
                }
                else
                {
                    if (pos.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        std::cout << "\ncurrently Goal Reached!" << std::endl;
                        break;
                    }
                }
            }
        }
    }
    return 0;
}