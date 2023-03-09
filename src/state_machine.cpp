#include <memory>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std::chrono_literals;

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1

{
	class Fin_State_Machi : public rclcpp::Node
	{
		public:
		
			Fin_State_Machi(const rclcpp::NodeOptions & options) : Node("state_machine", options)
			{ 
			  
				
				//State varaible initialization
				start = false;
				goal_reached = true;
				
				// Service and client initialization
				service_gui = this->create_service<Command>("/user_interface", std::bind(&Fin_State_Machi::user_interface, this, _1, _2, _3));
				client_go_to = this->create_client<Position>("/go_to_point");				
				client_random = this->create_client<RandomPosition>("/position_server");	
								
				while (!client_random->wait_for_service(std::chrono::seconds(3)))
				{
					if (!rclcpp::ok()) 
					{
						RCLCPP_ERROR(this->get_logger(), " CLIENT INTERRUPTED ");
						return;
					}
					RCLCPP_INFO(this->get_logger(), " RANDOM POSITION ");
				}   				
				RCLCPP_INFO(this->get_logger(), "RPS");
									
				while (!client_go_to->wait_for_service(std::chrono::seconds(3)))
				{
					if (!rclcpp::ok()) 
					{
						RCLCPP_ERROR(this->get_logger(), "INTERRUPT ERROR");
						return;
					}
					RCLCPP_INFO(this->get_logger(), "waiting for SERVICE");
				}

				RCLCPP_INFO(this->get_logger(), "CONNECTED TO POSITION SERVICE...");
				
				rp_req = std::make_shared<RandomPosition::Request>();
				rp_resp = std::make_shared<RandomPosition::Response>();
				p_req = std::make_shared<Position::Request>();
				
				 
				rp_req->x_max = 5.0;
				rp_req->x_min = -5.0;
				rp_req->y_max = 5.0;
				rp_req->y_min = -5.0;
				
				//Periodic status check
				RCLCPP_INFO(this->get_logger(), " VALIDATION...");
				timer_ = this->create_wall_timer(2000ms, std::bind(&Fin_State_Machi::status_check, this));
   				
			}			
			  
		private: 	
			  
								
			void status_check()
			{
				if (!goal_reached) return;
			
				if (!start) return;
			
				go_to_point();	
				RCLCPP_INFO(this->get_logger(), "GO TO POINT RUN...");				
			
			}		
			
			 
				
			void go_to_point()
			{
				
				call_randomPosition();	
				
				goal_reached = false;	
				
				p_req->x = rp_resp->x;
				p_req->y = rp_resp->y;
				p_req->theta = rp_resp->theta;
				
				RCLCPP_INFO(this->get_logger(), "Going to the position: x= %f y= %f theta= %f",
            				p_req->x, p_req->y, p_req->theta);
            				
				// Goal reached
				auto point_reached_callback = [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future){(void)future; goal_reached = true;
               			RCLCPP_INFO(this->get_logger(), "Mission complete!");};
				auto future_result = client_go_to->async_send_request(p_req, point_reached_callback);
				
			}
			
			  
			
			void call_randomPosition()
			{
			    auto RP = [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){rp_resp = future.get();};
			    auto future_result = client_random->async_send_request(rp_req,RP);
			}

			  
			void user_interface(
				const std::shared_ptr<rmw_request_id_t> request_header,
				const std::shared_ptr<Command::Request> req, 
				const std::shared_ptr<Command::Response> res)
			{	
				(void) request_header;
								
				if (req->command == "start")
				{
					start = true;
				}
				else 
				{
					start = false;
				}
				res->ok = start;
				RCLCPP_INFO(this->get_logger(), "Command received!");
			}
			
			bool start;
			bool goal_reached;
					
			rclcpp::Service<Command>::SharedPtr service_gui;          	
			rclcpp::Client<RandomPosition>::SharedPtr client_random;    
			rclcpp::Client<Position>::SharedPtr client_go_to;           

			std::shared_ptr<RandomPosition::Request> rp_req;    
			std::shared_ptr<RandomPosition::Response> rp_resp;  			
			std::shared_ptr<Position::Request> p_req;           
  
			rclcpp::TimerBase::SharedPtr timer_;				
	
	};
}


RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::Fin_State_Machi)
