#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std;
using UserInt = rt2_assignment1::srv::Command;
using GoalPos = rt2_assignment1::srv::Position;
using RndPos = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
bool start = false;
bool doneOnce= false;
namespace rt2_assignment1 {
	

class GoalPositionClient : public rclcpp::Node
	{
		public:
			GoalPositionClient() : Node("Goal_client")
				{
						// initialization of the client that calls the server /go_to_point
						client_ = this->create_client<GoalPos>("/go_to_point");
						while(!client_ -> wait_for_service(std::chrono::seconds(1)))
							{
								if (!rclcpp::ok())
									{
										RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
										return;
									}
							}
						this->request_=std::make_shared<GoalPos::Request>();
						this->response_=std::make_shared<GoalPos::Response>();	
				}
				
				void call_serverGTP()
					{
						auto result_future= client_->async_send_request(request_);
						if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
							{
								RCLCPP_ERROR(this->get_logger(), "service call failed");
							}
						this->response_=result_future.get();
					}
					
				std::shared_ptr<GoalPos::Request> request_;
				std::shared_ptr<GoalPos::Response> response_;
	
		private:
			rclcpp::Client<GoalPos>::SharedPtr client_;
	
	};	
	
class FSM : public rclcpp::Node
	{
		public:
			FSM(const rclcpp::NodeOptions & options) : Node("State_machine", options)
				{
					// creation of the timer
					// the function timer_callback will be executed every 500 ms
					timer_ =this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FSM::timer_callback, this));
					// initialization of the server /user_interface
					service_ = this->create_service<UserInt>("/user_interface", std::bind(&FSM::handle_service, this, _1, _2, _3));
					
					// initialization of the client for the position_server
					RP_client=this->create_client<RndPos>("/position_server");
					while(!RP_client -> wait_for_service(std::chrono::seconds(10)))
						{
							if (!rclcpp::ok())
								{
									RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
									return;
								}
						}
				}

		private:

			// function called when the client request this server
			void handle_service( 
			const std::shared_ptr<rmw_request_id_t> request_header,
			const std::shared_ptr<UserInt::Request> request,
			const std::shared_ptr<UserInt::Response> response)
			{
				(void) request_header;
				// if the request command is stop I set the global variable start to false
				if (request->command == "stop")
					{
						start= false;
						response->ok = true;
							
					}
				// if the request command is start I set the global variable start to true
				else if ( request->command == "start")
					{
						start= true;
						response->ok=true;	
					}
				response->ok=true;
			}
			
			void timer_callback()
				{
					auto random_position = std::make_shared<RndPos::Request>();
					// if I am executing the timer_callback right after 
					// one request was served I do nothing
					if (doneOnce)
						{
							doneOnce=false;
						}
					else if (start == false)
						{
							//do nothing
						}
					else if ( start == true)
						{
							// the position_server is called to get a new
							// random position in the feasible range
							random_position->x_max = 5.0;
							random_position->x_min = -5.0;
							random_position->y_max = 5.0;
							random_position->y_min = -5.0;
							using ServiceResponseFuture =rclcpp::Client<RndPos>::SharedFuture;
							auto response_received_callback= [this] (ServiceResponseFuture future) 
								{
									auto goal_position = std::make_shared<GoalPositionClient>();
									goal_position->request_->x=future.get()->x;
									goal_position->request_->y=future.get()->y;
									goal_position->request_->theta=future.get()->theta;
																std::cout << "\nGoing to the position: x= " << goal_position->request_->x << " y= " <<goal_position->request_->y << " theta = " <<goal_position->request_->theta << std::endl;
							goal_position->call_serverGTP();
							std::cout << "Position reached" << std::endl;
								};
							auto future_result= RP_client->async_send_request(random_position, response_received_callback);
							doneOnce=true;
						}		
				}
			rclcpp::Service<UserInt>::SharedPtr service_;
			rclcpp::TimerBase::SharedPtr timer_;
			rclcpp::Client<RndPos>::SharedPtr RP_client;
	};
	
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::FSM)	
