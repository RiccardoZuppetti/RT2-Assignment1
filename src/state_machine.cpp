#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

bool start = false;
bool done = false;

namespace rt2_assignment1{

    class ClientGotoPoint : public rclcpp :: Node {
        public:

            ClientGotoPoint() : Node("go_to_point_client"){
                // client for the /go_to_point server
                client_ = this->create_client<GoalPosition>("/go_to_point");
                while (!client_->wait_for_service(std::chrono::seconds(1))){
                    if (!rclcpp::ok()) {
                        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "waiting for service go_to_point to appear...");
                }
                this->request_=std::make_shared<GoalPosition::Request>();
			    this->response_=std::make_shared<GoalPosition::Response>();	
            }

            void server_clbk(){
                auto result_future = client_->async_send_request(request_);
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS){
                    RCLCPP_ERROR(this->get_logger(), "server callback failed :(");
                }
                this->response_=result_future.get();
            }

            std::shared_ptr<GoalPos::Request> request_;
			std::shared_ptr<GoalPos::Response> response_;

        private:

            rclcpp::Client<GoalPos>::SharedPtr client_;
    };

    class FSM : public rclpp :: Node{
        public:
            
            FSM(const rclcpp::NodeOptions & options) : Node("state_machine", options){
                // server for /user_interface
                service_ = this->create_service<rt2_assignment1::srv::Command>("/user_interface", std::bind(&FSM::handle_service, this, _1, _2, _3));
                // timer callback executed each 500ms
                timer_ =this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FSM::timer_callback, this));
                // client for /position_server
                RP_client=this->create_client<rt2_assignment1::srv::RandomPosition>("/position_server");
            }

        private:

            // function to handle the FSM server
            void handle_service(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<rt2_assignment1::srv::Command::Request> request, const std::shared_ptr<rt2_assignment1::srv::Command::Response> response){
                (void) request_header;
                if(request->command == "start"){
                    start = true;
                    response->ok = true;
                }
                else if(request->command == "stop"){
                    start = false;
                    response->ok = true;
                }
            }

            void timer_callback(){
                auto random_position = std::make_shared<rt2_assignment1::srv::RandomPosition::Request>();
                auto node_go_to_point = std::make_shared<ClientGotoPoint>();
                if(start){
                    // position_server recalled to get a new feasible random position
                    random_position->x_max = 5.0;
					random_position->x_min = -5.0;
					random_position->y_max = 5.0;
					random_position->y_min = -5.0;
                    using ServiceResponseFuture =rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture;
                    auto response_received_callback= [this] (ServiceResponseFuture future){
                        node_go_to_point->request_->x=future.get()->x;
						node_go_to_point->request_->y=future.get()->y;
						node_go_to_point->request_->theta=future.get()->theta;
                        RCLCPP_INFO(this->get_logger(), "Going to position: x:%f y:%f theta:%f",node_go_to_point->request_->x,node_go_to_point->request_->y,node_go_to_point->request_->theta);
                        // the go_to_point server is recalled to move the robot towards the goal position
                        node_go_to_point->server_clbk();
                    };
                    auto future_result = RP_client->async_send_request(random_position, response_received_callback);
                    done = true;
                }
                if(done){
                    done = false;
                }
            }
        	rclcpp::Service<rt2_assignment1::srv::Command>::SharedPtr service_;
			rclcpp::TimerBase::SharedPtr timer_;
			rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedPtr RP_client;
    };
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::FSM)