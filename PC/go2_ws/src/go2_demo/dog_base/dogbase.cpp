/* ** *****************************************************************
* dogbase.cpp
*
* Forwards a Twist to the sport_client and broadcasts the 
* transform as odometry
*
* Author:  Mauricio Matamoros
* License: MIT
** ** ****************************************************************/
#include <regex>
#include <chrono>
#include <memory>
#include <thread>
#include <csignal>
#include <cstdint>
#include <algorithm>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sport_client/sport_client.h"
// #include <unitree_api/msg/response.hpp>

using String           = std_msgs::msg::String;
using StringPtr        = std::shared_ptr<String>;
using Request          = unitree_api::msg::Request;
// using Response         = unitree_api::msg::Response;
// using ResponsePtr      = std::shared_ptr<Response>;
using Twist            = geometry_msgs::msg::Twist;
using TwistPtr         = std::shared_ptr<Twist>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using SportClientPtr   = std::shared_ptr<SportClient>;


int main(int argc, char **argv);
void signal_handler(int signal);

enum DogStatus{
	StandReady,
	Sitting,
	LayingDown,
	Damped,
	Dancing,
};

class DogBaseNode : public rclcpp::Node{
	private:
		SportClientPtr sc;
		rclcpp::Publisher<Request>::SharedPtr pub;
		rclcpp::Subscription<Twist>::SharedPtr sub_cmd_vel;
		rclcpp::Subscription<String>::SharedPtr sub_go2_trick;
		// rclcpp::Subscription<Response>::SharedPtr sub_response;
		std::unique_ptr<tf2_ros::TransformBroadcaster> tbc;
		DogStatus status;
		float bodyHeight;

	public:
		DogBaseNode();
		void dance(bool d2);
		void layDown();
		void standReady();
		void sitDown();
		void setBodyHeight(float height);

	private:
		void initGo2();
		void handleTwist(const TwistPtr msg);
		void handleTrick(const StringPtr msg);
		// void handleResponse(const ResponsePtr msg);
};


std::shared_ptr<DogBaseNode> node;


int main(int argc, char **argv){
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);
	rclcpp::init(argc, argv);
	rclcpp::spin(node = std::make_shared<DogBaseNode>());
	rclcpp::shutdown();
	return 0;
}

void signal_handler(int signal){
	if(!node) return;
	node->layDown();
}



DogBaseNode::DogBaseNode():
	Node("dog_base_node"), bodyHeight(0){
	tbc = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	pub = this->create_publisher<Request>("/api/sport/request", 5);
	// sub_response = this->create_subscription<Response>("/api/sport/response", 10,
	// 	std::bind(&DogBaseNode::handleResponse, this, std::placeholders::_1)
	// );
	sub_cmd_vel = this->create_subscription<Twist>("/cmd_vel", 5,
		std::bind(&DogBaseNode::handleTwist, this, std::placeholders::_1)
	);
	sub_go2_trick = this->create_subscription<String>("/go2_trick", 1,
		std::bind(&DogBaseNode::handleTrick, this, std::placeholders::_1)
	);
	sc  = std::make_shared<SportClient>(pub);
	initGo2();
}


void DogBaseNode::initGo2(){
	sc->ContinuousGait(false);
	RCLCPP_INFO(this->get_logger(), "Dogbase node running. Standing up...");
	sc->RiseSit();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	sc->StandUp();
	for(int i = 0; i < 3; i++){
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		sc->BalanceStand();
	}
	setBodyHeight(bodyHeight);
	// -0.18~0.03
	RCLCPP_INFO(this->get_logger(), "Dogbase ready");
	status = DogStatus::StandReady;
}


void DogBaseNode::standReady(){
	switch(status){
		case DogStatus::StandReady: return;

		case DogStatus::Sitting:
			sc->RiseSit();
			break;

		case DogStatus::LayingDown:
			sc->StandUp();
			break;

		case DogStatus::Damped:
			sc->StandDown();
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			sc->StandUp();
			break;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(700));
	sc->BalanceStand();
	status = DogStatus::StandReady;
}


void DogBaseNode::layDown(){
	if(status == DogStatus::LayingDown) return;
	standReady();
	std::this_thread::sleep_for(std::chrono::milliseconds(700));
	sc->StandDown();
	status = DogStatus::LayingDown;
}


void DogBaseNode::sitDown(){
	if(status == DogStatus::Sitting) return;
	standReady();
	std::this_thread::sleep_for(std::chrono::milliseconds(700));
	sc->Sit();
	status = DogStatus::Sitting;
}


void DogBaseNode::dance(bool d2){
	if(status == DogStatus::Dancing) return;
	standReady();
	std::this_thread::sleep_for(std::chrono::milliseconds(700));
	if(d2) sc->Dance2();
	else   sc->Dance1();
	std::make_unique<std::thread>( [this](){
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		status == DogStatus::StandReady;
	});
}


void DogBaseNode::setBodyHeight(float height){
	bodyHeight = std::max<float>(height, -0.06);
	bodyHeight = std::min<float>(bodyHeight, 0.03);
	sc->BodyHeight(bodyHeight);
}



void DogBaseNode::handleTrick(const StringPtr msg){
	static std::regex rxTrick("(\\w+)\\s*(-?\\d+(\\.\\d+)?)?");
	std::smatch match;

	// RCLCPP_INFO(this->get_logger(), "/trick: %s", trick.c_str() );
	if(!std::regex_search(msg->data, match, rxTrick)) return;
	std::string trick = match[1];

	if((trick == "standup") || (trick == "stand"))
		standReady();
	if(trick == "sit")   sitDown();
	if(trick == "lay")   layDown();
	if(trick == "dance1")  dance(false);
	if(trick == "dance2")  dance(true);
	// if(trick == "damp")  sc->Damp();
	// if(trick == "rise")  sc->RiseSit();
	if(trick == "bodyUp") setBodyHeight(bodyHeight + 0.005);
	if(trick == "bodyDown") setBodyHeight(bodyHeight - 0.005);
	if((trick == "bodyHeight") && (match.size() > 2))
		setBodyHeight(std::stof(match[2]));
}

void DogBaseNode::handleTwist(const TwistPtr msg){
	// RCLCPP_INFO(this->get_logger(),
	// 	"Handle twist: (%0.2f,%0.2f,%0.2f) (%0.2f,%0.2f,%0.2f)",
	// 	msg->linear.x,  msg->linear.y,  msg->linear.z,
	// 	msg->angular.x, msg->angular.y, msg->angular.z
	// );
	TransformStamped t;

	// Initialize tf variables
	t.header.stamp = this->get_clock()->now();
	t.header.frame_id = "odom";
	t.child_frame_id = "base_link";

	// Go2 moves on ground: x is front-back, y is left-right
	// Here we forward the Twist data to the Go2 and tranform.
	sc->Move(msg->linear.x, msg->linear.y, msg->angular.z);
	t.transform.translation.x+= msg->linear.x;
	t.transform.translation.y+= msg->linear.y;
	t.transform.translation.z = 0.0;
	//t.transform.translation = msg->linear;

	// Rotation is only on theta (vector z)
	tf2::Quaternion q;
	q.setRPY(0, 0, msg->angular.z);
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();

	// Broadcast the transform
	tbc->sendTransform(t);
}


// void DogBaseNode::handleResponse(const ResponsePtr msg){
// 	RCLCPP_INFO(this->get_logger(), "Response: %s", msg->data.c_str() );
// }
