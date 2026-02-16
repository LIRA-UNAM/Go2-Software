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
#include <nav_msgs/msg/odometry.hpp>

#include "sport_client/sport_client.h"
// #include <unitree_api/msg/response.hpp>

using String           = std_msgs::msg::String;
using StringPtr        = std::shared_ptr<String>;
using Request          = unitree_api::msg::Request;
using Odometry         = nav_msgs::msg::Odometry;
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
		rclcpp::Publisher<Odometry>::SharedPtr odom_pub_{nullptr};
		DogStatus status;
		float bodyHeight;
		bool publish_tf_{false};  // false: PC publica TF (odom_publisher o odom_filter)
		float odom_x_{0}, odom_y_{0}, odom_yaw_{0};  // integracion para odom topic

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
	this->declare_parameter("publish_tf", false);
	rclcpp::Parameter p;
	if (this->get_parameter("publish_tf", p)) {
		if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
			publish_tf_ = p.as_bool();
		} else if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
			std::string s = p.as_string();
			publish_tf_ = (s == "true" || s == "1");
		}
	}
	tbc = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	if (!publish_tf_) {
		odom_pub_ = this->create_publisher<Odometry>("/utlidar/robot_odom", 10);
	}
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
	auto now = this->get_clock()->now();
	// Integracion simple (dead reckoning desde cmd_vel)
	odom_x_   += msg->linear.x * 0.1;
	odom_y_   += msg->linear.y * 0.1;
	odom_yaw_ += msg->angular.z * 0.1;

	sc->Move(msg->linear.x, msg->linear.y, msg->angular.z);

	tf2::Quaternion q;
	q.setRPY(0, 0, odom_yaw_);

	if (publish_tf_) {
		TransformStamped t;
		t.header.stamp = now;
		t.header.frame_id = "odom";
		t.child_frame_id = "base_link";
		t.transform.translation.x = odom_x_;
		t.transform.translation.y = odom_y_;
		t.transform.translation.z = 0.0;
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
		tbc->sendTransform(t);
	} else if (odom_pub_) {
		Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.pose.pose.position.x = odom_x_;
		odom.pose.pose.position.y = odom_y_;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation.x = q.x();
		odom.pose.pose.orientation.y = q.y();
		odom.pose.pose.orientation.z = q.z();
		odom.pose.pose.orientation.w = q.w();
		odom.twist.twist.linear.x = msg->linear.x;
		odom.twist.twist.linear.y = msg->linear.y;
		odom.twist.twist.angular.z = msg->angular.z;
		odom_pub_->publish(odom);
	}
}


// void DogBaseNode::handleResponse(const ResponsePtr msg){
// 	RCLCPP_INFO(this->get_logger(), "Response: %s", msg->data.c_str() );
// }
