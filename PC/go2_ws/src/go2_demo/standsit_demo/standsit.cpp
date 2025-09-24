/* ** *****************************************************************
* standsit.cpp
*
* First test attempt to control the unitree Go2 using the SportClient
*
* Author:  Mauricio Matamoros
* License: MIT
** ** ****************************************************************/
#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "sport_client/sport_client.h"

using Request = unitree_api::msg::Request;

rclcpp::Publisher<Request>::SharedPtr publisher;
void task();

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr nodePtr = std::make_shared<rclcpp::Node>("req_sender");
	publisher = nodePtr->create_publisher<Request>("/api/sport/request", 10);

	std::thread task_thread( task );
	while(rclcpp::ok()) rclcpp::spin_some(nodePtr);
	task_thread.join();
	return 0;
}

void task(){
	SportClient sc(publisher);

	printf("Dog stand\n");
	sc.StandUp();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	printf("Dog sit\n");
	sc.Sit();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	printf("Dog down\n");
	sc.StandDown();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	printf("Dog stand\n");
	sc.StandUp();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	if( !rclcpp::ok() ) return;

	printf("Dog down\n");
	sc.StandDown();
	std::this_thread::sleep_for(std::chrono::seconds(2));

	rclcpp::shutdown();
}
