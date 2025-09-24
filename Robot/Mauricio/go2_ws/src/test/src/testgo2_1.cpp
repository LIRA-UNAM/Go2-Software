/* ** *****************************************************************
* testgo2_1.cpp
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

// Website code
// #include <unitree/robot/go2/sport/sport_client.hpp>

// Adapted code
#include "common/ros2_sport_client.h"

int main(int argc, char **argv){
	/* Website example. Does not build on client machine.
	std::string networkInterface = "lo";
	if (argc < 2){
		printf("no network interface specified. Defaulting to lo\n");
		printf("Usage: %s networkInterface\n", argv[0]);
	}
	else networkInterface = std::string(argv[1]);

	unitree::robot::ChannelFactory::Instance()->Init(0, networkInterface);

	unitree::robot::go2::SportClient sc;
	sc.SetTimeout(10.0f);
	sc.Init();

	printf("Dog sit\n");
	sc.Sit();
	std::this_thread::sleep_for(std::chrono::seconds(5));
	printf("Dog down\n");
	sc.StandDown();
	std::this_thread::sleep_for(std::chrono::seconds(5));
	printf("Dog stand\n");
	sc.BalanceStand();
	std::this_thread::sleep_for(std::chrono::seconds(5));

	printf("Dog move\n");
	sc.SpeedLevel(-1);
	sc.ContinuousGait(true);
	std::this_thread::sleep_for(std::chrono::seconds(5));
	sc.ContinuousGait(false);
	printf("Dog down\n");
	sc.StandDown();
	*/

	/* Adapted example
	*/
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr nodePtr = std::make_shared<rclcpp::Node>("req_sender");
	auto publisher = nodePtr->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
	std::thread spinner( [nodePtr](){rclcpp::spin(nodePtr);} );

	SportClient sc;
	unitree_api::msg::Request req;

        printf("Dog stand\n");
        sc.StandUp(req);
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::seconds(2));

	printf("Dog sit\n");
	sc.Sit(req);
	publisher->publish(req);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(3));

	printf("Dog stand\n");
	sc.StandUp(req);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(3));


	printf("Dog down\n");
	sc.StandDown(req);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(3));

	printf("Dog stand\n");
	sc.StandUp(req);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	printf("Dog move\n");sc.SpeedLevel(req, -1);
	sc.SpeedLevel(req, -1);
        publisher->publish(req);
for(int i=0; i<10; ++i){
        sc.Move(req, 0.2, 0, 0); 
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
for(int i=0; i<10; ++i){
        sc.Move(req, -0.3, 0, 0); 
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

        sc.Move(req, 0, 0, -1.7); 
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        sc.Move(req, 0, 0, 1.7); 
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::seconds(3));

        sc.StopMove(req);
        publisher->publish(req);
        std::this_thread::sleep_for(std::chrono::seconds(3));	/*


	sc.SpeedLevel(req, -1);
	publisher->publish(req);
	sc.ContinuousGait(req, true);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(5));
	sc.ContinuousGait(req, false);
	publisher->publish(req);
	*/


	printf("Dog down\n");
	sc.StandDown(req);
	publisher->publish(req);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	rclcpp::shutdown();
	spinner.join();
return 0;
}
