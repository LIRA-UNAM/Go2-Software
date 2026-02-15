/* ** *****************************************************************
* sport_client.cpp
*
* Implementation of the sport_client interface.
* Original code fetched from:
* https://github.com/unitreerobotics/unitree_ros2/tree/master/example/src/include/common
*
* Author:        Unitree Robotics
* Modifications: Mauricio Matamoros
* License: BSD-3 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
** ** ****************************************************************/
#include "sport_client.h"
#include <nlohmann/json.hpp>


using json = nlohmann::json;
using Request = unitree_api::msg::Request;

const int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
const int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
const int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
const int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
const int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
const int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
const int32_t ROBOT_SPORT_API_ID_EULER = 1007;
const int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
const int32_t ROBOT_SPORT_API_ID_SIT = 1009;
const int32_t ROBOT_SPORT_API_ID_RISESIT = 1010;
const int32_t ROBOT_SPORT_API_ID_SWITCHGAIT = 1011;
const int32_t ROBOT_SPORT_API_ID_TRIGGER = 1012;
const int32_t ROBOT_SPORT_API_ID_BODYHEIGHT = 1013;
const int32_t ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT = 1014;
const int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
const int32_t ROBOT_SPORT_API_ID_HELLO = 1016;
const int32_t ROBOT_SPORT_API_ID_STRETCH = 1017;
const int32_t ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW = 1018;
const int32_t ROBOT_SPORT_API_ID_CONTINUOUSGAIT = 1019;
const int32_t ROBOT_SPORT_API_ID_CONTENT = 1020;
const int32_t ROBOT_SPORT_API_ID_WALLOW = 1021;
const int32_t ROBOT_SPORT_API_ID_DANCE1 = 1022;
const int32_t ROBOT_SPORT_API_ID_DANCE2 = 1023;
const int32_t ROBOT_SPORT_API_ID_GETBODYHEIGHT = 1024;
const int32_t ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT = 1025;
const int32_t ROBOT_SPORT_API_ID_GETSPEEDLEVEL = 1026;
const int32_t ROBOT_SPORT_API_ID_SWITCHJOYSTICK = 1027;
const int32_t ROBOT_SPORT_API_ID_POSE = 1028;
const int32_t ROBOT_SPORT_API_ID_SCRAPE = 1029;
const int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1030;
const int32_t ROBOT_SPORT_API_ID_FRONTJUMP = 1031;
const int32_t ROBOT_SPORT_API_ID_FRONTPOUNCE = 1032;

SportClient::SportClient(rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub):
	pub(pub){}


void SportClient::BalanceStand(){
	SportClient::BalanceStand(req);
	pub->publish(req);
}

void SportClient::BodyHeight(float height){
	SportClient::BodyHeight(req, height);
	pub->publish(req);
}

void SportClient::Content(){
	SportClient::Content(req);
	pub->publish(req);
}

void SportClient::ContinuousGait(bool flag){
	SportClient::ContinuousGait(req, flag);
	pub->publish(req);
}

void SportClient::Damp(){
	SportClient::Damp(req);
	pub->publish(req);
}

void SportClient::Dance1(){
	SportClient::Dance1(req);
	pub->publish(req);
}

void SportClient::Dance2(){
	SportClient::Dance2(req);
	pub->publish(req);
}

void SportClient::Euler(float roll, float pitch, float yaw){
	SportClient::Euler(req, roll, pitch, yaw);
	pub->publish(req);
}

void SportClient::FootRaiseHeight(float height){
	SportClient::FootRaiseHeight(req, height);
	pub->publish(req);
}

void SportClient::FrontFlip(){
	SportClient::FrontFlip(req);
	pub->publish(req);
}

void SportClient::FrontJump(){
	SportClient::FrontJump(req);
	pub->publish(req);
}

void SportClient::FrontPounce(){
	SportClient::FrontPounce(req);
	pub->publish(req);
}

void SportClient::Hello(){
	SportClient::Hello(req);
	pub->publish(req);
}

void SportClient::Move(float vx, float vy, float vyaw){
	SportClient::Move(req, vx, vy, vyaw);
	pub->publish(req);
}

void SportClient::Pose(bool flag){
	SportClient::Pose(req, flag);
	pub->publish(req);
}

void SportClient::RecoveryStand(){
	SportClient::RecoveryStand(req);
	pub->publish(req);
}

void SportClient::RiseSit(){
	SportClient::RiseSit(req);
	pub->publish(req);
}

void SportClient::Scrape(){
	SportClient::Scrape(req);
	pub->publish(req);
}

void SportClient::Sit(){
	SportClient::Sit(req);
	pub->publish(req);
}

void SportClient::SpeedLevel(int level){
	SportClient::SpeedLevel(req, level);
	pub->publish(req);
}

void SportClient::Stretch(){
	SportClient::Stretch(req);
	pub->publish(req);
}

void SportClient::StandDown(){
	SportClient::StandDown(req);
	pub->publish(req);
}

void SportClient::StandUp(){
	SportClient::StandUp(req);
	pub->publish(req);
}

void SportClient::StopMove(){
	SportClient::StopMove(req);
	pub->publish(req);
}

void SportClient::SwitchGait(int d){
	SportClient::SwitchGait(req, d);
	pub->publish(req);
}

void SportClient::SwitchJoystick(bool flag){
	SportClient::SwitchJoystick(req, flag);
	pub->publish(req);
}

void SportClient::TrajectoryFollow(std::vector<PathPoint> &path){
	SportClient::TrajectoryFollow(req, path);
	pub->publish(req);
}

void SportClient::Trigger(){
	SportClient::Trigger(req);
	pub->publish(req);
}

void SportClient::Wallow(){
	SportClient::Wallow(req);
	pub->publish(req);
}


/* ** *****************************************************************
*
* Static methods
*
** ** ****************************************************************/


void SportClient::BalanceStand(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
}

void SportClient::BodyHeight(Request& req, float height){
	json js;
	js["data"] = height;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
}

void SportClient::Content(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTENT;
}

void SportClient::ContinuousGait(Request& req, bool flag){
	json js;
	js["data"] = flag;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_CONTINUOUSGAIT;
}

void SportClient::Damp(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
}

void SportClient::Dance1(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE1;
}

void SportClient::Dance2(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_DANCE2;
}

void SportClient::Euler(Request& req, float roll, float pitch, float yaw){
	json js;
	js["x"] = roll;
	js["y"] = pitch;
	js["z"] = yaw;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
}

void SportClient::FootRaiseHeight(Request& req, float height){
	json js;
	js["data"] = height;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT;
}

void SportClient::FrontFlip(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
}

void SportClient::FrontJump(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTJUMP;
}

void SportClient::FrontPounce(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTPOUNCE;
}

void SportClient::Hello(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_HELLO;
}

void SportClient::Move(Request& req, float vx, float vy, float vyaw){
	json js;
	js["x"] = vx;
	js["y"] = vy;
	js["z"] = vyaw;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
}

void SportClient::Pose(Request& req, bool flag){
	json js;
	js["data"] = flag;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_POSE;
}

void SportClient::RecoveryStand(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
}

void SportClient::RiseSit(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_RISESIT;
}

void SportClient::Scrape(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_SCRAPE;
}

void SportClient::Sit(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_SIT;
}

void SportClient::SpeedLevel(Request& req, int level){
	json js;
	js["data"] = level;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
}

void SportClient::StandDown(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
}

void SportClient::StandUp(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
}

void SportClient::StopMove(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
}

void SportClient::Stretch(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_STRETCH;
}

void SportClient::SwitchGait(Request& req, int d){
	json js;
	js["data"] = d;
	req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
	req.parameter = js.dump();
}

void SportClient::SwitchJoystick(Request& req, bool flag){
	json js;
	js["data"] = flag;
	req.parameter = js.dump();
	req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHJOYSTICK;
}

void SportClient::TrajectoryFollow(Request& req, std::vector<PathPoint> &path){
	json js_path;
	req.header.identity.api_id = ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW;
	for (int i = 0; i < 30; i++)
	{
		json js_point;
		js_point["t_from_start"] = path[i].timeFromStart;
		js_point["x"] = path[i].x;
		js_point["y"] = path[i].y;
		js_point["yaw"] = path[i].yaw;
		js_point["vx"] = path[i].vx;
		js_point["vy"] = path[i].vy;
		js_point["vyaw"] = path[i].vyaw;
		js_path.push_back(js_point);
	}
	req.parameter =js_path.dump();
}

void SportClient::Trigger(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_TRIGGER;
}

void SportClient::Wallow(Request& req){
	req.header.identity.api_id = ROBOT_SPORT_API_ID_WALLOW;
}


