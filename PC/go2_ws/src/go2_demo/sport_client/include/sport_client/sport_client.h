/* ** *****************************************************************
* sport_client.cpp
*
* Interface for the /sportmodestate topic via /api/sport/request.
* Original code fetched from:
* https://github.com/unitreerobotics/unitree_ros2/tree/master/example/src/include/common
*
* Author:        Unitree Robotics
* Modifications: Mauricio Matamoros
* License: BSD-3 2016-2024 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
** ** ****************************************************************/
#pragma once
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/request.hpp>


#pragma pack(1)
typedef struct
{
    float timeFromStart;
    float x;
    float y;
    float yaw;
    float vx;
    float vy;
    float vyaw;
} PathPoint;

class SportClient
{
protected:
    unitree_api::msg::Request req;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub;

public:
    SportClient(rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr pub);

private:
    SportClient(const SportClient& other)      = delete;
    SportClient& operator=(const SportClient&) = delete;

public:
    void BalanceStand();
    void BodyHeight(float height);
    void Content();
    void ContinuousGait(bool flag);
    void Damp();
    void Dance1();
    void Dance2();
    void Euler(float roll, float pitch, float yaw);
    void FootRaiseHeight(float height);
    void FrontFlip();
    void FrontJump();
    void FrontPounce();
    void Hello();
    void Move(float vx, float vy, float vyaw);
    void Pose(bool flag);
    void RecoveryStand();
    void RiseSit();
    void Scrape();
    void Sit();
    void SpeedLevel(int level);
    void StandDown();
    void StandUp();
    void StopMove();
    void Stretch();
    void SwitchGait(int d);
    void SwitchJoystick(bool flag);
    void TrajectoryFollow(std::vector<PathPoint> &path);
    void Trigger();
    void Wallow();

public:
    static void BalanceStand(unitree_api::msg::Request& req);
    static void BodyHeight(unitree_api::msg::Request& req, float height);
    static void Content(unitree_api::msg::Request& req);
    static void ContinuousGait(unitree_api::msg::Request& req, bool flag);
    static void Damp(unitree_api::msg::Request& req);
    static void Dance1(unitree_api::msg::Request& req);
    static void Dance2(unitree_api::msg::Request& req);
    static void Euler(unitree_api::msg::Request& req, float roll, float pitch, float yaw);
    static void FootRaiseHeight(unitree_api::msg::Request& req, float height);
    static void FrontFlip(unitree_api::msg::Request& req);
    static void FrontJump(unitree_api::msg::Request& req);
    static void FrontPounce(unitree_api::msg::Request& req);
    static void Hello(unitree_api::msg::Request& req);
    static void Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw);
    static void Pose(unitree_api::msg::Request& req, bool flag);
    static void RecoveryStand(unitree_api::msg::Request& req);
    static void RiseSit(unitree_api::msg::Request& req);
    static void Scrape(unitree_api::msg::Request& req);
    static void Sit(unitree_api::msg::Request& req);
    static void SpeedLevel(unitree_api::msg::Request& req, int level);
    static void StandDown(unitree_api::msg::Request& req);
    static void StandUp(unitree_api::msg::Request& req);
    static void StopMove(unitree_api::msg::Request& req);
    static void Stretch(unitree_api::msg::Request& req);
    static void SwitchGait(unitree_api::msg::Request& req, int d);
    static void SwitchJoystick(unitree_api::msg::Request& req, bool flag);
    static void TrajectoryFollow(unitree_api::msg::Request& req, std::vector<PathPoint> &path);
    static void Trigger(unitree_api::msg::Request& req);
    static void Wallow(unitree_api::msg::Request& req);
};
