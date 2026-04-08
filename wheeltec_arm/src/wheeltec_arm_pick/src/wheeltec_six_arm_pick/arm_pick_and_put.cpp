#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdbool.h>

// ROS标准消息头
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// 自定义消息
#include <wheeltec_arm_pick/pick_and_put.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "wheeltec_six_arm_pick/arm_pick_and_put.h"

// 全局变量定义
uint8_t car_state = 0;       // 底盘状态
int arm_done = 1;            // 机械臂动作完成标志，1为待执行，0为执行中
ros::Publisher joint_states_pub;  // 关节状态发布器

std::string arm_state = "none";  // 当前机械臂状态

// 机械臂状态话题回调函数
void arm_state_callback(const std_msgs::String &state)
{
    static std::string last_arm_state;
    arm_state = state.data;
    if (last_arm_state != arm_state)
        arm_done = 1;  // 状态切换，准备执行新动作
    last_arm_state = arm_state;
}

// 夹取动作函数声明
void arm_pick();
void arm_put();

int main(int argc, char **argv)
{ 
    wheeltec_arm_pick::pick_and_put msg;

    ros::init(argc, argv, "arm_pick_and_put");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化MoveIt接口
    moveit::planning_interface::MoveGroupInterface arm("arm");   // 机械臂控制组
    moveit::planning_interface::MoveGroupInterface hand("hand"); // 夹爪控制组

    arm.setGoalJointTolerance(0.01);               // 机械臂目标容差
    arm.setMaxVelocityScalingFactor(1);            // 最大速度比例

    // 初始动作：张开夹爪，抬起机械臂
    hand.setNamedTarget("hand_open");
    hand.move(); sleep(1);

    arm.setNamedTarget("arm_uplift");
    arm.move(); sleep(1);

    // 发布器和订阅器定义
    ros::Publisher car_command_pub = n.advertise<wheeltec_arm_pick::pick_and_put>("car_command", 10); // 底盘控制话题
    ros::Subscriber arm_state_sub = n.subscribe("arm_state", 10, arm_state_callback); // 机械臂状态订阅

    car_state = 0;
    arm_done = 1;

    while (ros::ok())
    {
        // 根据接收到的机械臂状态，执行对应动作（只执行一次）
        if (arm_done == 1 && arm_state == "pick") {
            arm_pick();
            car_state = 1;
            arm_done = 0;
        }
        else if (arm_done == 1 && arm_state == "put") {
            arm_put();
            car_state = 2;
            arm_done = 0;
        }
        else if (arm_done == 1 && arm_state == "no_msg") {
            car_state = 0;
            arm_done = 0;
        }

        // 根据car_state设置消息
        if (car_state == 0) {
            msg.car_state = 0; 
            msg.angle = 0;         // 空闲
        }
        if (car_state == 1) {
            msg.car_state = 1; 
            msg.angle = 1.57;      // 抓取后左转
        }
        if (car_state == 2) {
            msg.car_state = 2; 
            msg.angle = -1.57;     // 放置后右转
        }

        car_command_pub.publish(msg); // 发布指令给底盘节点
        ros::spinOnce();
        usleep(10000); // 10ms延迟，避免CPU过高占用
    }

    ros::shutdown(); 
    return 0;
}

// 完整的夹取动作流程
void arm_pick()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");

    arm.setGoalJointTolerance(0.01);
    arm.setMaxVelocityScalingFactor(0.1); // 动作慢一些

    ROS_INFO("Executing pick sequence...");
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1); // 降低机械臂
    hand.setNamedTarget("hand_close"); hand.move(); sleep(1); // 闭合夹爪
    arm.setNamedTarget("arm_uplift");  arm.move(); sleep(2);  // 抬起机械臂
    ROS_INFO("Pick operation completed");
    
    // 设置成功标志
    ros::param::set("/pick_success", true);
}

// 完整的放置动作流程
void arm_put()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");

    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);

    ROS_INFO("Executing put sequence...");
    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1); // 降低机械臂
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1); // 张开夹爪
    arm.setNamedTarget("arm_uplift");  arm.move();            // 抬起机械臂
    ROS_INFO("Put operation completed");
}