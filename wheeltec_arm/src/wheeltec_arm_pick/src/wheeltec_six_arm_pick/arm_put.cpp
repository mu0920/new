#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

// 放置动作
void arm_put()
{
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");

    arm.setGoalJointTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1);
    arm.setMaxVelocityScalingFactor(1);

    arm.setNamedTarget("arm_clamp");   arm.move();  sleep(1); // 降低机械臂
    hand.setNamedTarget("hand_open");  hand.move(); sleep(1); // 张开夹爪
    arm.setNamedTarget("arm_uplift");  arm.move();            // 抬起机械臂
    ros::param::set("/put_success", true);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_put_only");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化夹爪和机械臂
    moveit::planning_interface::MoveGroupInterface arm("arm");
    moveit::planning_interface::MoveGroupInterface hand("hand");

    arm.setGoalJointTolerance(0.01);
    arm.setMaxVelocityScalingFactor(1);

 
    // 执行放置动作
    arm_put();

    ros::shutdown();
    return 0;
}
