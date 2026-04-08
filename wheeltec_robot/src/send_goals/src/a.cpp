#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_once");
    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // 设置目标点位姿（导航目标）
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 12.242;
    goal.target_pose.pose.position.y = 2.749;
    goal.target_pose.pose.orientation.z = -0.679;
    goal.target_pose.pose.orientation.w = 0.735;

    ROS_INFO("Sending navigation goal...");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Successfully reached the target!");
        ros::param::set("/reach_pick_point", true);
    } else {
        ROS_WARN("Failed to reach the target.");
    }

    return 0;
}
