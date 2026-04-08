#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <set>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct NavPose {
    double x, y, z;
    double qx, qy, qz, qw;
};

bool waitForResultWithTimeout(MoveBaseClient& ac, double timeout_sec = 30.0) {
    ros::Rate rate(10);
    double waited = 0.0;
    while (ros::ok() && waited < timeout_sec) {
        if (ac.waitForResult(ros::Duration(0.1))) break;
        waited += 0.1;
        rate.sleep();
    }
    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool sendGoal(const NavPose& pose, MoveBaseClient& ac) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = pose.x;
    goal.target_pose.pose.position.y = pose.y;
    goal.target_pose.pose.position.z = pose.z;
    goal.target_pose.pose.orientation.x = pose.qx;
    goal.target_pose.pose.orientation.y = pose.qy;
    goal.target_pose.pose.orientation.z = pose.qz;
    goal.target_pose.pose.orientation.w = pose.qw;

    ac.sendGoal(goal);
    if (!waitForResultWithTimeout(ac, 100.0)) {
        std::cout << "[WARN] 导航失败或超时" << std::endl;
        return false;
    }
    std::cout << "[INFO] 导航成功！" << std::endl;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "selective_navigation_task");
    ros::NodeHandle nh;

    MoveBaseClient ac("move_base", true);
    std::cout << "[INFO] 等待 move_base 服务器启动..." << std::endl;
    while (ros::ok() && !ac.waitForServer(ros::Duration(5.0))) {
        std::cout << "[INFO] 等待中..." << std::endl;
    }

    // 固定起始点
    NavPose start_pose = { 0.016, 0.271, 0, 0, 0, -0.011, 1.000 };

// 4个目标点
std::vector<NavPose> goal_list = {
    { 3.687,  0.322, 0, 0, 0,  0.703, 0.711 },   // 目标1
    { 2.768, -1.463, 0, 0, 0,  1.000, 0.008 },   // 目标2
    { 1.054, -1.435, 0, 0, 0,  1.000, -0.001 },  // 目标3
    { 1.775, -1.505, 0, 0, 0, -0.018, 1.000 }    // 目标4
};


    std::set<int> visited;
    int current_index = -1;

    // 初始从起点出发
    std::cout << "[STEP] 前往起始点..." << std::endl;
    if (!sendGoal(start_pose, ac)) {
        std::cerr << "[ERROR] 起点导航失败，程序退出。" << std::endl;
        return 1;
    }

    while (ros::ok()) {
        // 展示可选目标
        std::cout << "\n请选择要前往的目标点：" << std::endl;
        for (int i = 0; i < goal_list.size(); ++i) {
            if (visited.count(i) == 0) {
                std::cout << "  " << i + 1 << " - 目标点 " << i + 1 << std::endl;
            }
        }
        std::cout << "  0 - 返回起点并退出程序" << std::endl;
        std::cout << "请输入数字：";

        int choice;
        std::cin >> choice;

        if (choice == 0) {
            std::cout << "[INFO] 返回起点..." << std::endl;
            sendGoal(start_pose, ac);
            break;
        } else if (choice >= 1 && choice <= goal_list.size()) {
            if (visited.count(choice - 1)) {
                std::cout << "[WARN] 目标点 " << choice << " 已访问，请选择其他目标。" << std::endl;
                continue;
            }
            current_index = choice - 1;
            visited.insert(current_index);
            std::cout << "[STEP] 前往目标点 " << choice << " ..." << std::endl;
            if (!sendGoal(goal_list[current_index], ac)) {
                std::cout << "[ERROR] 导航失败，程序退出。" << std::endl;
                break;
            }
        } else {
            std::cout << "[WARN] 输入无效，请重新选择。" << std::endl;
        }

        if (visited.size() == goal_list.size()) {
            std::cout << "[INFO] 所有目标点已访问完，返回起点..." << std::endl;
            sendGoal(start_pose, ac);
            break;
        }
    }

    std::cout << "[INFO] 程序已退出。" << std::endl;
    return 0;
}
