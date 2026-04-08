#include "wheeltec_six_arm_pick/car_location_color.h"
std::string  target_direction;
long int error_flag=1,error_count=0;

// 接收小车状态的回调函数
void car_init::car_command_callback(const wheeltec_arm_pick::pick_and_put &msg)
{
  car_state=msg.car_state;
  target_angle=msg.angle;
}

// 获取色块位置的回调函数
// 在函数中做PID计算，将色块定位到可夹取位置
// 当检测到非目标色块时曾执行一次夹爪“摇手”动作（已注释）
void car_init::color_location_callback(const wheeltec_arm_pick::six_arm_position &msg)
{
  static int count=0;

  // 检测到目标色块
  if(msg.correct && car_state==0 && shake_hand==0){
    error_flag=0; // 重置异常标志

    // ---------------- 小车底盘定位控制 -------------------
    if(car_mode=="mini_mec_six_arm") // 四轮全向模式
    {
      if(location_flag==0 && car_state==0){  
        distance_y = msg.angleX;
        distance_x = msg.angleY;
        target_liner_x=PID_control_x(); // 前后方向
        target_liner_y=PID_control_y(); // 左右方向
      }

      if((abs_float(target_liner_x)<0.003)&&(abs_float(target_liner_y)<0.005)&&car_state==0) 
        count++;
      else 
        count=0;

      if(count>100) location_flag=1, count=0; // 定位完成
      if(location_flag==0) target_angular_z=0;
      if(location_flag==1) target_liner_x=0, target_liner_y=0;
    }
    else if((car_mode=="mini_tank_six_arm")||(car_mode=="mini_4wd_six_arm")) // 差速或普通底盘
    {
      if(location_flag==0 && car_state==0){
        distance_x = msg.angleY;
        angular_z = msg.angleX;
        target_liner_x=PID_control_x();
        target_angular_z=PID_control_z();
      }

      if((abs_float(target_liner_x)<0.003)&&(abs_float(target_angular_z)<0.005)&&car_state==0) 
        count++;
      else 
        count=0;

      if(count>=150) location_flag=1, count=0;
      if(location_flag==0) target_liner_y=0;
      if(location_flag==1){
        target_liner_x=0;
        target_angular_z=0;
      }
      else if(location_flag==1 && car_state!=0){
        target_liner_x=0;
      }
    }

    // 发布速度指令
    cmd_vel_publish();
  }

  // 检测到非目标色块，原逻辑是触发夹爪摇动一次用于“打招呼”或提示
  else if(!msg.correct && car_state==0 && location_flag==0 && shake_hand==0){
    /*
    // ❌ 已注释此部分：取消因非目标色块而执行的夹爪摆动动作
    if (msg.color=="yellow"&&shake_hand_done[0]==0) shake_hand=1,shake_hand_done[0]=1;
    if (msg.color=="blue"&&shake_hand_done[1]==0)   shake_hand=1,shake_hand_done[1]=1;
    if (msg.color=="green"&&shake_hand_done[2]==0)  shake_hand=1,shake_hand_done[2]=1;
    */
  }
}

// 获取底盘姿态的回调函数，用于夹取或放置后判断是否自转完成
void car_init::car_pose_callback(const nav_msgs::Odometry &msg)
{
  static float last_target_angle=0,target_position_z=0;
  
  car_position_x=msg.pose.pose.position.x;
  car_position_y=msg.pose.pose.position.y;
  car_position_z=msg.pose.pose.position.z;

  if(last_target_angle!=target_angle) 
    target_position_z=car_position_z + target_angle;

  if(car_state==1) // 夹取色块后
  {
    if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.6,move_flag=1;
    else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.6,move_flag=1;
    else target_angular_z= 0,move_flag=2;

    last_target_angle=target_angle;
    cmd_vel_publish();
  }

  if(car_state==2) // 放置色块后
  {
    if(target_position_z<=(car_position_z-0.1))  target_angular_z= -0.6,move_flag=3;
    else if(target_position_z>=(car_position_z+0.1))  target_angular_z=  0.6,move_flag=3;
    else target_angular_z= 0,move_flag=4;

    last_target_angle=target_angle;  
    cmd_vel_publish();
  }

  last_target_angle=target_angle;
}

// 发布底盘运动控制指令
void car_init::cmd_vel_publish()
{
  geometry_msgs::Twist msg;
  msg.linear.x=target_liner_x;
  msg.linear.y=target_liner_y;
  msg.angular.z=target_angular_z;
  cmd_vel_pub.publish(msg);
}

// 发布机械臂状态命令（用于执行动作）
void car_init::arm_state_publish()
{
  static uint8_t last_move_flag=0,last_location_flag=0;
  std_msgs::String msg;
  msg.data="last_target_angle";

  if (shake_hand)                                                    msg.data="shake_hand";
  else if ((car_state==1)&&(rotate_mode=="holder"))                  msg.data="rotate_put";
  else if((location_flag==1)&&(move_flag<2))                         msg.data="pick";
  else if((move_flag==2||move_flag==3)&&(rotate_mode=="chassis"))    msg.data="put";
  else                                                               msg.data="no_msg";

  arm_state_pub.publish(msg);
}

// PID 控制器：X方向（前后）
float car_init::PID_control_x()
{
    static float last_error=0,output=0;
    pid_x.error=distance_x - color_location_x;
    output = x_d * last_error + x_p * pid_x.error;
    last_error = pid_x.error;
    return output;
}

// PID 控制器：Y方向（左右）
float car_init::PID_control_y()
{
    static float last_error=0,output=0;
    pid_y.error=distance_y - color_location_y;
    output = y_d * last_error + y_p * pid_y.error;
    last_error = pid_y.error;
    return output;
}

// PID 控制器：Z方向（角度旋转）
float car_init::PID_control_z()
{
    static float last_error=0,output=0;
    pid_z.error=angular_z - color_location_y;
    output = z_d * last_error + z_p * pid_z.error;
    last_error = pid_z.error;
    return output;
}

// 绝对值函数
float car_init::abs_float(float input)
{
  if(input<0) return -input;
  else return input;
}

// 主控制循环
void car_init::control()
{
  while(ros::ok())
  {
    // 任务执行完毕，重置状态
    if(move_flag==4 || car_state==3) {
      car_state=0;
      location_flag=0;
      move_flag=0;
      for (int i=0;i<shake_hand_done.size();i++){
        shake_hand_done[i]=0;
      }
    }

    // 夹爪摆动动作完成，重置标志
    if(car_state==4){
      car_state=0;
      location_flag=0;
      move_flag=0;
      shake_hand=0;
      sleep(1);
    }

    // 发布机械臂动作命令
    arm_state_publish();

    // 控制错误保护：连续识别失败停止底盘运动
    if(error_flag==0) error_count=0;
    else if(error_flag==1 && location_flag==0) error_count++;
    error_flag=1;

    if(error_count>10000) {
      target_liner_x=0;
      target_liner_y=0;
      target_angular_z=0;
      cmd_vel_publish();
      error_count=0;
    }

    ros::spinOnce();
  }
}

// 构造函数：初始化参数和话题
car_init::car_init()
{
    ros::NodeHandle param_nh("~");
    param_nh.param("x_p",x_p,0.0);
    param_nh.param("x_d",x_d,0.0);
    param_nh.param("y_p",y_p,0.0);
    param_nh.param("y_d",y_d,0.0);
    param_nh.param("z_p",z_p,0.0);
    param_nh.param("z_d",z_d,0.0);
    param_nh.param("color_location_x",color_location_x,0.0);
    param_nh.param("color_location_y",color_location_y,0.0);
    param_nh.param<std::string>("car_mode",car_mode,"mini_mec_six_arm");
    param_nh.param<std::string>("rotate_mode",rotate_mode,"chassis");

    arm_state_pub=n.advertise<std_msgs::String>("arm_state",10);
    cmd_vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
    car_command_sub=n.subscribe("car_command",10,&car_init::car_command_callback,this);
    color_location_sub=n.subscribe("object_tracker/current_position",10,&car_init::color_location_callback,this);
    if (rotate_mode=="chassis") 
      car_pose_sub=n.subscribe("odom",10,&car_init::car_pose_callback,this);

    target_angle=0;
    car_state=-1;
    location_flag=0;
    move_flag=0;
    shake_hand=0;
    for(int i=0;i<3;i++){
      shake_hand_done.push_back(0);
    }

    ROS_INFO_STREAM("car_location_color_node_init_successful");
}

// 析构函数
car_init::~car_init()
{
    ROS_INFO_STREAM("car_location_color_node_close");
}

// 主函数入口
int main(int argc, char **argv)
{   
    ros::init(argc, argv, "car_location_color");
    car_init car_control;
    car_control.control();
    return 0;
}
