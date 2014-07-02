#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>

#define STABILIZE_MODE 0    //Sets roll pitch yawrate and thrust
#define ALT_HOLD_MODE 1     //Sets roll pitch yawrate and climbrate
#define AUTO_MODE 2         //Sets roll pitch yaw and climbrate
#define ACRO_MODE 3         //Sets rollrate pitchrate yawrate and climbrate
#define FREE_MODE 4         //Sets rollrate pitchrate yawrate and thrust
#define PI (3.14159)
#define MAX_YAW_RATE PI     //Pi radians/sec half turn in a second
#define MAX_RP_ANGLE 20*PI/180       //degrees max lean angle converted to radians
#define MAX_RP_RATE PI
#define HOVER_THRUST 0.3;
#define FREQ 100


//ros::Subscriber joy_sub;
ros::Publisher acc_pub;
ros::Subscriber pose_sub;
ros::Subscriber des_pose_sub;


//ros::Publisher cmd_pub;

//nav_msgs::Odometry uav_cmd;

float LX,LY,LT,RX,RY,RT,DPX,DPY;
bool BA,BB,BX,BY;
int flight_mode;
std::string uav_type;
geometry_msgs::TransformStamped pose, old_pose, des_pose, vel;

geometry_msgs::Vector3 des_accel;

double Kp_xy,Kp_z,Kd_xy,Kd_z,Kpsi;


/*void joyCallBack(const sensor_msgs::Joy &joy_msg){

    LX = joy_msg.axes[0];
    LY = joy_msg.axes[1];
    RX = -joy_msg.axes[3];
    RY = joy_msg.axes[4];

    BA = joy_msg.buttons[0];
    BB = joy_msg.buttons[1];
    BX = joy_msg.buttons[2];
    BY = joy_msg.buttons[3];

    if(BA){flight_mode = STABILIZE_MODE;}
    if(BB){flight_mode = ALT_HOLD_MODE;}
    if(BX){flight_mode = AUTO_MODE;}
    if(BY){flight_mode = ACRO_MODE;}

    switch(flight_mode){

        case STABILIZE_MODE:
        {
            if(LY>0){ //only use positive values
             uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity

            }else{
             uav_cmd.twist.twist.linear.z = 0; //set to zero just to be sure
            }

            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;

        }
        case ALT_HOLD_MODE:
        {
            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;
        }
        case AUTO_MODE:
        {
            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.pose.pose.orientation.z = LX;
            uav_cmd.pose.pose.orientation.x = MAX_RP_ANGLE*RX;
            uav_cmd.pose.pose.orientation.y = MAX_RP_ANGLE*RY;
            break;
        }
        case ACRO_MODE:
        {

            uav_cmd.twist.twist.linear.z = LY; //Thust, use z velocity
            uav_cmd.twist.twist.angular.z = MAX_YAW_RATE*LX;
            uav_cmd.twist.twist.angular.x = MAX_RP_RATE*RX;
            uav_cmd.twist.twist.angular.y = MAX_RP_RATE*RY;
            break;
        }
        default:
        break;
    }

}*/

void poseCallBack(const geometry_msgs::TransformStamped &pose_msg){
    pose = pose_msg;
    vel.transform.translation.x = (pose.transform.translation.x - old_pose.transform.translation.x)/(0.01);
    vel.transform.translation.y = (pose.transform.translation.y - old_pose.transform.translation.y)/(0.01);
    vel.transform.translation.z = (pose.transform.translation.z - old_pose.transform.translation.z)/(0.01);
    old_pose = pose;

}

void desPoseCallBack(const  geometry_msgs::TransformStamped &pose_msg){
    des_pose = pose_msg;
}

void runPositionContorller(){

    des_accel.x = Kp_xy*(des_pose.transform.translation.x - pose.transform.translation.x) +
                  Kd_xy*(0 - vel.transform.translation.x);

    des_accel.y = Kp_xy*(des_pose.transform.translation.y - pose.transform.translation.y) +
                  Kd_xy*(0 - vel.transform.translation.y);

    des_accel.z = Kp_z*(des_pose.transform.translation.z - pose.transform.translation.z) +
                  Kd_z*(0 - vel.transform.translation.z);

}

int main(int argc, char **argv){

    flight_mode = STABILIZE_MODE;
    ros::init(argc, argv, "uav_planner_node");

    ros::NodeHandle nh;


    //cmd_pub = nh.advertise<nav_msgs::Odometry>("uav_cmd",100);
    //oy_sub = nh.subscribe("joy",1000,joyCallBack);
    pose_sub = nh.subscribe("/vicon/CrazyFlie_1/Body",1000,poseCallBack);
    des_pose_sub = nh.subscribe("des_uav_pose",1000,desPoseCallBack);
    acc_pub = nh.advertise<geometry_msgs::Vector3>("des_accel",100);


    if(nh.getParam("uav_type",uav_type)){
        ROS_INFO("Using parameter %s",uav_type.c_str());
    }else{
        uav_type = "generic";
        ROS_WARN("Defaulting to Uav type %s",uav_type.c_str());
    }

    if(nh.getParam("gains/Kp_z",Kp_z)){
        ROS_INFO("Using parameter Kp_xy = %2.3f",Kp_z);
    }else{
        Kp_z = 1;
        ROS_WARN("Defaulting to Kp Gain %2.3f",Kp_z);
    }
    if(nh.getParam("gains/Kp_xy",Kp_xy)){
        ROS_INFO("Using parameter Kp_xy = %2.3f",Kp_xy);
    }else{
        Kp_xy = 1;
        ROS_WARN("Defaulting to Kp Gain %2.3f",Kp_xy);
    }

    if(nh.getParam("gains/Kd_z",Kd_z)){
        ROS_INFO("Using parameter Kp_xy = %2.3f",Kp_z);
    }else{
        Kd_z = 1;
        ROS_WARN("Defaulting to Kd Gain %2.3f",Kp_z);
    }
    if(nh.getParam("gains/Kd_xy",Kd_xy)){
        ROS_INFO("Using parameter Kd_xy = %2.3f",Kd_xy);
    }else{
        Kd_xy = 1;
        ROS_WARN("Defaulting to Kp Gain %2.3f",Kd_xy);
    }

    des_pose.transform.translation.x = 0;
    des_pose.transform.translation.y= 0;
    des_pose.transform.translation.z = 1;

    ros::Rate loop_rate(FREQ);
    while(ros::ok()){
        runPositionContorller();
        acc_pub.publish(des_accel);

        ros::spinOnce();// Allow ROS to check for new ROS Messages

        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    return 0;
}
