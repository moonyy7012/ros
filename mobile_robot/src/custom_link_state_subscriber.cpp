#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Twist.h>
#include <vector>

#define ptrl ptr->twist[lidx].linear
#define ptrr ptr->twist[ridx].linear
#define radi 3.14159265359 / 180

class CustomGazeboSubscriber{
public:
    CustomGazeboSubscriber()
    : private_nh("mobile_robot_odometry")
    , left_vel(0.0)
    , right_vel(0.0)
    , base_vel(0.0)
    {
        gazebo_sub = nh.subscribe("/gazebo/link_states", 100, &CustomGazeboSubscriber::callbackGazeboLinkStates, this);

        if(!private_nh.getParam("base_link_id",base_link_id)) throw std::runtime_error("set base_link_id");
        if(!private_nh.getParam("odom_link_id",odom_link_id)) throw std::runtime_error("set odom_link_id");
        if(!private_nh.getParam("leftwheel_linkname",wheel_1_id)) throw std::runtime_error("set wheel_1_id");
        if(!private_nh.getParam("rightwheel_linkname",wheel_3_id)) throw std::runtime_error("set wheel_3_id");

    }

    void callbackGazeboLinkStates(const gazebo_msgs::LinkStates::ConstPtr& ptr){
            int lidx = -1, ridx = -1, i=0;
            for(int i=0;i<ptr->name.size();i++){
                if(ptr->name[i] == wheel_1_id){lidx = i;}
                else if(ptr->name[i] == wheel_3_id){ridx = i;}
            }
            if(lidx == -1 || ridx == -1) return ;
            left_vel = sqrt(pow(ptrl.x, 2) + pow(ptrl.y, 2) + pow(ptrl.z, 2));
            right_vel = sqrt(pow(ptrr.x, 2) + pow(ptrr.y, 2) + pow(ptrr.z, 2));
            base_vel = (left_vel + right_vel) / 2;
    }

    void get_v(){
        ROS_INFO("----------------------------");
        ROS_INFO("base_link_id   = %s", base_link_id.c_str());
        ROS_INFO("odom_link_id   = %s", odom_link_id.c_str());
        ROS_INFO("wheel_1_id     = %s", wheel_1_id.c_str());
        ROS_INFO("wheel_3_id     = %s", wheel_3_id.c_str());
        ROS_INFO("left_velocity  = %lf", left_vel);
        ROS_INFO("right_velocity = %lf", right_vel);
        ROS_INFO("base_velocity  = %lf", base_vel);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Subscriber gazebo_sub;
    
    std::string wheel_1_id;
    std::string wheel_3_id;
    std::string base_link_id;
    std::string odom_link_id;

    double left_vel;
    double right_vel;
    double base_vel;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "custom_link_state_subscriber");
    CustomGazeboSubscriber CGS;
    ros::Rate loop_rate(30);
   
    while(ros::ok()){
        ros::spinOnce();
        CGS.get_v();
        loop_rate.sleep();
    }
    return 0;
}
