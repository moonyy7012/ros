/*목표 : Odometry tf를 publish하는 노드를 작성한다.
● 로봇을 2차원상의 2륜구동으로 해석한다. wheel_1,
wheel_3만 해석하는 것으로 한다.
● 이를 위해 먼저 base_link에서 +x축으로 0.1m 지점에
위치한 custom_base를 정의한다. 최종적으로는
custom_base->custom_odom tf를 publish해야 한다.
● input : /gazebo/link_states
● output : /custom_odom(nav_msgs::Odometry) 토픽
발행, /custom_base->/custom_odom tf 발행
*/
#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<cmath>
#include<gazebo_msgs/LinkStates.h>
#include<geometry_msgs/Twist.h>
#include<vector>
#include<iostream>

//#define ptrl ptr->twist[lidx].linear
//#define ptrr ptr->twist[ridx].linear
#define radi 3.14159265359 / 180

using namespace std;

class DiffDriveOdometry{
public:
    DiffDriveOdometry()
	: seq(0)
	, x(0.0)
	, y(0.0)
	, theta(0.0)
    , last(ros::Time::now())
    {
        sub = nh.subscribe("/gazebo/link_states", 100, &DiffDriveOdometry::calcWheelVelocityGazeboCB, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("/custom_odom",100);

        if(!nh.getParam("/mobile_robot_odometry/base_link_id", base_link_id)) throw std::runtime_error("set base_link_id");
        if(!nh.getParam("/mobile_robot_odometry/odom_link_id", odom_link_id)) throw std::runtime_error("set odom_link_id");
        if(!nh.getParam("/mobile_robot_odometry/leftwheel_linkname", wheel_1_id)) throw std::runtime_error("set wheel_1_id");
        if(!nh.getParam("/mobile_robot_odometry/rightwheel_linkname", wheel_3_id)) throw std::runtime_error("set wheel_3_id");
        if(!nh.getParam("/mobile_robot_odometry/separation_length", separation_length)) throw std::runtime_error("set separation_length");
    }

    void calcWheelVelocityGazeboCB(const gazebo_msgs::LinkStates::ConstPtr& ptr){
     /*      int lidx = -1, ridx = -1, i=0;
            for(int i=0;i<ptr->name.size();i++){
                if(ptr->name[i] == wheel_1_id){lidx = i;}
                else if(ptr->name[i] == wheel_3_id){ridx = i;}
            }
            if(lidx == -1 || ridx == -1) return ;
            Vl = sqrt(pow(ptrl.x, 2) + pow(ptrl.y, 2) + pow(ptrl.z, 2));
            Vr = sqrt(pow(ptrr.x, 2) + pow(ptrr.y, 2) + pow(ptrr.z, 2));
            V = (Vl + Vr) / 2;  
    */
        int wheel1_count = 0;
        int wheel3_count = 0;
        double wheel1_x, wheel1_y, wheel1_z, wheel3_x, wheel3_y, wheel3_z;

        while(ptr->name[wheel1_count] != wheel_1_id) wheel1_count++;
        while(ptr->name[wheel3_count] != wheel_3_id) wheel3_count++;

        wheel1_x = ptr->twist[wheel1_count].linear.x;
        wheel1_y = ptr->twist[wheel1_count].linear.y;
        wheel1_z = ptr->twist[wheel1_count].linear.z;
        wheel3_x = ptr->twist[wheel3_count].linear.x;
        wheel3_y = ptr->twist[wheel3_count].linear.y;
        wheel3_z = ptr->twist[wheel3_count].linear.z;

        Vl = sqrt((wheel1_x * wheel1_x) + (wheel1_y * wheel1_y) + (wheel1_z * wheel1_z));
        Vr = sqrt((wheel3_x * wheel3_x) + (wheel3_y * wheel3_y) + (wheel3_z * wheel3_z));
        V = (Vr + Vl) / 2.0;
        
        
    }

    void boradcastTransform(){

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x,y,0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom_link_id, base_link_id));
        //yaml 파일로 받아온 link를 파라미터로 받아와서 사용해야합니다.
    }

    void pubTF(){
        nav_msgs::Odometry odom;
        ros::Time cur = ros::Time::now();
		double dt = (cur - last).toSec();
/*
        theta_dot = (Vr - Vl) / separation_length;
        theta += theta_dot * dt;
        
        x_dot = V * cos(theta);
        y_dot = V * sin(theta);

        x += x_dot * dt;
        y += y_dot * dt;  

		last = cur;  */

        theta += ((Vr - Vl) / separation_length)*dt;
        x += V * cos(theta) * dt;
        y += V * sin(theta) * dt; 
        last = cur;
        x_dot = V * cos(theta);
        y_dot = V * sin(theta);
        theta_dot = (Vr - Vl) / separation_length; 

        odom.header.seq             = seq++;
        odom.header.stamp           = cur;
// same as  br.sendTransform
        odom.header.frame_id        = odom_link_id;
        odom.child_frame_id         = base_link_id; 

        odom.pose.pose.position.x   = x;
        odom.pose.pose.position.y   = y;
        odom.pose.pose.position.z   = 0;
        odom.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x   = x_dot;
        odom.twist.twist.linear.y   = y_dot;
        odom.twist.twist.angular.z  = theta_dot;


        odomPub.publish(odom);
        boradcastTransform();
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher odomPub;
    ros::Time last;

    string base_link_id, odom_link_id, wheel_1_id, wheel_3_id;
    double separation_length;
    double x_dot, y_dot, theta_dot;
    double x, y, theta;
    double Vl, Vr, V;
    int seq;
    tf::TransformBroadcaster br;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mobile_robot_odometry");
    DiffDriveOdometry Odom;
    ros::Rate loop_rate(100); // dt is always 0.01s

    while(ros::ok()){
        ros::spinOnce();
        Odom.pubTF();
        loop_rate.sleep();
    }
    return 0;
}
