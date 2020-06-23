#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

using namespace std;


class CustomJointStatePublisher{
public:
    CustomJointStatePublisher()
    :n("custom_joint_state_publisher")
    {
        n.getParam("joint_names", joint_names);
        n.getParam("joint_positions", joint_positions);
        joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);

    }

    ~CustomJointStatePublisher(){}

    void pub(){
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        for(int i=0;i<joint_names.size();i++){
            joint_state.name.push_back(joint_names[i]);
            joint_state.position.push_back(joint_positions[i]);
        }
        joint_pub.publish(joint_state);
    }

private:
    ros::NodeHandle n;
    ros::Publisher joint_pub;



    vector<string> joint_names;
    vector<int> joint_positions;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_joint_state_publisher");
    CustomJointStatePublisher c;

    ros::Rate loop_rate(30);
    
    while(ros::ok()){
        c.pub();
        loop_rate.sleep();
    }
/*
    c.pub();
    ros::spin();*/
    return 0;
}
