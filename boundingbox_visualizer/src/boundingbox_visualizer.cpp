#include<ros/ros.h>
#include<jsk_recognition_msgs/BoundingBoxArray.h>
#include<jsk_recognition_msgs/BoundingBox.h>
#include<boundingbox_visualizer_msgs/myPose.h>

struct BoxPose{
    double x;
    double y;
    double z;
};

class BoxVisualizer{
public:
    BoxVisualizer()
    : seq(0)
    , subseq(0)
    {
        if(!nh.getParam("/boundingbox_visualizer/totalboxnum", totalboxnum))
            throw std::runtime_error("set totalboxnum");

            box_visual_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("box_visualizer", 1000);
            pose_sub = nh.subscribe("box_pose_topic", 1000, &BoxVisualizer::callbackBoxPose, this);

            box_arr_msg.boxes.clear();
            box_arr_msg.boxes.resize(0);
    }

    ~BoxVisualizer(){}

    void callbackBoxPose(const boundingbox_visualizer_msgs::myPose::ConstPtr& msg){
        pose_info.x = msg->x;
        pose_info.y = msg->y;
        pose_info.z = msg->z;
    }

    
    void publishBox(){
        box_arr_msg.header.frame_id = "senrogong_box";
        box_arr_msg.header.seq = seq++;
        box_arr_msg.header.stamp = ros::Time::now();

        for(int i = 0; i < totalboxnum; i++){
            box_msg.header.frame_id = "senrogong_box";
            box_msg.header.seq = subseq++;
            box_msg.header.stamp = ros::Time::now();

            box_msg.pose.position.x = pose_info.x + i;
            box_msg.pose.position.y = pose_info.y;
            box_msg.pose.position.z = pose_info.z;
            box_msg.pose.orientation.x = 0.0;
            box_msg.pose.orientation.y = 0.0;
            box_msg.pose.orientation.z = 0.0;
            box_msg.pose.orientation.w = 0.0;
            box_msg.dimensions.x = 0.5;
            box_msg.dimensions.y = 0.5;
            box_msg.dimensions.z = 0.5;

            box_arr_msg.boxes.emplace_back(box_msg);
        }

        box_visual_pub.publish(box_arr_msg);
        box_arr_msg.boxes.clear();
        box_arr_msg.boxes.resize(0);
    }

    private:
        ros::NodeHandle nh;
        ros::Publisher box_visual_pub;
        ros::Subscriber pose_sub;

        jsk_recognition_msgs::BoundingBox box_msg;
        jsk_recognition_msgs::BoundingBoxArray box_arr_msg;

        BoxPose pose_info;

        unsigned int seq, subseq;
        int totalboxnum;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "boundingbox_visualizer");
    BoxVisualizer BV;
    ROS_INFO("TESTEST");
    while(ros::ok()){
        ros::spinOnce();
        BV.publishBox();
    }
    
    return 0;
}