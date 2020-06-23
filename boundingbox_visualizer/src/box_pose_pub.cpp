#include<ros/ros.h>
#include<boundingbox_visualizer_msgs/myPose.h>


class BoxPosePublisher{
public:
    BoxPosePublisher()
    {
        if(!nh.getParam("/box_pose_pub/mybox_pose_x", poseX))
            throw std::runtime_error("you must set poseX!");
        if(!nh.getParam("/box_pose_pub/mybox_pose_y", poseY))
            throw std::runtime_error("you must set poseY!");
        if(!nh.getParam("/box_pose_pub/mybox_pose_z", poseZ))
            throw std::runtime_error("you must set poseZ!");

        pose_pub = nh.advertise<boundingbox_visualizer_msgs::myPose>("box_pose_topic", 1000);

    }

    ~BoxPosePublisher(){}

    void posePublish(){
        msg.x = poseX;
        msg.y = poseY;
        msg.z = poseZ;

        pose_pub.publish(msg);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;

    boundingbox_visualizer_msgs::myPose msg;

    double poseX, poseY, poseZ;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "box_pose_pub");
    BoxPosePublisher bpp;

    while(ros::ok()) bpp.posePublish();

    return 0;
}
