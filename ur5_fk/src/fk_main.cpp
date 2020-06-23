//first, launch ur5_description and run this or run rviz.
#include<ros/ros.h>
#include<iostream>
#include<eigen3/Eigen/Dense>
#include<vector>
#include<sensor_msgs/JointState.h>

#define N_JOINT 6

using namespace std;
using namespace Eigen;

static constexpr double a_dh[N_JOINT] = {0.00000, -0.42500, -0.39225, 0.00000, 0.00000, 0.00000};
static constexpr double d_dh[N_JOINT] = {0.089159, 0.00000, 0.00000, 0.10915, 0.09465, 0.0823};
static constexpr double alpha_dh[N_JOINT] = {1.570796327, 0, 0, 1.570796327, -1.570796327, 0};

class UR5{
public:
    UR5()
    :nh("")
    {
        sub = nh.subscribe("/joint_states", 1000, &UR5::callbackjointstates, this);
    }

    void callbackjointstates(const sensor_msgs::JointState::ConstPtr& ptr){
        m << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
            0, 0, 0, 1; 
    
        for(int i=0; i<6; i++){
            m *= (RotZ(ptr->position[i]) * TransZ(d_dh[i]) * TransX(a_dh[i]) * RotX(alpha_dh[i]));
        }
    
        cout << "result tf matrix -" << endl << m << endl<<endl;
    }
//Joint angle : the angle of rotation from the Xi-1 axis to the Xi axis about the Zi-1 axis. It is the joint variable if joint i is rotary.
    Matrix4d RotZ(double rad){
        Matrix4d m1;
        m1 <<   cos(rad), -sin(rad), 0, 0,
                sin(rad), cos(rad), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return m1;
    }
//Joint distance : the distance from the origin of the (i-1) coordinate system to the intersection of the Zi-1 axis and the Xi axis along the Zi-1 axis. It is the joint variable if joint i is prismatic.
    Matrix4d TransZ(double d){
        Matrix4d m2;
        m2 <<   1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, d,
                0, 0, 0, 1;
        return m2;
    }
//Link length : the distance from the intersection of the Zi-1 axis and the Xi axis to the origin of the (i+1) coordinate system along the Xi axis.
    Matrix4d TransX(double a){
        Matrix4d m3;
        m3 <<   1, 0, 0, a,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return m3;
    }
//Link twist angle : the angle of rotation from the Zi-1 axis to the Zi axis about the Xi axis.
    Matrix4d RotX(double rad){
        Matrix4d m4;
        m4 <<   1, 0, 0, 0,
                0, cos(rad), -sin(rad), 0,
                0, sin(rad), cos(rad), 0,
                0, 0, 0, 1;
        return m4;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    Matrix4d m;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fk_main");
    UR5 genousMatrix;
    ros::spin();

    return 0;
}

