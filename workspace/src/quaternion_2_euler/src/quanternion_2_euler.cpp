#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <sstream>
#include <math.h>

class Quaternion2Euler{
    public:
        Quaternion2Euler(ros::NodeHandle * nh){
            sub = nh->subscribe("odom",1000,&Quaternion2Euler::callback,this);
            pub = nh->advertise<std_msgs::Float32>("robot_yaw",10);
            ROS_INFO("QUATERNION NODE ONLINE");
        }

        void callback(const nav_msgs::Odometry & odom){
            double x,y,z,w; // quaternion terms
            double roll,pitch,yaw; // euler angles terms
            x = odom.pose.pose.orientation.x;
            y = odom.pose.pose.orientation.y;
            z = odom.pose.pose.orientation.z;
            w = odom.pose.pose.orientation.w;
            // convert quaterion to euler angles
            tf::Quaternion q(x,y,z,w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);
            yaw = radian_2_degree(yaw);
            std_msgs::Float32 yaw_msg;
            yaw_msg.data = yaw;
            pub.publish(yaw_msg);


        }

        double radian_2_degree(double euler_ang){
            return euler_ang * (180.0/M_PI);
        }

    private:
        
        ros::Publisher pub;
        ros::Subscriber sub;
};

int main(int argc,char **argv){

    ros::init(argc, argv, "quaterion_2_euler");
    ros::NodeHandle nh;
    Quaternion2Euler node(&nh);
    ros::spin();

    return 0;
}


