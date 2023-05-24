#define _USE_MATH_DEFINES
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "turtlebot_navigation/NavigationAction.h"
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <math.h>

//using std::placeholders::_1;

class NavigationAction{
    public:
    
        NavigationAction(std::string name):action_server(nh,name,boost::bind(&NavigationAction::execute_cb,this,_1),false),action_name(name)
        {
            action_server.start();

            // instance publisher and subscriber
            odom_sum = nh.subscribe("odom",1000,&NavigationAction::odometry_callback,this);
            vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);

            // set arbitrary gains for testing 
            kp = 0.3;
            kd = 0.4;

            // set arbitrary linear speed
            linear_speed = 1.0;

            ROS_INFO("NAVIGATION SERVICE ONLINE");
        }

        // robot navigation system with feedback to the client
        void execute_cb(const turtlebot_navigation::NavigationGoalConstPtr & goal){

        }

        // function that retrieves the curren position of the robot
        void odometry_callback(const nav_msgs::Odometry & odom){
            double x,y,z,w; // quaternion terms
            double pos_x,pos_y; // position terms
            double roll,pitch,yaw; // euler angles terms

            x = odom.pose.pose.orientation.x;
            y = odom.pose.pose.orientation.y;
            z = odom.pose.pose.orientation.z;
            w = odom.pose.pose.orientation.w;

            // convert quaterion to euler angles
            tf::Quaternion q(x,y,z,w);
            tf::Matrix3x3 m(q);
            m.getRPY(roll,pitch,yaw);

            // retieve position terms
            pos_x = odom.pose.pose.position.x;
            pos_y = odom.pose.pose.position.y;

            // store parameters
            x_curr = pos_x;
            y_curr = pos_y;
            yaw_curr = yaw;
        }





        // navigation system's auxiliary functions are defined below
        void go_2_goal(void){
            
        }

        // target rotation before it starts walking towards the desired goal
        void target_rotation(void){
            direction_factor = 1.0;
            this->initialize_angular_error();
            while(angular_error > angular_threshold){
                this->update_angular_error();
                auto cmd_vel = geometry_msgs::Twist();
                cmd_vel.angular.z = direction_factor * (angular_error * kp +(angular_error-past_angular_error) * kd);
                vel_pub.publish(cmd_vel);
            }
            auto cmd_vel = geometry_msgs::Twist();
            cmd_vel.angular.z = 0.0;
            vel_pub.publish(cmd_vel);
        }

        // this function initialises the angular and past angular error to the maximum value
        void initialize_angular_error(void){
            angular_error = M_PI;
            past_angular_error = M_PI;
        }


        void update_angular_error(void){
            float xt,yt,tt;
            float xc,yc,tc;
            float x_diff, y_diff,yaw_diff;
            // target position
            xt = x_target;
            yt = y_target;
            
            // current position
            xc = x_curr;
            yc = y_curr;
            tc = yaw_curr;

            // difference
            x_diff = xt-xc;
            y_diff = yt-yc;
            tt = atan2(x_diff,y_diff);
            yaw_diff = tt-tc;
            past_angular_error = angular_error;
            angular_error = abs(yaw_diff);
            this->calculateFactor(yaw_diff);
        }

        // this is to decide the robot's direction
        void calculateFactor(float theta_diff){

            if(angular_error > M_PI && theta_diff > 0){
                direction_factor = -1.0;
            }
            else if(angular_error > M_PI && theta_diff < 0){
                direction_factor = 1.0;
            }
            else if(angular_error < M_PI && theta_diff < 0){
                direction_factor = -1.0;
            }
            else{
                direction_factor = 1.0;
            }
            
        }

    private:
        // odometry and velocity publishers and subscribers to control the robot
        ros::Publisher vel_pub;
        ros::Subscriber odom_sum;
        // node handler
        ros::NodeHandle nh;
        // action server
        actionlib::SimpleActionServer<turtlebot_navigation::NavigationAction> action_server;
        std::string action_name;
        turtlebot_navigation::NavigationFeedback action_feedback;
        turtlebot_navigation::NavigationResult action_result;

        // placeholders for current possition
        float x_curr;
        float y_curr;
        float yaw_curr;

        // placeholders for current target
        float x_target;
        float y_target;
        float yaw_target;

        // errors & thresholds
        float distance_error;
        float angular_error,past_angular_error;
        float angular_threshold;
        float distance_threshold;

        // direction factor for rotation
        float direction_factor;

        // linear speed
        float linear_speed;

        // controller gains 
        float kp,kd;




};

int main(int argc,char **argv){

    ros::init(argc, argv, "turtlebot_navigation_action_server");

    return 0;
}