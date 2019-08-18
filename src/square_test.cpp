#include <iostream>

#include <cstdlib>

#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na

float r;
float x, y, theta;

class SquareTest
{
private:
    ros::NodeHandle n;
    ros::Publisher square_vel_pub;
    ros::Subscriber odom_sub;


    geometry_msgs::Twist calculateCommand()
    {
        //Variable to store original position
        static float x_orig , y_orig, theta_orig;

        static int switch_case = 0;
        static bool init_flag = false;
        static bool done_flag = false;

        if(!init_flag){
          x_orig = 0;
          y_orig = 0;
          theta_orig = 0;
          init_flag = true;
        }

        auto square_vel_msg = geometry_msgs::Twist();
        //Code here for square test

        //Allowing error of 10 degrees
        if(theta_orig<=(M_PI/18) && theta_orig>=(2*M_PI - M_PI/18)){
        	// 0 degree line
        	switch_case = 1;
        }
        else if (theta_orig>=(M_PI/2.25) && theta_orig<=(M_PI/1.8)){
        	// 90 degree line
        	switch_case = 2;
        }
        else if(theta_orig>=(M_PI-M_PI/18) && theta_orig<=(M_PI+M_PI/18)){
        	// 180 degree line
        	switch_case = 3;
        }
        else {
          //270 degree line to complete square
          switch_case = 4;
        }


        switch(switch_case) {

            case 1 :       	// 0 degree line - moving right on horizontal
                            if (x < x_orig + 0.5){
                              square_vel_msg.linear.x = 0.001; // m/0.1s
                              square_vel_msg.angular.z = 0;
                            }else{
                              done_flag = true;
                            }
                            break;

            case 2 :		// Avoidance manuever
                            if (y < y_orig + 0.5){
                              square_vel_msg.linear.x = 0.001; // m/0.1s
                              square_vel_msg.angular.z = 0;
                            }else{
                              done_flag = true;
                            }
                            break;
            case 3 :       	// PI controller
                            if (x > x_orig - 0.5){
                              square_vel_msg.linear.x = 0.001; // m/0.1s
                              square_vel_msg.angular.z = 0;
                            }else{
                              done_flag = true;
                            }
                            break;

            case 4 :		// Avoidance manuever
                            if (y > y_orig - 0.5){
                              square_vel_msg.linear.x = 0.001; // m/0.1s
                              square_vel_msg.angular.z = 0;
                            }else{
                              done_flag = true;
                            }
                            break;
        }

        if(done_flag){
          if(turning_flag){
            theta_orig = theta;
          }else{
            turning_flag = false;
          }
          if(theta < theta_orig + M_PI/2){
            square_vel_msg.linear.x = 0.0; // m/0.1s
            square_vel_msg.angular.z = M_PI;
          } else{
          	x_orig = x;
          	y_orig = y;
            done_flag = false;
          }
        }

        return square_vel_msg;
    }




    void odomCallback(const nav_msgs::Odometry& odom_msg)
    {
    	// Update position values with values from the pose_pub msgs
        x = odom_msg.pose.pose.position.x;
        y = odom_msg.pose.pose.position.y;
        theta = odom_msg.pose.pose.orientation.w;
    }


public:
    SquareTest(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages to square_vel_pub
        this->square_vel_pub = this->n.advertise<geometry_msgs::Twist>("square_vel_pub", 10);

        // Create a subscriber for position
        this->odom_sub = n.subscribe("odom_pub", 10, &SquareTest::odomCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to drive in squares
            auto square_vel_msg = calculateCommand();

            // Publish the new command
            this->square_vel_pub.publish(square_vel_msg);

            // Go through buffer
            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "square_test");


    // Create our controller object and run it
    auto controller = SquareTest();
    controller.run();
}
