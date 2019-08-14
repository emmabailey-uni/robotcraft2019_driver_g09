#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na

float x;
float y;
float theta;

class SquareTest
{
private:
    ros::NodeHandle n;
    ros::Publisher square_vel_pub;
    ros::Subscriber odom_sub;


    geometry_msgs::Twist calculateCommand()
    {
        auto square_vel_msg = geometry_msgs::Twist();
        //Code here for square test

        

        
        
        return square_vel_msg;
    }




    void odomCallback(const nav_msgs::Odometry& odom_msg)
    {	
    	// Update position values with values from the pose_pub msgs
        x = &odom_msg.pose.pose.position.x;
        y = &odom_msg.pose.pose.position.y;
        theta = &odom_msg.pose.pose.orientation.w;
    }


public:
    SquareTest(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages to square_vel_pub
        this->square_vel_pub = this->n.advertise<geometry_msgs::Twist>("square_vel_pub", 10);

        // Create a subscriber for position 
        this->odom_sub = n.subscribe("odom_pub", 10, &SquareTest::poseCallback, this);

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
