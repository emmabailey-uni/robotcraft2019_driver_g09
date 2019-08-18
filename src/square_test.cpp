#include <iostream>

#include <cstdlib>
#include <stdbool.h>

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

        static bool init_flag = false;
        static bool done_driving_forward_flag = false;
        static bool start_turning_flag = false;


        if(!init_flag){
          x_orig = 0;
          y_orig = 0;
          theta_orig = 0;
          init_flag = true;
        }

        auto square_vel_msg = geometry_msgs::Twist();

        //std::cout<<"Theta current: "<<theta<<"\n";


        float distance = sqrt((x_orig - x)*(x_orig - x) + (y_orig - y)*(y_orig - y));
        if (distance<0.5){
          square_vel_msg.linear.x = 0.5; // m/0.1s
          square_vel_msg.angular.z = 0;
          theta_orig = theta;
        }else{
          done_driving_forward_flag = true;
          start_turning_flag = true;
        }


        if(start_turning_flag){
          if(theta_orig > 1.5 && theta < 0.5){
            theta = theta+2;
          }
          if(fabs(theta - theta_orig) <= 0.499){
            square_vel_msg.angular.z = 1;
            std::cout<<"Theta current: "<<theta<<"\n";
            std::cout<<"Theta orig: "<<theta_orig<<"\n";
          }else{
            // When the turn is finished update the reference position
            x_orig = x;
            y_orig = y;
            distance = 0;
            start_turning_flag = false;
            done_driving_forward_flag = false;
          }
        }













        /*



        if(start_turning_flag){
        	// Start turning
          float desired_orient = theta_orig + M_PI/2;
          if(desired_orient > 2*M_PI){
            desired_orient = desired_orient - 2*M_PI;
          }

          if(theta < desired_orient || abs(theta-desired_orient) > M_PI/2){
            square_vel_msg.linear.x = 0.0; // m/0.1s
            square_vel_msg.angular.z = M_PI/5;
            //std::cout<<"Theta current: "<<theta<<"\n";
            //std::cout<<"Theta desired: "<<desired_orient<<"\n";

          } else{
          	// When the turn is finished update the reference position
          	x_orig = x;
          	y_orig = y;
            distance = 0;

          	// End turning and start moving forward
            start_turning_flag = false;
            done_driving_forward_flag = false;
          }
        }
        // Return the velocity command message
        */
        return square_vel_msg;
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
      x = msg->pose.pose.position.x;
      y = msg->pose.pose.position.y;
      //theta = msg->pose.pose.orientation.z;


      theta = msg->pose.pose.orientation.z;
      if(theta < 0){
        theta = 1+1+theta;
      }

    }


public:
    SquareTest(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages to square_vel_pub
        this->square_vel_pub = this->n.advertise<geometry_msgs::Twist>("square_vel_pub", 10);

        // Create a subscriber for position
        //CHANGE BACK TO ODOM_PUB FOR ARDUINO
        this->odom_sub = n.subscribe("odom", 10, &SquareTest::odomCallback, this);

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
