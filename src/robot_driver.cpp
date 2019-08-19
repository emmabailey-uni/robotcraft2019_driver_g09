#include <iostream>
#include <cstdlib>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosserial_arduino/Test.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Header.h"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884L
#endif na


class RobotDriver
{
private:
    //Initialising node NodeHandle
    ros::NodeHandle n;

    //Keep track of current ctime
    ros::Time current_time;
    ros::Time last_time = ros::Time::now();


    //Declare transform broadcaster to send messages via tf and ROS
    tf::TransformBroadcaster odom_broadcaster;

    double v;
    double w;


    //Publisher Topics
    ros::Publisher cmd_vel_pub;
    ros::Publisher odom_pub;
    ros::Publisher set_pose_pub;
    ros::Publisher rgb_leds_pub;
    ros::Publisher ir_front_sensor;
    ros::Publisher ir_left_sensor;
    ros::Publisher ir_right_sensor;

    //Subscriber Topics
    ros::Subscriber left_dist_sub;
    ros::Subscriber right_dist_sub;
    ros::Subscriber front_dist_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber square_vel_sub;

    //Service client
    ros::ServiceClient client;


    double front_obstacle_distance;
    double right_obstacle_distance;
    double left_obstacle_distance;

    geometry_msgs::Twist calculateCommand(){
        auto msg = geometry_msgs::Twist();
    }

    //Callback functions for subscribers
    void poseCallback(const geometry_msgs::Pose2D& pose_msg){
      //  relay <pose_sub> [odom_pub];

        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_msg.theta);

        //first, we'll publish the transform over tf - broadcasts to rviz
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = pose_msg.x;
        odom_trans.transform.translation.y = pose_msg.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = pose_msg.x;
        odom.pose.pose.position.y = pose_msg.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = w;

        //publish the message
        odom_pub.publish(odom);
        last_time = current_time;

    }

    void leftCallback( const std_msgs::Float32& left_dist_msg){
        sensor_msgs::Range ir_left;
        //relay <left_dist_sub> [ir_left_sensor]

        //.data is for acces to all the data from the msg
        float sensor_val_left = static_cast<float>(left_dist_msg.data);

        ir_left.header.frame_id = "base_link";
        ir_left.radiation_type = 1;
        ir_left.field_of_view = 0.034906585;
        ir_left.min_range = 0.1;
        ir_left.max_range = 0.8;
        ir_left.range = sensor_val_left;

        //Publish the left sensor value to ir_left_sensor
        ir_left_sensor.publish(ir_left);



    }

    void rightCallback(const std_msgs::Float32& right_dist_msg){
        sensor_msgs::Range ir_right;
        //relay <right_dist_sub> [ir_right_sensor]

        float sensor_val_right = static_cast<float>(right_dist_msg.data);

        ir_right.header.frame_id = "base_link";
        ir_right.radiation_type = 1;
        ir_right.field_of_view = 0.034906585;
        ir_right.min_range = 0.1;
        ir_right.max_range = 0.8;
        ir_right.range = sensor_val_right;

        //Publish the right sensor value to ir_right_sensor
        ir_right_sensor.publish(ir_right);
    }

    void frontCallback(const std_msgs::Float32& front_dist_msg){
        sensor_msgs::Range ir_front;
        //relay <front_dist_sub> [ir_front_sensor]

        float sensor_val_front = static_cast<float>(front_dist_msg.data);

        ir_front.header.frame_id = "base_link";
        ir_front.radiation_type = 1;
        ir_front.field_of_view = 0.034906585;
        ir_front.min_range = 0.1;
        ir_front.max_range = 0.8;
        ir_front.range = sensor_val_front;

        //Publish the front sensor value to ir_front_sensor
        ir_front_sensor.publish(ir_front);
    }

    void SquareVelCallback(const geometry_msgs::Twist& square_vel_msg){
        // Update globally stored velocities for further use/publishing
        v=square_vel_msg.linear.x;
        w=square_vel_msg.angular.z;
    }

    geometry_msgs::Twist cmdVelUpdate(){
        auto vel_MSG = geometry_msgs::Twist();
        // Update velocities to be published
        vel_MSG.linear.x = v;
        vel_MSG.angular.z = w;
        return vel_MSG;
    }

    geometry_msgs::Pose2D setPose(){
        auto pose_MSG = geometry_msgs::Pose2D();
        // Set position to zero
        pose_MSG.x = 0.0;
        pose_MSG.y = 0.0;
        pose_MSG.theta = 0.0;
        return pose_MSG;
    }

    std_msgs::UInt8MultiArray setLEDs(){
        auto rgb_MSG = std_msgs::UInt8MultiArray();
        // Set color of led lights, the first 3 enteries are for LED_1 [255,0,0] and the last 3 for LED_2 [0,255,0]
        rgb_MSG.data = [255,0,0,0,255,0];
        return rgb_MSG;
    }

    void switchBuzzerState(char c){
        // Set led to "0" or "1" by char c
        rosserial_arduino::Test Buzzer_ctr;
        Buzzer_ctr.request.input = c;

        //NOT SURE IF THIS IS THE CORRECT WAY TO ACCESS CLIENT
        if(this->client.call(Buzzer_ctr)){
            ROS_INFO(Buzzer_ctr.response.output);
        }else{
            ROS_ERROR("Failed to call service ");
        }
    }



public:
    RobotDriver(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        this->odom_pub = this->n.advertise<nav_msgs::Odometry>("odom", 10);
        this->set_pose_pub = this->n.advertise<geometry_msgs::Pose2D>("set_pose", 10);
        this->rgb_leds_pub = this->n.advertise<std_msgs::UInt8MultiArray>("rgb_leds", 10);
        this->ir_front_sensor = this->n.advertise<sensor_msgs::Range>("ir_front_sensor", 10);
        this->ir_left_sensor = this->n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);
        this->ir_right_sensor = this->n.advertise<sensor_msgs::Range>("ir_right_sensor", 10);

        // Create a subscriber for laser scans
        //this->laser_sub = n.subscribe("base_scan", 10, &SquareTest::laserCallback, this);
        this->pose_sub = n.subscribe("pose", 10, &RobotDriver::poseCallback, this);
        this->left_dist_sub = n.subscribe("left_dist", 10, &RobotDriver::leftCallback, this);
        this->right_dist_sub = n.subscribe("right_dist", 10, &RobotDriver::rightCallback, this);
        this->front_dist_sub = n.subscribe("front_dist", 10, &RobotDriver::frontCallback, this);
        this->square_vel_sub = n.subscribe("square_vel_pub", 10, &RobotDriver::SquareVelCallback, this);


        // Service Client ALSO NOT SURE IF I NEED TO ADD "~" BEFORE switch_buzzer_state
        this->client = n.serviceClient<rosserial_arduino::Test>("switch_buzzer_state");


    }


    void run(){
        int count = 0;
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();
            auto vel_MSG = cmdVelUpdate();
            auto pose_MSG = setPose();
            auto rgb_MSG = setLEDs();

            //HAVENT CREATED THESE VARIABLES YET
            // Publish the new command
            this->cmd_vel_pub.publish(vel_MSG);
            this->set_pose_pub.publish(pose_MSG);
            this->rgb_leds_pub.publish(rgb_MSG);

            // Buzz from count 100 to 200
            if(count = 100){
                switchBuzzerState("1");
            }
            if(count = 200){
                switchBuzzerState("0");
            }
            if(count > 200){
                count = count;
            }else{
                count++;
            }

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){

    // Initialize ROS
    ros::init(argc, argv, "message_service");

    // Create our controller object and run it
    auto controller = RobotDriver();
    controller.run();
}