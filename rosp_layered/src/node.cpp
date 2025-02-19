/**
 * @file node.cpp
 * @brief Implementation of the MotionControl class for ROS2
 * 
 */

#include "rosp_layered/node.hpp"
#include <cmath>
#include <iostream>
#include <chrono>
#include <geometry_msgs/msg/twist.h>

using placeholders:: _1;
using namespace chrono_literals;



MotionControl::MotionControl(const string& name) 
                            : Node(name)
{

    /**
     * @brief Subscribes to the turtle's pose topic.
     */
    pose_sub = create_subscription<turtlesim::msg::Pose>("/turtle/pose", 10, bind(&MotionControl::poseCallback, this, _1));


    /**
     * @brief Publishes velocity commands to the turtle.
     */

    vel_pub = create_publisher<geometry_msgs::msg::Twist>("/turtle/cmd_vel", 10);

    stateMachine();

}

void MotionControl::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
    current_pose = *msg;
}


void MotionControl::stateMachine()
{
    rclcpp::Rate rate(100);

    for (const auto& point : POINTS){

        double goal_x = point[0];
        double goal_y = point[1];
        double goal_theta = point[2];



        while (rclcpp::ok()){

            /**
             *  @brief Step 1: Align with the goal position.
             */
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;
            

            double angle_to_goal = atan2(goal_y - current_pose.y, goal_x - current_pose.x);
            double angle_error = angle_to_goal - current_pose.theta;

            cout<< " Angle to goal: "<< angle_to_goal << endl;
            cout<< " Current angle: "<< current_pose.theta << endl;
            cout<< " Angle error: "<< angle_error << endl;

            if (abs(angle_error) < 0.01)
                break;

            
            cmd.angular.z = 3.0 * angle_error;
            vel_pub->publish(cmd);
            rate.sleep();
    }

        /**
         * @brief Step 2: Move to the goal position.
         */
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;
        
            double dx = goal_x - current_pose.x;
            double dy = goal_y - current_pose.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double direction = std::atan2(dy, dx);

            cout << "Distance to goal: " << distance << endl;
            cout << "Current x: " << current_pose.x << endl;
            cout << "Current y: " << current_pose.y << endl;
            cout << "Direction: " << direction << endl;

            if (distance < 0.1)
                break;
            cmd.linear.x = 2.0 * distance * std::cos(direction - current_pose.theta);
            vel_pub->publish(cmd);
            rate.sleep();
            
        }

        /**
         * @brief Step 3: Align with goal orientation 
         */
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            geometry_msgs::msg::Twist cmd;
        
            double theta_error = goal_theta - current_pose.theta;
  
            if (theta_error > M_PI)
                theta_error -= 2 * M_PI;
            if (theta_error < -M_PI)
                theta_error += 2 * M_PI;
  
            if (abs(theta_error) < 0.1)
                break;

            cmd.angular.z = 2.0 * theta_error;
            vel_pub->publish(cmd);
            rate.sleep();
        }
    }


}

/**
 * @brief Main function to initialize and run the MotionControl node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Program exit status.
 */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<MotionControl>("rosp_motion_control");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}