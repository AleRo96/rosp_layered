/**
 * @file node.hpp
 * @brief Declaration of the members of the Class MotionControl.
 * 
 */

#ifndef NODE_HPP
#define NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>

using namespace std;

/**
 * @class MotionControl.
 * @brief It control the turtle through a given sequence of waypoints.
 * 
 */

class MotionControl : public rclcpp::Node
{
    public:

     /**
     * @brief Constructor for MotionControl.
     * @param name Name of the ROS2 node.
     */

    MotionControl(const std::string& name);


    private:

    /**
     * @brief Callback function to update the current pose of the turtle.
     * @param msg Shared pointer of type Pose message.
     */

    void poseCallback(const turtlesim::msg::Pose::SharedPtr pose);


    /**
     * @brief Implementation of the turtle's movements
     */
    void stateMachine();

    /**
     * @brief Subscriber
     */
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub;

    /**
     * @brief Publisher
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;


    turtlesim::msg::Pose current_pose;

    /**
     * @brief Sequence of waypoints to be followed
     */
    const std::vector<std::vector<double>> POINTS = {
        {1.0, 8.0, 0.0}, {3.0, 8.0, 0.0}, {3.0, 5.0, 0.0}, {1.0, 5.0, 0.0},
        {3.0, 3.0, 0.0}, {5.0, 3.0, 0.0}, {5.0, 8.0, 0.0}, {7.0, 8.0, 0.0},
        {7.0, 3.0, 0.0}, {5.0, 3.0, 0.0}, {9.0, 8.0, 0.0}, {7.0, 8.0, 0.0},
        {7.0, 6.0, 0.0}, {9.0, 6.0, 0.0}, {9.0, 4.0, 0.0}, {7.0, 4.0, 0.0}};


};

#endif