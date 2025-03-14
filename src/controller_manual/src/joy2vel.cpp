#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class JoyToCmdVel
{
public:
    JoyToCmdVel();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    ros::NodeHandle nh_;

    // Parameters
    int linear_axis_;
    int angular_axis_;
    double linear_scale_;
    double angular_scale_;
    double deadzone_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber joy_sub_;
};

JoyToCmdVel::JoyToCmdVel() : linear_axis_(1),
                             angular_axis_(0),
                             linear_scale_(0.2),
                             angular_scale_(1.0),
                             deadzone_(0.1)
{
    // Load parameters from the parameter server
    nh_.param("linear_axis", linear_axis_, linear_axis_);
    nh_.param("angular_axis", angular_axis_, angular_axis_);
    nh_.param("linear_scale", linear_scale_, linear_scale_);
    nh_.param("angular_scale", angular_scale_, angular_scale_);
    nh_.param("deadzone", deadzone_, deadzone_);

    // Publisher for cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscriber for joy topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyToCmdVel::joyCallback, this);

    ROS_INFO("Manual controller node initialized. Waiting for joystick input...");
}

void JoyToCmdVel::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;

    // Apply deadzone and get values from axes
    double linear_input = joy->axes[linear_axis_];
    double angular_input = joy->axes[angular_axis_];

    if (std::abs(linear_input) < deadzone_)
        linear_input = 0.0;
    if (std::abs(angular_input) < deadzone_)
        angular_input = 0.0;

    // Set linear velocity (forward/backward)
    twist.linear.x = linear_input * linear_scale_;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;

    // Set angular velocity (rotation)
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = angular_input * angular_scale_;

    // Publish the twist message
    cmd_vel_pub_.publish(twist);

    // Debug output
    ROS_DEBUG("Linear vel: %.2f, Angular vel: %.2f", twist.linear.x, twist.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_cmd_vel");
    JoyToCmdVel controller;
    ros::spin();

    return 0;
}