#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>

class JoyToCmdVel
{
public:
    JoyToCmdVel();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void timerCallback(const ros::TimerEvent&);

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;

    geometry_msgs::Twist last_twist_;
    bool is_moving_;        // True if joystick is giving movement commands
    bool need_publish_;     // True if we need to publish a cmd_vel message

    // Parameters
    int linear_axis_;
    int angular_axis_;
    double linear_scale_;
    double angular_scale_;
    double deadzone_;
};

JoyToCmdVel::JoyToCmdVel()
    : linear_axis_(1),
      angular_axis_(0),
      linear_scale_(0.2),
      angular_scale_(1.0),
      deadzone_(0.1),
      is_moving_(false),
      need_publish_(false)
{
    // Load parameters (can be overridden via ROS parameter server)
    nh_.param("linear_axis", linear_axis_, linear_axis_);
    nh_.param("angular_axis", angular_axis_, angular_axis_);
    nh_.param("linear_scale", linear_scale_, linear_scale_);
    nh_.param("angular_scale", angular_scale_, angular_scale_);
    nh_.param("deadzone", deadzone_, deadzone_);

    // Publisher for velocity commands
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscriber for joystick input
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyToCmdVel::joyCallback, this);

    // Timer to periodically check and publish velocity commands
    timer_ = nh_.createTimer(ros::Duration(0.05), &JoyToCmdVel::timerCallback, this); // 20Hz

    ROS_INFO("Joystick-to-cmd_vel node initialized.");
}

void JoyToCmdVel::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Read joystick axes
    double linear_input = joy->axes[linear_axis_];
    double angular_input = joy->axes[angular_axis_];

    // Apply deadzone
    if (std::abs(linear_input) < deadzone_)
        linear_input = 0.0;
    if (std::abs(angular_input) < deadzone_)
        angular_input = 0.0;

    // Scale inputs and store them in the last_twist message
    last_twist_.linear.x = linear_input * linear_scale_;
    last_twist_.linear.y = 0.0;
    last_twist_.linear.z = 0.0;

    last_twist_.angular.x = 0.0;
    last_twist_.angular.y = 0.0;
    last_twist_.angular.z = angular_input * angular_scale_;

    // Decide if we should start or stop publishing
    if (linear_input != 0.0 || angular_input != 0.0)
    {
        is_moving_ = true;
        need_publish_ = true;
    }
    else if (is_moving_)
    {
        // Joystick returned to center: stop the robot and publish zero velocity once
        is_moving_ = false;
        last_twist_ = geometry_msgs::Twist();  // All zeros
        need_publish_ = true;
    }
}

void JoyToCmdVel::timerCallback(const ros::TimerEvent&)
{
    if (need_publish_)
    {
        cmd_vel_pub_.publish(last_twist_);

        // Continue publishing only if still moving
        need_publish_ = is_moving_;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_cmd_vel");
    JoyToCmdVel controller;
    ros::spin();
    return 0;
}
