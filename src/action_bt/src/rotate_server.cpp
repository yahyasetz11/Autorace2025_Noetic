#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/RotateAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

// Define constants
#define DEG_TO_RAD 0.017453293  // PI/180.0
#define RAD_TO_DEG 57.295779513 // 180.0/PI

class RotateServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::RotateAction> as_;
    std::string action_name_;
    msg_file::RotateFeedback feedback_;
    msg_file::RotateResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    // Parameters
    double angular_speed_; // rad/s

    // Odometry data
    double current_yaw_; // in radians
    double initial_yaw_; // in radians
    bool odom_received_;

    // Action parameters
    std::string direction_;
    double target_angle_deg_;

public:
    RotateServer(std::string name) : as_(nh_, name, boost::bind(&RotateServer::executeCB, this, _1), false),
                                     action_name_(name),
                                     odom_received_(false),
                                     angular_speed_(0.5) // Default value in case config loading fails
    {
        // Load speed from config file
        loadSpeedConfig();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        odom_sub_ = nh_.subscribe("/odom", 1, &RotateServer::odomCallback, this);

        as_.start();
        ROS_INFO("Rotate Action Server Started with angular_speed: %.2f rad/s", angular_speed_);
    }

    void loadSpeedConfig()
    {
        try
        {
            // Get the package path
            std::string package_path = ros::package::getPath("action_bt");
            if (package_path.empty())
            {
                package_path = ros::package::getPath("behaviortree");
            }

            // Build the path to the config file
            std::string config_path = package_path + "/../config/speed_conf.yaml";

            // Load the YAML file
            YAML::Node config = YAML::LoadFile(config_path);

            // Extract angular speed
            if (config["angular_speed"])
            {
                angular_speed_ = config["angular_speed"].as<double>();
                ROS_INFO("Loaded angular_speed = %.2f from config file", angular_speed_);
            }
            else
            {
                ROS_WARN("Could not find 'angular_speed' in config file, using default: %.2f", angular_speed_);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error loading config file: %s", e.what());
            ROS_WARN("Using default angular_speed: %.2f", angular_speed_);
        }
    }

    ~RotateServer()
    {
        // Ensure the robot stops when the server shuts down
        stopRobot();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Extract yaw from quaternion
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw_ = yaw;
        odom_received_ = true;
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Stopping robot");
    }

    void rotateRobot(double angular_vel)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_vel;
        cmd_vel_pub_.publish(cmd);
    }

    // Normalize angle to (-pi, pi)
    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // Calculate the smallest angle difference between two angles
    double getAngleDifference(double target, double current)
    {
        double diff = normalizeAngle(target - current);
        return diff;
    }

    void executeCB(const msg_file::RotateGoalConstPtr &goal)
    {
        bool success = true;
        ros::Rate r(10); // 10 Hz

        // Get parameters from goal
        direction_ = goal->direction;
        target_angle_deg_ = fabs(goal->angle); // Make sure angle is positive

        // Validate parameters
        if (direction_ != "clockwise" && direction_ != "counterclockwise")
        {
            ROS_ERROR("Invalid direction: %s. Must be 'clockwise' or 'counterclockwise'", direction_.c_str());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        if (target_angle_deg_ <= 0.0)
        {
            ROS_ERROR("Invalid angle: %.2f. Must be greater than 0", target_angle_deg_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Determine rotation direction
        double rotate_speed = angular_speed_;
        if (direction_ == "clockwise")
        {
            rotate_speed = -rotate_speed; // Negative is clockwise for ROS
        }

        ROS_INFO("Rotate Server: Direction=%s, Angle=%.2f degrees",
                 direction_.c_str(), target_angle_deg_);

        // Wait until we receive odometry data
        ros::Time start_wait = ros::Time::now();
        while (!odom_received_ && (ros::Time::now() - start_wait).toSec() < 5.0)
        {
            ROS_INFO_THROTTLE(1, "Waiting for odometry data...");
            r.sleep();
        }

        if (!odom_received_)
        {
            ROS_ERROR("No odometry data received after waiting 5 seconds!");
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Record initial yaw
        initial_yaw_ = current_yaw_;
        ROS_INFO("Initial yaw: %.2f degrees", initial_yaw_ * RAD_TO_DEG);

        // Convert target angle to radians
        double target_angle_rad = target_angle_deg_ * DEG_TO_RAD;

        // Initialize angle tracking
        double angle_turned = 0.0;
        double prev_yaw = initial_yaw_;
        double total_rotated = 0.0; // Keep track of total rotation

        // Main execution loop
        while (ros::ok())
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                stopRobot();
                as_.setPreempted();
                success = false;
                break;
            }

            // Calculate how much we've rotated since last iteration
            double delta_yaw = getAngleDifference(current_yaw_, prev_yaw);

            // Adjust delta based on direction to ensure it's counted correctly
            if (direction_ == "clockwise" && delta_yaw > 0)
            {
                delta_yaw = delta_yaw - 2 * M_PI;
            }
            else if (direction_ == "counterclockwise" && delta_yaw < 0)
            {
                delta_yaw = delta_yaw + 2 * M_PI;
            }

            // Update total rotation
            total_rotated += fabs(delta_yaw);

            // Update previous yaw for next iteration
            prev_yaw = current_yaw_;

            // Convert to degrees for feedback and logging
            double degrees_turned = total_rotated * RAD_TO_DEG;

            // Update feedback
            feedback_.current_angle = degrees_turned;
            as_.publishFeedback(feedback_);

            // Check if we've rotated enough
            if (degrees_turned >= target_angle_deg_)
            {
                ROS_INFO("Rotation completed: %.2f degrees rotated", degrees_turned);
                success = true;
                break;
            }

            // Move the robot
            rotateRobot(rotate_speed);

            // Debug info
            ROS_INFO_THROTTLE(0.5, "Current rotation: %.2f degrees, Target: %.2f degrees",
                              degrees_turned, target_angle_deg_);

            r.sleep();
        }

        // Stop the robot when done
        stopRobot();

        if (success)
        {
            result_.success = true;
            result_.final_angle = feedback_.current_angle;
            ROS_INFO("%s: Succeeded, rotated %.2f degrees", action_name_.c_str(), result_.final_angle);
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_server");
    RotateServer rotateServer("rotate");
    ros::spin();
    return 0;
}