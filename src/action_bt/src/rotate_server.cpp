#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/RotateAction.h>
#include <geometry_msgs/Twist.h>
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

        // Debug info - print the command being sent
        ROS_INFO_THROTTLE(1.0, "Sending rotation command: angular.z = %.2f", angular_vel);
    }

    // Normalize angle to [-π, π]
    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
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

        // Add small tolerance for completion
        double angle_tolerance_deg = 2.0; // 2 degrees tolerance

        // Set rotation speed based on direction
        double rotate_speed;
        if (direction_ == "clockwise")
        {
            rotate_speed = -angular_speed_; // Negative for clockwise
        }
        else
        {                                  // counterclockwise
            rotate_speed = angular_speed_; // Positive for counterclockwise
        }

        ROS_INFO("Rotate Server: Direction=%s, Angle=%.2f degrees, Speed=%.2f",
                 direction_.c_str(), target_angle_deg_, rotate_speed);

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

        // Store initial orientation
        double start_yaw = current_yaw_;
        ROS_INFO("Initial orientation: %.2f degrees", start_yaw * RAD_TO_DEG);

        // Convert target angle to radians
        double target_angle_rad = target_angle_deg_ * DEG_TO_RAD;

        // Start rotating
        rotateRobot(rotate_speed);

        // Store the previous yaw to calculate incremental changes
        double prev_yaw = start_yaw;

        // Accumulated angle turned (in radians)
        double accumulated_angle = 0.0;

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

            // Calculate delta yaw (change in orientation since last iteration)
            double delta_yaw = normalizeAngle(current_yaw_ - prev_yaw);

            // Debug angle information
            ROS_INFO("Current yaw: %.2f, Prev yaw: %.2f, Delta: %.2f",
                     current_yaw_ * RAD_TO_DEG, prev_yaw * RAD_TO_DEG, delta_yaw * RAD_TO_DEG);

            // Adjust delta based on direction to ensure proper accumulation
            if (direction_ == "clockwise")
            {
                // For clockwise, we want negative changes to add to our total
                // If delta is positive (counterclockwise), it might be due to wraparound
                if (delta_yaw > 0 && delta_yaw > M_PI_2)
                {                            // Large positive jump (>90 deg)
                    delta_yaw -= 2.0 * M_PI; // Convert to equivalent negative angle
                }
                // For clockwise, accumulate negative rotation (convert to positive)
                if (delta_yaw < 0)
                {
                    accumulated_angle += -delta_yaw;
                }
            }
            else
            { // counterclockwise
                // For counterclockwise, we want positive changes to add to our total
                // If delta is negative (clockwise), it might be due to wraparound
                if (delta_yaw < 0 && delta_yaw < -M_PI_2)
                {                            // Large negative jump (<-90 deg)
                    delta_yaw += 2.0 * M_PI; // Convert to equivalent positive angle
                }
                // For counterclockwise, accumulate positive rotation
                if (delta_yaw > 0)
                {
                    accumulated_angle += delta_yaw;
                }
            }

            // Store current orientation for next iteration
            prev_yaw = current_yaw_;

            // Convert to degrees for feedback
            double degrees_turned = accumulated_angle * RAD_TO_DEG;

            // Update feedback
            feedback_.current_angle = degrees_turned;
            as_.publishFeedback(feedback_);

            // Check if we've rotated enough
            if (degrees_turned >= (target_angle_deg_ - angle_tolerance_deg))
            {
                ROS_INFO("Rotation goal reached: %.2f degrees rotated (target: %.2f)",
                         degrees_turned, target_angle_deg_);
                success = true;
                break;
            }

            // Debug total rotation
            ROS_INFO("Total rotation: %.2f degrees / %.2f degrees",
                     degrees_turned, target_angle_deg_);

            // Keep rotating the robot
            rotateRobot(rotate_speed);

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