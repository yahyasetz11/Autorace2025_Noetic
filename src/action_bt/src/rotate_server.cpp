#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/RotateAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <msg_file/EulerAngles.h>
#include <tf/transform_datatypes.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "pid.hpp" // Include PID controller header

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
    ros::Subscriber euler_sub_; // New subscriber for yaw_tracker

    // Parameters
    double angular_speed_; // Maximum angular speed (rad/s)

    // PID controller and parameters
    PID_t pid_controller_;
    double pid_kp_;
    double pid_ki_;
    double pid_kd_;

    // Yaw tracking
    double current_yaw_; // in degrees
    double initial_yaw_; // in degrees
    bool yaw_received_;

    // Action parameters
    std::string direction_;
    double target_angle_deg_; // Used for both rotation amount and target yaw

public:
    RotateServer(std::string name) : as_(nh_, name, boost::bind(&RotateServer::executeCB, this, _1), false),
                                     action_name_(name),
                                     yaw_received_(false),
                                     angular_speed_(0.5), // Default value
                                     pid_kp_(1.0),
                                     pid_ki_(0.0),
                                     pid_kd_(0.0)
    {
        // Load parameters from config file
        loadConfig();

        // Initialize PID controller
        pid_param(&pid_controller_, pid_kp_, pid_ki_, pid_kd_);
        pid_controller_.setpoint = 0.0; // Will be set in executeCB

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        odom_sub_ = nh_.subscribe("/odom", 1, &RotateServer::odomCallback, this);
        euler_sub_ = nh_.subscribe("/euler_angles", 1, &RotateServer::eulerCallback, this);

        as_.start();
        ROS_INFO("Rotate Action Server Started with angular_speed: %.2f rad/s, PID: [%.2f, %.2f, %.2f]",
                 angular_speed_, pid_kp_, pid_ki_, pid_kd_);
    }

    void loadConfig()
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
            std::string config_path = package_path + "/../config/rotate_params.yaml";

            // Load the YAML file
            YAML::Node config = YAML::LoadFile(config_path);

            // Extract parameters
            if (config["angular_speed"])
            {
                angular_speed_ = config["angular_speed"].as<double>();
            }

            // Extract PID parameters
            if (config["pid_kp"])
            {
                pid_kp_ = -1 * config["pid_kp"].as<double>();
            }
            if (config["pid_ki"])
            {
                pid_ki_ = config["pid_ki"].as<double>();
            }
            if (config["pid_kd"])
            {
                pid_kd_ = config["pid_kd"].as<double>();
            }

            ROS_INFO("Loaded parameters from %s", config_path.c_str());
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error loading config file: %s", e.what());
            ROS_WARN("Using default parameters: angular_speed=%.2f, PID=[%.2f, %.2f, %.2f]",
                     angular_speed_, pid_kp_, pid_ki_, pid_kd_);
        }
    }

    ~RotateServer()
    {
        // Ensure the robot stops when the server shuts down
        stopRobot();
    }

    // Callback for odometry data (keeping for backward compatibility)
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        // Only use if we haven't received yaw from the yaw_tracker
        if (!yaw_received_)
        {
            // Extract quaternion orientation
            tf::Quaternion q(
                odom_msg->pose.pose.orientation.x,
                odom_msg->pose.pose.orientation.y,
                odom_msg->pose.pose.orientation.z,
                odom_msg->pose.pose.orientation.w);

            // Convert to Euler angles
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            // Convert to degrees
            current_yaw_ = yaw * RAD_TO_DEG;
        }
    }

    // New callback for the yaw_tracker's euler angles message
    void eulerCallback(const msg_file::EulerAngles::ConstPtr &euler_msg)
    {
        // Update current yaw (already in degrees from yaw_tracker)
        current_yaw_ = euler_msg->yaw;
        yaw_received_ = true;
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

    // Normalize angle to [-180, 180]
    double normalizeAngle(double angle)
    {
        while (angle > 180.0)
            angle -= 360.0;
        while (angle < -180.0)
            angle += 360.0;
        return angle;
    }

    // Calculate angular error (for PID controller)
    double calculateAngularError(double current, double target)
    {
        double error = target - current;

        // Normalize to [-180, 180]
        error = normalizeAngle(error);

        return error;
    }

    void executeCB(const msg_file::RotateGoalConstPtr &goal)
    {
        bool success = true;
        ros::Rate r(10); // 10 Hz

        // Get parameters from goal
        direction_ = goal->direction;
        target_angle_deg_ = goal->angle; // Used for both rotation amount and target yaw

        // Validate parameters
        if (direction_ != "clockwise" && direction_ != "counterclockwise" && direction_ != "align")
        {
            ROS_ERROR("Invalid direction: %s. Must be 'clockwise', 'counterclockwise', or 'align'", direction_.c_str());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        if (direction_ != "align" && target_angle_deg_ <= 0.0)
        {
            ROS_ERROR("Invalid angle: %.2f. Must be greater than 0", target_angle_deg_);
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Add small tolerance for completion
        double angle_tolerance_deg = 1.0; // 2 degrees tolerance

        // Wait until we receive yaw data
        ros::Time start_wait = ros::Time::now();
        while (!yaw_received_ && (ros::Time::now() - start_wait).toSec() < 5.0)
        {
            ROS_INFO_THROTTLE(1, "Waiting for yaw data...");
            ros::spinOnce();
            r.sleep();
        }

        if (!yaw_received_)
        {
            ROS_ERROR("No yaw data received after waiting 5 seconds!");
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Store initial orientation
        initial_yaw_ = current_yaw_;
        ROS_INFO("Initial orientation: %.2f degrees", initial_yaw_);

        // Reset PID controller
        pid_controller_.setpoint = 0.0; // We'll feed error directly
        pid_controller_.sum_error = 0.0;
        pid_controller_.last_error = 0.0;

        // For align mode, log the target yaw
        if (direction_ == "align")
        {
            ROS_INFO("Align mode: current=%.2f, target=%.2f", current_yaw_, target_angle_deg_);
        }
        else
        {
            // For normal rotation modes, calculate target based on direction and amount
            double amount = fabs(target_angle_deg_); // Make sure angle is positive

            // Calculate direction sign
            int dir_sign = (direction_ == "clockwise") ? -1 : 1;

            ROS_INFO("Rotate mode: %s, Amount=%.2f degrees",
                     direction_.c_str(), amount);
        }

        // Variables for tracking rotation progress
        double accumulated_angle = 0.0;
        double prev_yaw = initial_yaw_;

        // Target for rotation modes
        double target_yaw = 0.0; // Will be calculated for each mode

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

            // Calculate delta yaw from previous reading
            double delta_yaw = normalizeAngle(current_yaw_ - prev_yaw);

            // Update accumulated angle based on direction
            if (direction_ == "clockwise")
            {
                // For clockwise, negative changes (decreasing yaw) contribute to accumulated angle
                if (delta_yaw > 0 && delta_yaw > 90.0)
                {
                    // Large positive jump means we crossed the -180/180 boundary
                    delta_yaw -= 360.0;
                }

                if (delta_yaw < 0)
                {
                    accumulated_angle += -delta_yaw;
                }
            }
            else if (direction_ == "counterclockwise")
            {
                // For counterclockwise, positive changes (increasing yaw) contribute to accumulated angle
                if (delta_yaw < 0 && delta_yaw < -90.0)
                {
                    // Large negative jump means we crossed the -180/180 boundary
                    delta_yaw += 360.0;
                }

                if (delta_yaw > 0)
                {
                    accumulated_angle += delta_yaw;
                }
            }

            // Store current yaw for next iteration
            prev_yaw = current_yaw_;

            // Handle different rotation modes with PID
            double error = 0.0;

            if (direction_ == "align")
            {
                // For align mode, error is difference between current and target absolute yaw
                error = calculateAngularError(current_yaw_, target_angle_deg_);

                // Debug info
                ROS_INFO_THROTTLE(0.5, "Align mode: current=%.2f, target=%.2f, error=%.2f",
                                  current_yaw_, target_angle_deg_, error);

                // Success condition for align mode
                if (fabs(error) < angle_tolerance_deg)
                {
                    ROS_INFO("Alignment complete: current=%.2f, target=%.2f, error=%.2f",
                             current_yaw_, target_angle_deg_, error);
                    success = true;
                    break;
                }
            }
            else
            {
                // For clockwise/counterclockwise modes
                double target_amount = fabs(target_angle_deg_);
                double remaining = target_amount - accumulated_angle;

                // Error is the remaining angle to rotate (with direction)
                error = (direction_ == "clockwise") ? -remaining : remaining;

                // Debug info
                ROS_INFO_THROTTLE(0.5, "Rotation mode: %s, accumulated=%.2f, target=%.2f, remaining=%.2f",
                                  direction_.c_str(), accumulated_angle, target_amount, remaining);

                // Success condition for normal rotation modes
                if (accumulated_angle >= (target_amount - angle_tolerance_deg))
                {
                    ROS_INFO("Rotation complete: accumulated=%.2f, target=%.2f",
                             accumulated_angle, target_amount);
                    success = true;
                    break;
                }
            }

            // Use PID controller to calculate angular velocity
            pid_input(&pid_controller_, error);
            pid_calculate(&pid_controller_);
            double angular_vel = pid_output(&pid_controller_);

            // Apply limits to angular velocity
            if (fabs(angular_vel) > angular_speed_)
            {
                angular_vel = angular_vel > 0 ? angular_speed_ : -angular_speed_;
            }

            // Apply minimum angular velocity for small errors to overcome friction
            // Only if we're not very close to the target
            if (fabs(error) > angle_tolerance_deg * 2 && fabs(angular_vel) < 0.1)
            {
                angular_vel = error > 0 ? 0.1 : -0.1;
            }

            // Send command to robot
            rotateRobot(angular_vel);

            // Update feedback
            if (direction_ == "align")
            {
                feedback_.current_angle = error; // For align mode, feedback is the current error
            }
            else
            {
                feedback_.current_angle = accumulated_angle; // For rotation modes, feedback is accumulated angle
            }
            as_.publishFeedback(feedback_);

            ros::spinOnce();
            r.sleep();
        }

        // Stop the robot when done
        stopRobot();

        if (success)
        {
            result_.success = true;
            result_.final_angle = feedback_.current_angle;
            ROS_INFO("%s: Succeeded, final angle: %.2f degrees", action_name_.c_str(), result_.final_angle);
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