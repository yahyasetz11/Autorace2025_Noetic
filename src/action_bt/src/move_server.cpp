#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/MoveAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <yaml-cpp/yaml.h>

// Define constants
#define SCAN_TIMEOUT 1.0 // Scan timeout in seconds

class MoveServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::MoveAction> as_;
    std::string action_name_;
    msg_file::MoveFeedback feedback_;
    msg_file::MoveResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scan_sub_;

    // Parameters
    double linear_speed_;

    // Latest scan data
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_;
    ros::Time last_scan_time_;

    // Action parameters
    std::string mode_;
    std::string until_;
    float distance_threshold_;

public:
    MoveServer(std::string name) : as_(nh_, name, boost::bind(&MoveServer::executeCB, this, _1), false),
                                   action_name_(name),
                                   scan_received_(false),
                                   linear_speed_(0.1) // Default value in case config loading fails
    {
        // Load speed from config file
        loadSpeedConfig();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        scan_sub_ = nh_.subscribe("/scan", 1, &MoveServer::scanCallback, this);

        as_.start();
        ROS_INFO("Move Action Server Started with linear_speed: %.2f", linear_speed_);
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

            // Extract linear speed
            if (config["linear_speed"])
            {
                linear_speed_ = config["linear_speed"].as<double>();
                ROS_INFO("Loaded linear_speed = %.2f from config file", linear_speed_);
            }
            else
            {
                ROS_WARN("Could not find 'linear_speed' in config file, using default: %.2f", linear_speed_);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error loading config file: %s", e.what());
            ROS_WARN("Using default linear_speed: %.2f", linear_speed_);
        }
    }

    ~MoveServer()
    {
        // Ensure the robot stops when the server shuts down
        stopRobot();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        latest_scan_ = *msg;
        scan_received_ = true;
        last_scan_time_ = ros::Time::now();
    }

    double getDistanceInDirection(const std::string &direction)
    {
        if (!scan_received_)
        {
            ROS_WARN_THROTTLE(1, "No laser scan data received yet!");
            return -1.0; // Invalid distance
        }

        // Check if scan data is too old
        if ((ros::Time::now() - last_scan_time_).toSec() > SCAN_TIMEOUT)
        {
            ROS_WARN_THROTTLE(1, "Laser scan data is too old! Last received: %.2f seconds ago",
                              (ros::Time::now() - last_scan_time_).toSec());
            return -1.0; // Invalid distance
        }

        // Initialize with a large value instead of max double
        double min_distance = 99.0; // Default "very far" value for no obstacles
        bool any_valid_reading = false;

        // Get ranges size
        int ranges_size = latest_scan_.ranges.size();

        // Define angle range based on direction
        int start_angle, end_angle;

        if (direction == "front")
        {
            // Front is around 0 degrees
            start_angle = -15;
            end_angle = 15;
        }
        else if (direction == "back")
        {
            // Back is around 180 degrees
            start_angle = 165;
            end_angle = 195;
        }
        else if (direction == "left")
        {
            // Left is around 90 degrees
            start_angle = 75;
            end_angle = 105;
        }
        else if (direction == "right")
        {
            // Right is around 270 degrees
            start_angle = 255;
            end_angle = 285;
        }
        else
        {
            ROS_ERROR("Invalid direction: %s", direction.c_str());
            return -1.0;
        }

        // Convert angles to indices and make sure they're within range
        int start_idx = (start_angle + 360) % 360;
        int end_idx = (end_angle + 360) % 360;

        // Convert to laser scan indices (assuming 360 scans spanning 360 degrees)
        start_idx = (start_idx * ranges_size) / 360;
        end_idx = (end_idx * ranges_size) / 360;

        // Find minimum distance in the angle range
        if (start_idx <= end_idx)
        {
            for (int i = start_idx; i <= end_idx; i++)
            {
                if (i >= 0 && i < ranges_size)
                {
                    if (std::isfinite(latest_scan_.ranges[i]))
                    {
                        min_distance = std::min(min_distance, static_cast<double>(latest_scan_.ranges[i]));
                        any_valid_reading = true;
                    }
                    else if (latest_scan_.ranges[i] <= 0.01)
                    {
                        latest_scan_.ranges[i] = 99;
                    }
                }
            }
        }
        else
        {
            // Handle wrap-around case
            for (int i = start_idx; i < ranges_size; i++)
            {
                if (i >= 0 && std::isfinite(latest_scan_.ranges[i]))
                {
                    min_distance = std::min(min_distance, static_cast<double>(latest_scan_.ranges[i]));
                    any_valid_reading = true;
                }
            }
            for (int i = 0; i <= end_idx; i++)
            {
                if (i >= 0 && i < ranges_size && std::isfinite(latest_scan_.ranges[i]))
                {
                    min_distance = std::min(min_distance, static_cast<double>(latest_scan_.ranges[i]));
                    any_valid_reading = true;
                }
            }
        }

        // If no valid readings (all infinities), return the large default value
        // instead of treating it as an error
        if (!any_valid_reading)
        {
            ROS_INFO_THROTTLE(2, "No obstacles detected in direction '%s', using distance %.2f",
                              direction.c_str(), min_distance);
        }

        return min_distance;
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Stopping robot");
    }

    void moveRobot(double linear_vel)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_vel;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO_THROTTLE(0.5, "Publishing cmd_vel: linear.x = %.4f", linear_vel);
    }

    void executeCB(const msg_file::MoveGoalConstPtr &goal)
    {
        bool success = true;
        bool condition_met = false;
        ros::Rate r(10); // 10 Hz

        // Get parameters from goal
        mode_ = goal->mode;
        until_ = goal->until;
        distance_threshold_ = goal->distance;

        // Validate parameters
        if (mode_ != "forward" && mode_ != "backward")
        {
            ROS_ERROR("Invalid mode: %s. Must be 'forward' or 'backward'", mode_.c_str());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        if (until_ != "front" && until_ != "back" && until_ != "right" && until_ != "left" && until_ != "timer")
        {
            ROS_ERROR("Invalid until direction: %s. Must be 'front', 'back', 'right', 'left', or 'timer'", until_.c_str());
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Determine movement direction
        double move_speed = (mode_ == "forward") ? linear_speed_ : -linear_speed_;

        // Add this debug line after: double move_speed = (mode_ == "forward") ? linear_speed_ : -linear_speed_;
        ROS_INFO("Timer mode: move_speed = %.4f, linear_speed_ = %.4f", move_speed, linear_speed_);

        ROS_INFO("Move Server: Mode=%s, Until=%s, Distance/Time=%.2f",
                 mode_.c_str(), until_.c_str(), distance_threshold_);

        // For timer mode, we don't need scan data
        if (until_ != "timer")
        {
            // Wait until we receive scan data
            ros::Time start_time = ros::Time::now();
            while (!scan_received_ && (ros::Time::now() - start_time).toSec() < 5.0)
            {
                ROS_INFO_THROTTLE(1, "Waiting for scan data...");
                r.sleep();
            }

            if (!scan_received_)
            {
                ROS_ERROR("No scan data received after waiting 5 seconds!");
                result_.success = false;
                as_.setAborted(result_);
                return;
            }
        }

        // Special case for timer-based movement
        ros::Time timer_start;
        if (until_ == "timer")
        {
            timer_start = ros::Time::now();
            ROS_INFO("Starting timer-based movement for %.2f seconds", distance_threshold_);
        }

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

            // Check success condition based on mode
            if (until_ == "timer")
            {
                // Timer-based condition
                double elapsed_time = (ros::Time::now() - timer_start).toSec();

                // Update feedback - for timer mode, use elapsed time as the "distance"
                feedback_.current_distance = elapsed_time;
                as_.publishFeedback(feedback_);

                if (elapsed_time >= distance_threshold_)
                {
                    ROS_INFO("Timer goal reached: %.2f seconds elapsed", elapsed_time);
                    success = true;
                    break;
                }

                // Debug info
                ROS_INFO_THROTTLE(0.5, "Timer mode: %.2f / %.2f seconds elapsed",
                                  elapsed_time, distance_threshold_);
            }
            else
            {
                // Distance-based condition
                double current_distance = getDistanceInDirection(until_);

                if (current_distance < 0)
                {
                    ROS_WARN_THROTTLE(1, "Unable to get valid distance data");
                    r.sleep();
                    continue;
                }
                else if (current_distance <= 0.01)
                {
                    current_distance = 99;
                }

                // Determine success condition based on mode and until direction
                bool move_forward_until_back_front = (mode_ == "forward" && until_ == "back");
                bool move_backward_until_front_back = (mode_ == "backward" && until_ == "front");

                if (move_forward_until_back_front || move_backward_until_front_back)
                {
                    // Special case with >= comparison
                    condition_met = (current_distance >= distance_threshold_);
                }
                else
                {
                    // Normal case with <= comparison
                    condition_met = (current_distance <= distance_threshold_);
                }

                // Update feedback
                feedback_.current_distance = current_distance;
                as_.publishFeedback(feedback_);

                // Check if condition is met
                if (condition_met)
                {
                    ROS_INFO("Goal condition met: %s=%s distance %.2f %s %.2f",
                             mode_.c_str(), until_.c_str(), current_distance,
                             (move_forward_until_back_front || move_backward_until_front_back) ? ">=" : "<=",
                             distance_threshold_);
                    success = true;
                    break;
                }

                // Debug info
                ROS_INFO_THROTTLE(0.5, "Current %s distance: %.2f, Threshold: %.2f",
                                  until_.c_str(), current_distance, distance_threshold_);
            }

            // Move the robot
            moveRobot(move_speed);

            r.sleep();
        }

        // Stop the robot when done
        stopRobot();

        if (success)
        {
            result_.success = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_server");
    MoveServer moveServer("move");
    ros::spin();
    return 0;
}