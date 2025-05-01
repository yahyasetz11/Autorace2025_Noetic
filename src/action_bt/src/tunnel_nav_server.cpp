#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/TunnelNavAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>
#include <cmath>
#include <fstream>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <cstdlib> // For system()

class TunnelNavServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::TunnelNavAction> as_;
    std::string action_name_;

    // Action feedback and result
    msg_file::TunnelNavFeedback feedback_;
    msg_file::TunnelNavResult result_;

    // ROS communication
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Publisher goal_pub_; // Publisher for move_base_simple/goal
    ros::ServiceClient map_saver_client_;

    // State tracking
    bool wall_detected_;
    bool slam_initialized_;
    bool navigation_started_;
    ros::Time start_time_;
    std::string current_mode_;
    std::string map_file_path_;

    // YAML configuration
    YAML::Node config;

    // Configuration parameters loaded from YAML
    double wall_distance_threshold_;
    int wall_detection_angle;
    double initial_linear_speed;
    double max_linear_speed;
    double max_angular_speed;
    std::string maps_folder_name;
    std::string default_map_name;
    bool map_save_enabled;
    double navigation_timeout;

    // Goal position tracking
    geometry_msgs::PoseStamped goal_pose_;
    bool goal_reached_;

    // TF listener for position tracking
    tf::TransformListener tf_listener_;

    // List of running processes
    std::vector<std::string> running_processes_;

public:
    TunnelNavServer(std::string name) : as_(nh_, name, boost::bind(&TunnelNavServer::executeCB, this, _1), false),
                                        action_name_(name),
                                        wall_detected_(false),
                                        slam_initialized_(false),
                                        navigation_started_(false),
                                        goal_reached_(false),
                                        wall_distance_threshold_(0.5),
                                        wall_detection_angle(270),
                                        initial_linear_speed(0.2),
                                        max_linear_speed(0.2),
                                        max_angular_speed(1.0),
                                        maps_folder_name("maps"),
                                        default_map_name("tunnel_map"),
                                        map_save_enabled(true),
                                        navigation_timeout(300.0)
    {
        // Load YAML configuration file
        std::string config_path = ros::package::getPath("config") + "/tunnel_param.yaml";
        ROS_INFO("Loading configuration from: %s", config_path.c_str());

        try
        {
            config = YAML::LoadFile(config_path);

            // Load parameters from YAML
            if (config["tunnel_navigation"])
            {
                auto tunnel_config = config["tunnel_navigation"];

                // Wall detection parameters
                if (tunnel_config["wall_distance_threshold"])
                    wall_distance_threshold_ = tunnel_config["wall_distance_threshold"].as<double>();
                if (tunnel_config["wall_detection_angle"])
                    wall_detection_angle = tunnel_config["wall_detection_angle"].as<int>();

                // Velocity parameters
                if (tunnel_config["initial_linear_speed"])
                    initial_linear_speed = tunnel_config["initial_linear_speed"].as<double>();
                if (tunnel_config["max_linear_speed"])
                    max_linear_speed = tunnel_config["max_linear_speed"].as<double>();
                if (tunnel_config["max_angular_speed"])
                    max_angular_speed = tunnel_config["max_angular_speed"].as<double>();

                // Map parameters
                if (tunnel_config["maps_folder"])
                    maps_folder_name = tunnel_config["maps_folder"].as<std::string>();
                if (tunnel_config["default_map_name"])
                    default_map_name = tunnel_config["default_map_name"].as<std::string>();
                if (tunnel_config["map_save_enabled"])
                    map_save_enabled = tunnel_config["map_save_enabled"].as<bool>();

                // Navigation parameters
                if (tunnel_config["navigation_timeout"])
                    navigation_timeout = tunnel_config["navigation_timeout"].as<double>();
            }

            ROS_INFO("YAML configuration loaded successfully");
        }
        catch (const YAML::Exception &e)
        {
            ROS_WARN("Failed to load YAML configuration: %s. Using default values.", e.what());
        }

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        // Start action server
        as_.start();
        ROS_INFO("Tunnel Navigation Action Server Started");

        // Log configuration
        ROS_INFO("Configuration parameters:");
        ROS_INFO("  wall_distance_threshold: %.2f m", wall_distance_threshold_);
        ROS_INFO("  wall_detection_angle: %d degrees", wall_detection_angle);
        ROS_INFO("  initial_linear_speed: %.2f m/s", initial_linear_speed);
        ROS_INFO("  max_linear_speed: %.2f m/s", max_linear_speed);
        ROS_INFO("  max_angular_speed: %.2f rad/s", max_angular_speed);
        ROS_INFO("  maps_folder: %s", maps_folder_name.c_str());
        ROS_INFO("  default_map_name: %s", default_map_name.c_str());
        ROS_INFO("  map_save_enabled: %s", map_save_enabled ? "true" : "false");
        ROS_INFO("  navigation_timeout: %.2f s", navigation_timeout);
    }

    ~TunnelNavServer()
    {
        // Stop the robot
        stopRobot();

        // No need to manually kill processes as they'll be managed by ROS
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        // Calculate the index based on the wall_detection_angle parameter
        // Convert angle from degrees to index in the laser scan array

        // Calculate the normalized angle (0-360) for detection
        double angle_rad = wall_detection_angle * M_PI / 180.0;

        // Calculate index based on scan angle_min, angle_max and angle_increment
        int detection_idx = (angle_rad - scan->angle_min) / scan->angle_increment;

        // Ensure the index is valid
        if (detection_idx >= 0 && detection_idx < scan->ranges.size())
        {
            float distance = scan->ranges[detection_idx];

            // Check if the distance is valid (not inf or nan) and less than threshold
            if (!std::isnan(distance) &&
                !std::isinf(distance) &&
                distance <= wall_distance_threshold_)
            {
                wall_detected_ = true;
                ROS_INFO("Wall detected at angle %d degrees at distance: %.2f m",
                         wall_detection_angle, distance);
            }
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Wall detection angle index out of range: %d (max: %lu)",
                              detection_idx, scan->ranges.size() - 1);
        }
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Robot stopped");
    }

    void moveForward()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = initial_linear_speed;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    void launchProcess(const std::string &command)
    {
        ROS_INFO("Executing: %s", command.c_str());

        // Use system() to run the command
        int result = system(command.c_str());

        if (result != 0)
        {
            ROS_WARN("Command execution returned non-zero status: %d", result);
        }
        else
        {
            // Keep track of running processes
            running_processes_.push_back(command);
        }
    }

    void initializeOnlineSLAM()
    {
        ROS_INFO("Initializing online SLAM mode");

        // Launch gmapping
        std::string gmapping_cmd = "roslaunch turtlebot3_slam turtlebot3_gmapping.launch";
        launchProcess(gmapping_cmd);

        // Launch move_base with the default turtlebot3 configuration
        std::string move_base_cmd = "roslaunch turtlebot3_navigation move_base.launch";
        launchProcess(move_base_cmd);

        slam_initialized_ = true;
        ros::Duration(2.0).sleep(); // Give time for nodes to start
    }

    void initializeOfflineSLAM(const std::string &map_file)
    {
        ROS_INFO("Initializing offline navigation mode with map: %s", map_file.c_str());

        // Launch map server with the specified map
        std::string map_server_cmd = "roslaunch turtlebot3_navigation map_server.launch map_file:=" + map_file;
        launchProcess(map_server_cmd);

        // Launch AMCL for localization
        std::string amcl_cmd = "roslaunch turtlebot3_navigation amcl.launch";
        launchProcess(amcl_cmd);

        // Launch move_base
        std::string move_base_cmd = "roslaunch turtlebot3_navigation move_base.launch";
        launchProcess(move_base_cmd);

        slam_initialized_ = true;
        ros::Duration(2.0).sleep(); // Give time for nodes to start
    }

    void setInitialPose()
    {
        ROS_INFO("Setting initial pose");

        // Create initial pose message
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.header.frame_id = "map";

        // Set position to origin
        initial_pose.pose.pose.position.x = 0.0;
        initial_pose.pose.pose.position.y = 0.0;
        initial_pose.pose.pose.position.z = 0.0;

        // Set orientation to identity (forward)
        initial_pose.pose.pose.orientation.w = 1.0;
        initial_pose.pose.pose.orientation.x = 0.0;
        initial_pose.pose.pose.orientation.y = 0.0;
        initial_pose.pose.pose.orientation.z = 0.0;

        // Set covariance (low uncertainty)
        for (int i = 0; i < 36; i++)
        {
            initial_pose.pose.covariance[i] = 0.0;
        }
        initial_pose.pose.covariance[0] = 0.25;  // x variance
        initial_pose.pose.covariance[7] = 0.25;  // y variance
        initial_pose.pose.covariance[35] = 0.06; // yaw variance

        // Publish initial pose
        initial_pose_pub_.publish(initial_pose);

        // Wait for AMCL to process the initial pose
        ros::Duration(1.0).sleep();

        ROS_INFO("Initial pose set");
    }

    void sendGoalPose()
    {
        // Get goal pose from config
        double goal_x = 2.0, goal_y = 0.0, goal_yaw = 0.0; // defaults
        if (config["tunnel_navigation"] && config["tunnel_navigation"]["default_goal"])
        {
            auto goal_config = config["tunnel_navigation"]["default_goal"];
            if (goal_config["x"])
                goal_x = goal_config["x"].as<double>();
            if (goal_config["y"])
                goal_y = goal_config["y"].as<double>();
            if (goal_config["yaw"])
                goal_yaw = goal_config["yaw"].as<double>();
        }

        // Create goal pose message
        goal_pose_.header.frame_id = "map";
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x = goal_x;
        goal_pose_.pose.position.y = goal_y;
        goal_pose_.pose.position.z = 0.0;

        // Convert yaw to quaternion
        tf::Quaternion q;
        q.setRPY(0, 0, goal_yaw);
        goal_pose_.pose.orientation.x = q.x();
        goal_pose_.pose.orientation.y = q.y();
        goal_pose_.pose.orientation.z = q.z();
        goal_pose_.pose.orientation.w = q.w();

        // Publish goal pose
        goal_pub_.publish(goal_pose_);
        navigation_started_ = true;

        ROS_INFO("Navigation goal sent: x=%.2f, y=%.2f, yaw=%.2f",
                 goal_x, goal_y, goal_yaw);
    }

    double getDistanceToGoal()
    {
        try
        {
            // Get the current robot position
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            // Calculate distance to goal
            double dx = goal_pose_.pose.position.x - transform.getOrigin().x();
            double dy = goal_pose_.pose.position.y - transform.getOrigin().y();
            double distance = std::sqrt(dx * dx + dy * dy);

            // Check if goal is reached (within 0.2m)
            if (distance < 0.2)
            {
                goal_reached_ = true;
                ROS_INFO("Goal reached! Distance: %.2f m", distance);
            }

            return distance;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failed to get robot position: %s", ex.what());
            return -1.0;
        }
    }

    std::string findNextMapFilename(const std::string &base_path)
    {
        // Get base filename from configuration
        std::string base_name = default_map_name;
        std::string extension = ".yaml";
        std::string full_path = base_path + "/" + base_name + extension;

        // Check if the default name exists
        int counter = 1;
        while (boost::filesystem::exists(full_path))
        {
            full_path = base_path + "/" + base_name + "_" + std::to_string(counter) + extension;
            counter++;
        }

        return full_path;
    }

    bool saveMap()
    {
        // Check if map saving is enabled from configuration
        if (!map_save_enabled)
        {
            ROS_INFO("Map saving disabled by configuration");
            return true;
        }

        ROS_INFO("Saving map...");

        // Use maps folder from configuration
        std::string maps_folder = ros::package::getPath("turtlebot3_navigation") + "/" + maps_folder_name;

        // Create maps directory if it doesn't exist
        boost::filesystem::create_directories(maps_folder);

        // Get unique map filename
        std::string map_filename = findNextMapFilename(maps_folder);

        // Extract base filename without path and extension
        size_t last_slash = map_filename.find_last_of("/");
        size_t last_dot = map_filename.find_last_of(".");
        std::string base_filename = map_filename.substr(
            last_slash + 1,
            last_dot - last_slash - 1);

        // Call map_saver to save the map
        std::string save_cmd = "rosrun map_server map_saver -f " +
                               maps_folder + "/" + base_filename;

        // Execute the command and wait for it to complete
        int result = system(save_cmd.c_str());

        if (result == 0)
        {
            ROS_INFO("Map saved successfully to: %s", map_filename.c_str());
            result_.map_path = map_filename;
            return true;
        }
        else
        {
            ROS_ERROR("Failed to save map, return code: %d", result);
            return false;
        }
    }

    void executeCB(const msg_file::TunnelNavGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Reset state flags
        wall_detected_ = false;
        slam_initialized_ = false;
        navigation_started_ = false;
        goal_reached_ = false;

        // Get mode and map_file from goal
        current_mode_ = goal->mode;
        map_file_path_ = goal->map_file;

        // If in offline mode and no map file specified, try to get from config
        if (current_mode_ == "offline" && map_file_path_.empty())
        {
            if (config["tunnel_navigation"] && config["tunnel_navigation"]["default_map_file"])
            {
                map_file_path_ = config["tunnel_navigation"]["default_map_file"].as<std::string>();
            }

            if (map_file_path_.empty())
            {
                ROS_ERROR("Offline mode requires a map file path. None provided in goal or default_map_file config.");
                as_.setAborted();
                return;
            }
        }

        // Validate mode
        if (current_mode_ != "online" && current_mode_ != "offline")
        {
            ROS_ERROR("Invalid mode: %s. Must be 'online' or 'offline'", current_mode_.c_str());
            as_.setAborted();
            return;
        }

        // For offline mode, verify map file exists
        if (current_mode_ == "offline" && !boost::filesystem::exists(map_file_path_))
        {
            ROS_ERROR("Map file does not exist: %s", map_file_path_.c_str());
            as_.setAborted();
            return;
        }

        ROS_INFO("TunnelNav Server: Mode=%s, Wall threshold=%.2f",
                 current_mode_.c_str(), wall_distance_threshold_);

        // Subscribe to laser scan for wall detection
        laser_sub_ = nh_.subscribe("/scan", 1, &TunnelNavServer::laserCallback, this);

        // PHASE 1: Move forward until wall detected on right side
        ROS_INFO("Phase 1: Moving forward until wall detected on right side");
        feedback_.current_state = "MOVING_TO_WALL";

        while (ros::ok() && !wall_detected_)
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                stopRobot();
                return;
            }

            // Move forward
            moveForward();

            // Update feedback
            feedback_.distance_to_goal = -1.0; // Not applicable yet
            as_.publishFeedback(feedback_);

            ros::spinOnce();
            r.sleep();
        }

        // Stop the robot after wall detection
        stopRobot();
        ROS_INFO("Wall detected on right side, stopping robot");

        // PHASE 2: Initialize SLAM based on mode
        ROS_INFO("Phase 2: Initializing SLAM/Navigation");
        feedback_.current_state = "INITIALIZING_SLAM";
        as_.publishFeedback(feedback_);

        if (current_mode_ == "online")
        {
            initializeOnlineSLAM();
        }
        else
        { // offline mode
            initializeOfflineSLAM(map_file_path_);
        }

        // PHASE 3: Set initial pose
        feedback_.current_state = "SETTING_INITIAL_POSE";
        as_.publishFeedback(feedback_);
        setInitialPose();
        ros::Duration(1.0).sleep(); // Wait for pose to be processed

        // PHASE 4: Send navigation goal
        feedback_.current_state = "SENDING_GOAL";
        as_.publishFeedback(feedback_);
        sendGoalPose();

        // PHASE 5: Monitor navigation progress
        feedback_.current_state = "NAVIGATING";
        ros::Time navigation_start_time = ros::Time::now();

        while (ros::ok() && !goal_reached_)
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                stopRobot();
                return;
            }

            // Check timeout
            double elapsed = (ros::Time::now() - navigation_start_time).toSec();
            if (elapsed > navigation_timeout)
            {
                ROS_ERROR("Navigation timeout after %.1f seconds", elapsed);
                success = false;
                break;
            }

            // Monitor distance to goal
            double distance = getDistanceToGoal();
            if (distance >= 0)
            {
                feedback_.distance_to_goal = distance;
                as_.publishFeedback(feedback_);
            }

            // Check if goal reached
            if (goal_reached_)
            {
                ROS_INFO("Goal reached successfully");
                success = true;
                break;
            }

            ros::spinOnce();
            r.sleep();
        }

        // PHASE 6: Save map if in online mode
        if (success && current_mode_ == "online")
        {
            feedback_.current_state = "SAVING_MAP";
            as_.publishFeedback(feedback_);
            saveMap();
        }

        // Set result
        result_.success = success;

        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
        else
        {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tunnel_nav_server");
    TunnelNavServer tunnelNavServer("tunnel_nav");
    ros::spin();
    return 0;
}