#include <signal.h>
#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/TunnelNavAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <fstream>
#include <vector>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <stdlib.h>

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
    ros::Publisher cmd_vel_pub_;
    ros::Publisher goal_pub_; // Publisher for move_base/goal

    // State tracking
    bool navigation_started_;
    bool current_goal_reached_;
    ros::Time start_time_;
    std::string current_mode_;
    std::string map_base_path_;

    // Process tracking
    pid_t slam_init_process_id_;

    // YAML configuration
    YAML::Node config;

    // Configuration parameters loaded from YAML
    double navigation_timeout;
    int goal_total;
    std::vector<std::vector<double>> goal_poses; // [x, y, yaw]

    // Goal position tracking
    std::vector<geometry_msgs::PoseStamped> goal_pose_list_;
    int current_goal_index_;
    double finish_area_width_;       // Width of the square area around goal point
    double finish_area_timeout_;     // Timeout in seconds
    ros::Time in_finish_area_since_; // Timestamp when robot entered finish area
    bool in_finish_area_;            // Flag to track if robot is in finish area

    // TF listener for position tracking
    tf::TransformListener tf_listener_;

public:
    TunnelNavServer(std::string name) : as_(nh_, name, boost::bind(&TunnelNavServer::executeCB, this, _1), false),
                                        action_name_(name),
                                        navigation_started_(false),
                                        current_goal_reached_(false),
                                        current_goal_index_(0),
                                        slam_init_process_id_(-1),
                                        navigation_timeout(300.0),
                                        goal_total(1),
                                        map_base_path_("~/Documents/Autorace2025_Noetic/src/maps"),
                                        finish_area_width_(0.4),   // Default 0.4m square area
                                        finish_area_timeout_(5.0), // Default 5 seconds
                                        in_finish_area_(false)     // Initially not in finish area

    {
        // Load YAML configuration file
        std::string config_path = ros::package::getPath("action_bt") + "/../config/tunnel_param.yaml";
        ROS_INFO("Loading configuration from: %s", config_path.c_str());

        try
        {
            config = YAML::LoadFile(config_path);

            // Load parameters from YAML
            if (config["tunnel_navigation"])
            {
                auto tunnel_config = config["tunnel_navigation"];

                // Navigation parameters
                if (tunnel_config["navigation_timeout"])
                    navigation_timeout = tunnel_config["navigation_timeout"].as<double>();

                // Multiple goal parameters
                if (tunnel_config["goal_total"])
                    goal_total = tunnel_config["goal_total"].as<int>();

                // Load goal poses
                if (tunnel_config["goal_poses"] && tunnel_config["goal_poses"].IsSequence())
                {
                    auto poses = tunnel_config["goal_poses"];
                    for (size_t i = 0; i < poses.size(); i++)
                    {
                        if (poses[i].IsSequence() && poses[i].size() == 3)
                        {
                            std::vector<double> pose = {
                                poses[i][0].as<double>(), // x
                                poses[i][1].as<double>(), // y
                                poses[i][2].as<double>()  // yaw
                            };
                            goal_poses.push_back(pose);
                        }
                    }
                }
                else if (tunnel_config["default_goal"])
                {
                    // For backward compatibility, use default_goal if goal_poses not found
                    auto goal_config = tunnel_config["default_goal"];
                    if (goal_config["x"] && goal_config["y"] && goal_config["yaw"])
                    {
                        std::vector<double> pose = {
                            goal_config["x"].as<double>(),
                            goal_config["y"].as<double>(),
                            goal_config["yaw"].as<double>()};
                        goal_poses.push_back(pose);
                    }
                }

                // Map base path
                if (tunnel_config["map_base_path"])
                    map_base_path_ = tunnel_config["map_base_path"].as<std::string>();

                if (tunnel_config["finish_area_width"])
                    finish_area_width_ = tunnel_config["finish_area_width"].as<double>();

                if (tunnel_config["finish_area_timeout"])
                    finish_area_timeout_ = tunnel_config["finish_area_timeout"].as<double>();
            }

            ROS_INFO("YAML configuration loaded successfully");
        }
        catch (const YAML::Exception &e)
        {
            ROS_WARN("Failed to load YAML configuration: %s. Using default values.", e.what());

            // Set default goal if not loaded from YAML
            if (goal_poses.empty())
            {
                goal_poses.push_back({2.0, 0.0, 0.0}); // Default goal
            }
        }

        // Verify we have at least one goal
        if (goal_total < 1)
        {
            goal_total = 1;
            ROS_WARN("goal_total must be at least 1, setting to 1");
        }

        // Ensure we have enough goal poses
        if (goal_poses.size() < goal_total)
        {
            ROS_WARN("Not enough goal poses defined. Expected %d, got %lu. Will use the last pose for remaining goals.",
                     goal_total, goal_poses.size());

            // Fill with the last valid pose
            std::vector<double> last_pose = goal_poses.back();
            while (goal_poses.size() < goal_total)
            {
                goal_poses.push_back(last_pose);
            }
        }

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        // Convert goal_poses to PoseStamped messages for later use
        for (const auto &pose : goal_poses)
        {
            geometry_msgs::PoseStamped goal_msg;
            goal_msg.header.frame_id = "map";
            goal_msg.pose.position.x = pose[0];
            goal_msg.pose.position.y = pose[1];
            goal_msg.pose.position.z = 0.0;

            // Convert yaw to quaternion
            tf::Quaternion q;
            q.setRPY(0, 0, pose[2]);
            goal_msg.pose.orientation.x = q.x();
            goal_msg.pose.orientation.y = q.y();
            goal_msg.pose.orientation.z = q.z();
            goal_msg.pose.orientation.w = q.w();

            goal_pose_list_.push_back(goal_msg);
        }

        // Start action server
        as_.start();
        ROS_INFO("Tunnel Navigation Action Server Started");

        // Log configuration
        ROS_INFO("Configuration parameters:");
        ROS_INFO("  navigation_timeout: %.2f s", navigation_timeout);
        ROS_INFO("  goal_total: %d", goal_total);
        ROS_INFO("  map_base_path: %s", map_base_path_.c_str());

        // Log goal poses
        for (size_t i = 0; i < goal_poses.size(); i++)
        {
            ROS_INFO("  Goal %lu: x=%.2f, y=%.2f, yaw=%.2f",
                     i + 1, goal_poses[i][0], goal_poses[i][1], goal_poses[i][2]);
        }
    }

    ~TunnelNavServer()
    {
        // Stop the robot
        stopRobot();

        // Clean up SLAM processes
        cleanupSLAMProcesses();
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Robot stopped");
    }

    void sendGoalPose(int goal_index)
    {
        if (goal_index >= 0 && goal_index < goal_pose_list_.size())
        {
            // Update timestamp
            goal_pose_list_[goal_index].header.stamp = ros::Time::now();

            // Publish goal pose
            goal_pub_.publish(goal_pose_list_[goal_index]);
            navigation_started_ = true;
            current_goal_reached_ = false;

            double x = goal_pose_list_[goal_index].pose.position.x;
            double y = goal_pose_list_[goal_index].pose.position.y;

            // Extract yaw from quaternion
            tf::Quaternion q(
                goal_pose_list_[goal_index].pose.orientation.x,
                goal_pose_list_[goal_index].pose.orientation.y,
                goal_pose_list_[goal_index].pose.orientation.z,
                goal_pose_list_[goal_index].pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            ROS_INFO("Navigation goal %d sent: x=%.2f, y=%.2f, yaw=%.2f",
                     goal_index + 1, x, y, yaw);
        }
        else
        {
            ROS_ERROR("Invalid goal index: %d", goal_index);
        }
    }

    double getDistanceToGoal(int goal_index)
    {
        if (goal_index < 0 || goal_index >= goal_pose_list_.size())
        {
            ROS_ERROR("Invalid goal index for distance calculation: %d", goal_index);
            return -1.0;
        }

        try
        {
            // Get the current robot position
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            // Calculate distance to goal
            double dx = goal_pose_list_[goal_index].pose.position.x - transform.getOrigin().x();
            double dy = goal_pose_list_[goal_index].pose.position.y - transform.getOrigin().y();
            double distance = std::sqrt(dx * dx + dy * dy);

            // Check if robot is within finish area (square)
            bool in_area_now = (std::abs(dx) <= finish_area_width_ / 2.0) &&
                               (std::abs(dy) <= finish_area_width_ / 2.0);

            // Check if goal is reached (within 0.2m)
            // if (distance < 0.2)
            // {
            //     current_goal_reached_ = true;
            //     ROS_INFO("Goal %d reached! Distance: %.2f m", goal_index + 1, distance);
            //     in_finish_area_ = false;
            // }
            // else
            if (in_area_now)
            {
                if (!in_finish_area_)
                {
                    // Just entered the finish area, start the timer
                    in_finish_area_ = true;
                    in_finish_area_since_ = ros::Time::now();
                    ROS_INFO("Robot entered finish area for goal %d. Distance: %.2f m",
                             goal_index + 1, distance);
                }
                else
                {
                    // Already in finish area, check timeout
                    double time_in_area = (ros::Time::now() - in_finish_area_since_).toSec();

                    if (time_in_area >= finish_area_timeout_)
                    {
                        // Timeout reached, consider goal achieved
                        current_goal_reached_ = true;
                        ROS_INFO("Goal %d reached by finish area timeout! Distance: %.2f m, Time in area: %.2f s",
                                 goal_index + 1, distance, time_in_area);
                        in_finish_area_ = false; // Reset finish area flag

                        // Add a brief pause to ensure status updates propagate
                        ros::Duration(0.5).sleep();
                    }
                    else
                    {
                        // Still in area but timeout not reached yet
                        ROS_INFO_THROTTLE(1.0, "Robot in finish area for %.2f s. Need %.2f s more to consider goal reached.",
                                          time_in_area, finish_area_timeout_ - time_in_area);
                    }
                }
            }
            else
            {
                // Not in finish area
                if (in_finish_area_)
                {
                    // Just left the finish area
                    in_finish_area_ = false;
                    ROS_INFO("Robot left finish area for goal %d before timeout.", goal_index + 1);
                }
            }

            return distance;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failed to get robot position: %s", ex.what());
            return -1.0;
        }
    }

    // Initialize SLAM based on mode
    bool initializeSLAM(const std::string &mode)
    {
        ROS_INFO("Initializing SLAM for mode: %s", mode.c_str());

        if (mode == "online")
        {
            // Launch online SLAM (gmapping) - run in background with &
            std::string cmd = "roslaunch launch_file gmapping_online.launch & echo $! > /tmp/slam_pid";
            int ret = system(cmd.c_str());

            // Read the PID from the temp file
            std::ifstream pid_file("/tmp/slam_pid");
            pid_file >> slam_init_process_id_;
            pid_file.close();

            if (ret != 0)
            {
                ROS_ERROR("Failed to launch online SLAM: %s (return code: %d)", cmd.c_str(), ret);
                return false;
            }
            ROS_INFO("Online SLAM initialization launched in background (PID: %d)", slam_init_process_id_);
            return true;
        }
        else if (mode == "offline")
        {
            // Launch offline SLAM (map server + AMCL + move_base) - run in background with &
            std::string map_file = "map_file:=$HOME/Documents/Autorace2025_Noetic/src/maps/map6.yaml";
            std::string cmd = "roslaunch launch_file localization_offline.launch " + map_file + " & echo $! > /tmp/slam_pid";
            int ret = system(cmd.c_str());

            // Read the PID from the temp file
            std::ifstream pid_file("/tmp/slam_pid");
            pid_file >> slam_init_process_id_;
            pid_file.close();

            if (ret != 0)
            {
                ROS_ERROR("Failed to launch offline SLAM: %s (return code: %d)", cmd.c_str(), ret);
                return false;
            }
            ROS_INFO("Offline SLAM initialization launched in background (PID: %d)", slam_init_process_id_);
            return true;
        }
        else
        {
            ROS_ERROR("Invalid SLAM mode: %s", mode.c_str());
            return false;
        }
    }

    // Find next available map filename
    std::string findNextMapFilename()
    {
        std::string base_path = map_base_path_;
        // Replace ~ with actual home path if needed
        if (base_path.find("~") == 0)
        {
            const char *home = getenv("HOME");
            if (home)
            {
                base_path.replace(0, 1, home);
            }
        }

        int counter = 1;
        std::string map_path;

        // Find next available filename
        while (true)
        {
            map_path = base_path + "_" + std::to_string(counter);
            // Check if .yaml file exists
            if (!boost::filesystem::exists(map_path + ".yaml") &&
                !boost::filesystem::exists(map_path + ".pgm"))
            {
                break;
            }
            counter++;
        }

        return map_path;
    }

    // Save map for online mode
    bool saveMap()
    {
        std::string map_path = findNextMapFilename();
        ROS_INFO("Saving map to: %s", map_path.c_str());

        std::string cmd = "rosrun map_server map_saver -f " + map_path;
        int ret = system(cmd.c_str());
        if (ret != 0)
        {
            ROS_ERROR("Failed to save map: %s (return code: %d)", cmd.c_str(), ret);
            return false;
        }

        // Store the map path in the result
        result_.map_path = map_path + ".yaml";
        return true;
    }

    // Add a cleanup method to properly terminate all processes
    void cleanupSLAMProcesses()
    {
        // Kill SLAM initialization process if still running
        if (slam_init_process_id_ > 0)
        {
            ROS_INFO("Terminating SLAM initialization process (PID: %d)", slam_init_process_id_);
            kill(slam_init_process_id_, SIGTERM);

            // Wait a moment for the process to terminate gracefully
            ros::Duration(1.0).sleep();

            // Check if still running and force kill if necessary
            std::string check_cmd = "ps -p " + std::to_string(slam_init_process_id_) + " > /dev/null";
            if (system(check_cmd.c_str()) == 0)
            {
                ROS_WARN("Process didn't terminate gracefully, using SIGKILL");
                kill(slam_init_process_id_, SIGKILL);
            }

            slam_init_process_id_ = -1;
        }

        // For "online" mode, ensure gmapping is stopped
        if (current_mode_ == "online")
        {
            ROS_INFO("Ensuring gmapping is terminated");
            system("rosnode kill /slam_gmapping 2>/dev/null || true");
            system("pkill -f gmapping_node 2>/dev/null || true");
        }

        // For "offline" mode, ensure move_base and amcl are stopped
        else if (current_mode_ == "offline")
        {
            ROS_INFO("Ensuring move_base and amcl are terminated");
            system("rosnode kill /move_base 2>/dev/null || true");
            system("rosnode kill /amcl 2>/dev/null || true");
        }

        // Make sure cmd_vel is zeroed out
        stopRobot();

        // Brief pause to allow ROS system to update
        ros::Duration(1.0).sleep();
    }

    // Check if TF frames are available (to verify SLAM is running)
    bool waitForTFFrames(double timeout = 60.0)
    {
        ROS_INFO("Waiting for TF frames to become available...");
        ros::Time start = ros::Time::now();

        while (ros::ok())
        {
            try
            {
                // Check if we can transform between base_footprint and map
                if (tf_listener_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0)))
                {
                    ROS_INFO("TF frames are available - SLAM initialized successfully");
                    return true;
                }
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN_THROTTLE(5.0, "TF not available yet: %s", ex.what());
            }

            // Check for timeout
            if ((ros::Time::now() - start).toSec() > timeout)
            {
                ROS_ERROR("Timed out waiting for TF frames after %.1f seconds", timeout);
                return false;
            }

            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }

        return false;
    }

    void executeCB(const msg_file::TunnelNavGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Reset state flags
        navigation_started_ = false;
        current_goal_reached_ = false;
        current_goal_index_ = 0;
        in_finish_area_ = false;

        // Get mode from goal
        current_mode_ = goal->mode;

        // Validate mode
        if (current_mode_ != "online" && current_mode_ != "offline")
        {
            ROS_ERROR("Invalid mode: %s. Must be 'online' or 'offline'", current_mode_.c_str());
            as_.setAborted();
            return;
        }

        ROS_INFO("TunnelNav Server: Mode=%s", current_mode_.c_str());

        // PHASE 1: Initialize SLAM based on mode
        ROS_INFO("Phase 1: Initializing SLAM/Navigation");
        feedback_.current_state = "INITIALIZING_SLAM";
        as_.publishFeedback(feedback_);

        if (!initializeSLAM(current_mode_))
        {
            ROS_ERROR("Failed to initialize SLAM");
            as_.setAborted();
            return;
        }

        // Give SLAM time to initialize
        ros::Duration(5.0).sleep();

        // PHASE 2: Verify SLAM initialization
        if (!waitForTFFrames(60.0))
        {
            ROS_ERROR("SLAM initialization failed - TF frames not available");
            as_.setAborted();
            return;
        }

        // PHASE 3: Navigate to each goal sequentially
        for (current_goal_index_ = 0; current_goal_index_ < goal_total; current_goal_index_++)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                cleanupSLAMProcesses(); // Add cleanup here
                as_.setPreempted();
                stopRobot();
                return;
            }

            // Send goal
            feedback_.current_state = "SENDING_GOAL";
            feedback_.distance_to_goal = -1.0;
            as_.publishFeedback(feedback_);

            sendGoalPose(current_goal_index_);

            // Monitor navigation progress
            feedback_.current_state = "NAVIGATING_TO_GOAL_" + std::to_string(current_goal_index_ + 1);
            ros::Time navigation_start_time = ros::Time::now();
            current_goal_reached_ = false;

            while (ros::ok() && !current_goal_reached_)
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
                    ROS_ERROR("Navigation timeout after %.1f seconds for goal %d",
                              elapsed, current_goal_index_ + 1);
                    success = false;
                    break;
                }

                // Monitor distance to goal
                double distance = getDistanceToGoal(current_goal_index_);
                if (distance >= 0)
                {
                    feedback_.distance_to_goal = distance;
                    as_.publishFeedback(feedback_);
                }

                // Check if goal reached
                if (current_goal_reached_)
                {
                    ROS_INFO("Goal %d reached successfully", current_goal_index_ + 1);

                    // Pause briefly at each goal
                    ros::Duration(1.0).sleep();
                    break;
                }

                ros::spinOnce();
                r.sleep();
            }

            // If current goal failed, abort entire mission
            if (!current_goal_reached_)
            {
                ROS_ERROR("Failed to reach goal %d, aborting mission", current_goal_index_ + 1);
                success = false;
                break;
            }
        }

        // PHASE 4: Save map if in online mode and successful
        if (success && current_mode_ == "online")
        {
            feedback_.current_state = "SAVING_MAP";
            as_.publishFeedback(feedback_);

            if (!saveMap())
            {
                ROS_WARN("Failed to save map, but navigation was successful");
                // Don't fail the mission just because map saving failed
            }
        }

        // Ensure all processes are cleaned up regardless of success or failure path
        cleanupSLAMProcesses();

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