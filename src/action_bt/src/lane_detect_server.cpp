#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/LaneDetectAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <cmath>                    // For math functions
#include <nav_msgs/Odometry.h>      // For odometry data
#include <tf/transform_datatypes.h> // For quaternion to Euler conversion
#include <yaml-cpp/yaml.h>          // For YAML parsing
#include <ros/package.h>            // For package path resolution
#include <fstream>                  // For file handling
#include <signal.h>                 // For process termination
#include "pid.hpp"                  // PID controller header

class LaneDetectServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::LaneDetectAction> as_;
    std::string action_name_;
    msg_file::LaneDetectFeedback feedback_;
    msg_file::LaneDetectResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber traffic_sign_sub_; // New subscriber for traffic sign
    ros::Subscriber odom_sub_;         // For odometry data
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // Variables for lane detection
    cv::Mat img_bgr_;
    cv::Mat img_hsv_;
    cv::Mat img_masked_;
    bool image_received_;
    ros::Time start_time_;
    std::string camera_topic_;

    // Sign detection variables
    int last_sign_data_;           // Last sign data received
    bool sign_detected_;           // Flag for sign detection
    std::string target_sign_;      // Target sign to wait for
    pid_t detect_sign_process_id_; // Process ID for launched sign detection

    // Lane detection variables
    std::string current_mode_;              // Current operation mode: "center", "left", "right", "intersection", etc.
    std::string actual_driving_mode_;       // The actual driving mode (used by intersection)
    double x_previous_;                     // Stored distance from lane to center
    bool both_lanes_detected_;              // Flag for detecting both lanes
    std::vector<int> recorded_left_lane_x_; // size = numRegions
    std::vector<int> recorded_right_lane_x_;

    // Yaw tracking for turn modes and intersection
    double initial_yaw_;   // Initial yaw angle when turn started
    double current_yaw_;   // Current yaw angle
    bool is_turn_mode_;    // Whether we're in a turning mode
    bool yaw_initialized_; // Whether initial yaw has been set

    // Intersection detection variables
    int intersection_flag_;          // Counter for intersection detection
    bool previous_both_lanes_;       // Previous state of both lanes detection
    ros::Time last_transition_time_; // Time of last lane detection state change

    // Intersection mode specific variables
    bool intersection_initial_turn_done_;     // Whether initial turn is done
    bool intersection_x_sign_seen_;           // Whether X sign has been seen
    int intersection_retry_count_;            // Count of retries after X sign
    std::string intersection_turn_direction_; // "left" or "right" based on detected sign
    double intersection_initial_yaw_;         // Initial yaw when intersection mode starts
    bool intersection_yaw_recorded_;          // Whether initial yaw is recorded
    double yaw_tolerance_;                    // Tolerance for yaw comparison in degrees
    bool turn_left_permanent_;
    bool turn_right_permanent_;

    // Lane detection parameters
    int hue_white_l, hue_white_h;
    int saturation_white_l, saturation_white_h;
    int value_white_l, value_white_h;
    int hue_yellow_l, hue_yellow_h;
    int saturation_yellow_l, saturation_yellow_h;
    int value_yellow_l, value_yellow_h;
    double roi_height_factor; // ROI height factor

    // Navigation parameters
    double max_linear_speed;       // m/s
    double max_angular_speed;      // rad/s
    double steering_sensitivity;   // steering gain
    double non_linear_factor;      // non-linear factor
    double min_linear_speed;       // Minimum linear speed
    double speed_reduction_factor; // Factor to reduce speed when turning

    // PID controller
    PID_t steering_pid_;
    double pid_kp_;
    double pid_ki_;
    double pid_kd_;

    // Configuration file paths
    std::string speed_config_path_;
    std::string color_config_path_;

    // CrossWalk parameter
    ros::Subscriber crosswalk_pixel_sub_;
    int current_crosswalk_pixels_;
    bool crosswalk_pixel_received_;
    int crosswalk_threshold_;
    bool crosswalk_stopped_;

    // Intersection enhancement variables
    bool use_sign_finding_;             // Toggle for sign-finding sequence
    bool use_x_sign_;                   // Toggle for X sign behavior
    int sign_finding_attempts_;         // Counter for forward-backward attempts
    bool sign_finding_forward_;         // Whether we're moving forward or backward
    std::string default_direction_;     // Default direction if no sign detected
    ros::Time sign_finding_start_time_; // Start time for sign-finding movement
    double sign_finding_timeout_;       // Timeout for each sign-finding movement
    double sign_no_x_wait_time_;        // Time to wait before simulating X sign
    bool in_sign_finding_mode_;         // Whether we're in sign-finding mode

public:
    LaneDetectServer(std::string name) : as_(nh_, name, boost::bind(&LaneDetectServer::executeCB, this, _1), false),
                                         action_name_(name),
                                         it_(nh_),
                                         image_received_(false),
                                         current_mode_("center"),
                                         actual_driving_mode_("center"),
                                         sign_detected_(false),
                                         last_sign_data_(0),
                                         target_sign_(""),
                                         both_lanes_detected_(false),
                                         x_previous_(0.0),
                                         intersection_flag_(0),
                                         previous_both_lanes_(false),
                                         initial_yaw_(0.0),
                                         current_yaw_(0.0),
                                         is_turn_mode_(false),
                                         yaw_initialized_(false),
                                         detect_sign_process_id_(-1),
                                         intersection_initial_turn_done_(false),
                                         intersection_x_sign_seen_(false),
                                         intersection_retry_count_(0),
                                         intersection_turn_direction_(""),
                                         intersection_initial_yaw_(0.0),
                                         intersection_yaw_recorded_(false),
                                         turn_left_permanent_(false),
                                         turn_right_permanent_(false),
                                         current_crosswalk_pixels_(0),
                                         crosswalk_pixel_received_(false),
                                         crosswalk_threshold_(900),
                                         crosswalk_stopped_(false),
                                         yaw_tolerance_(10.0) // 10 degrees tolerance
    {
        recorded_left_lane_x_.resize(5, -1);
        recorded_right_lane_x_.resize(5, -1);
        // Get camera topic from parameter server
        nh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_projected_compensated");

        // Define config file paths (using the src/config directory)
        speed_config_path_ = ros::package::getPath("action_bt") + "/../config/speed_conf.yaml";
        color_config_path_ = ros::package::getPath("action_bt") + "/../config/color_lane.yaml";

        // Load configuration parameters
        loadSpeedParameters();
        loadColorParameters();
        loadCrosswalkParameters();

        // Initialize PID controller
        pid_param(&steering_pid_, pid_kp_, pid_ki_, pid_kd_);
        steering_pid_.setpoint = 0.0; // We want to center the robot

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &LaneDetectServer::imageCallback, this);

        // Subscribe to the traffic sign topic
        traffic_sign_sub_ = nh_.subscribe("/detect/traffic_sign", 10, &LaneDetectServer::trafficSignCallback, this);

        // Subscribe to odometry for yaw tracking
        odom_sub_ = nh_.subscribe("/odom", 10, &LaneDetectServer::odomCallback, this);

        // Subscribe to crosswalk value
        crosswalk_pixel_sub_ = nh_.subscribe("/crosswalk/pixel_count", 10, &LaneDetectServer::crosswalkPixelCallback, this);

        as_.start();
        ROS_INFO("Lane Detection Action Server Started");

        // Display loaded navigation parameters
        ROS_INFO("Navigation parameters from %s:", speed_config_path_.c_str());
        ROS_INFO("  max_linear_speed: %.2f m/s", max_linear_speed);
        ROS_INFO("  max_angular_speed: %.2f rad/s", max_angular_speed);
        ROS_INFO("  steering_sensitivity: %.2f", steering_sensitivity);
        ROS_INFO("  non_linear_factor: %.2f", non_linear_factor);
        ROS_INFO("  min_linear_speed: %.2f m/s", min_linear_speed);
        ROS_INFO("  speed_reduction_factor: %.2f", speed_reduction_factor);
        ROS_INFO("  PID parameters: kp=%.3f, ki=%.3f, kd=%.3f", pid_kp_, pid_ki_, pid_kd_);

        ROS_INFO("Color parameters from %s:", color_config_path_.c_str());
        ROS_INFO("  white_lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 hue_white_l, hue_white_h, saturation_white_l, saturation_white_h, value_white_l, value_white_h);
        ROS_INFO("  yellow_lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 hue_yellow_l, hue_yellow_h, saturation_yellow_l, saturation_yellow_h, value_yellow_l, value_yellow_h);
        ROS_INFO("  ROI height factor: %.2f", roi_height_factor);
    }

    ~LaneDetectServer()
    {
        // Ensure the robot stops when the server shuts down
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);

        // Terminate any active sign detection process
        terminateSignDetection();
    }

    void loadSpeedParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(speed_config_path_);

            // Extract parameters with defaults if not found
            max_linear_speed = config["max_linear_speed"] ? config["max_linear_speed"].as<double>() : 0.08;
            max_angular_speed = config["max_angular_speed"] ? config["max_angular_speed"].as<double>() : 1.5;
            steering_sensitivity = config["steering_sensitivity"] ? config["steering_sensitivity"].as<double>() : 5.0;
            non_linear_factor = config["non_linear_factor"] ? config["non_linear_factor"].as<double>() : 1.5;
            min_linear_speed = config["min_linear_speed"] ? config["min_linear_speed"].as<double>() : 0.05;
            speed_reduction_factor = config["speed_reduction_factor"] ? config["speed_reduction_factor"].as<double>() : 0.7;

            // Load PID parameters
            pid_kp_ = config["pid_kp"] ? config["pid_kp"].as<double>() : 5.0;
            pid_ki_ = config["pid_ki"] ? config["pid_ki"].as<double>() : 0.0;
            pid_kd_ = config["pid_kd"] ? config["pid_kd"].as<double>() : 0.0;
            pid_kp_ *= -1;

            // Load enhancement parameters
            if (config["use_sign_finding"])
                use_sign_finding_ = config["use_sign_finding"].as<bool>();

            if (config["use_x_sign"])
                use_x_sign_ = config["use_x_sign"].as<bool>();

            if (config["default_direction"])
                default_direction_ = config["default_direction"].as<std::string>();

            if (config["sign_finding_timeout"])
                sign_finding_timeout_ = config["sign_finding_timeout"].as<double>();

            if (config["sign_no_x_wait_time"])
                sign_no_x_wait_time_ = config["sign_no_x_wait_time"].as<double>();

            ROS_INFO("Intersection enhancement parameters:");
            ROS_INFO("  use_sign_finding: %s", use_sign_finding_ ? "true" : "false");
            ROS_INFO("  use_x_sign: %s", use_x_sign_ ? "true" : "false");
            ROS_INFO("  default_direction: %s", default_direction_.c_str());
            ROS_INFO("  sign_finding_timeout: %.2f", sign_finding_timeout_);
            ROS_INFO("  sign_no_x_wait_time: %.2f", sign_no_x_wait_time_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading speed configuration YAML file: %s", e.what());
            ROS_WARN("Using default speed parameters");
            // Default values
            max_linear_speed = 0.08;
            max_angular_speed = 1.5;
            steering_sensitivity = 5.0;
            non_linear_factor = 1.5;
            min_linear_speed = 0.05;
            speed_reduction_factor = 0.7;
            pid_kp_ = -5.0;
            pid_ki_ = 0.05;
            pid_kd_ = 0.2;

            use_sign_finding_ = true;
            use_x_sign_ = true;
            sign_finding_attempts_ = 0;
            sign_finding_forward_ = true;
            default_direction_ = "left";
            in_sign_finding_mode_ = false;
            sign_finding_timeout_ = 5.0; // 5 seconds per movement
            sign_no_x_wait_time_ = 2.0;  // 2 seconds wait if use_x_sign is false
        }
    }

    void loadColorParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(color_config_path_);

            // Extract white lane parameters
            YAML::Node white_lane = config["white_lane"];
            if (white_lane)
            {
                hue_white_l = white_lane["hue"]["low"] ? white_lane["hue"]["low"].as<int>() : 0;
                hue_white_h = white_lane["hue"]["high"] ? white_lane["hue"]["high"].as<int>() : 179;
                saturation_white_l = white_lane["saturation"]["low"] ? white_lane["saturation"]["low"].as<int>() : 0;
                saturation_white_h = white_lane["saturation"]["high"] ? white_lane["saturation"]["high"].as<int>() : 70;
                value_white_l = white_lane["value"]["low"] ? white_lane["value"]["low"].as<int>() : 165;
                value_white_h = white_lane["value"]["high"] ? white_lane["value"]["high"].as<int>() : 255;
            }
            else
            {
                ROS_WARN("White lane section not found in color config file. Using defaults.");
                hue_white_l = 0;
                hue_white_h = 179;
                saturation_white_l = 0;
                saturation_white_h = 70;
                value_white_l = 165;
                value_white_h = 255;
            }

            // Extract yellow lane parameters
            YAML::Node yellow_lane = config["yellow_lane"];
            if (yellow_lane)
            {
                hue_yellow_l = yellow_lane["hue"]["low"] ? yellow_lane["hue"]["low"].as<int>() : 10;
                hue_yellow_h = yellow_lane["hue"]["high"] ? yellow_lane["hue"]["high"].as<int>() : 50;
                saturation_yellow_l = yellow_lane["saturation"]["low"] ? yellow_lane["saturation"]["low"].as<int>() : 100;
                saturation_yellow_h = yellow_lane["saturation"]["high"] ? yellow_lane["saturation"]["high"].as<int>() : 255;
                value_yellow_l = yellow_lane["value"]["low"] ? yellow_lane["value"]["low"].as<int>() : 100;
                value_yellow_h = yellow_lane["value"]["high"] ? yellow_lane["value"]["high"].as<int>() : 255;
            }
            else
            {
                ROS_WARN("Yellow lane section not found in color config file. Using defaults.");
                hue_yellow_l = 10;
                hue_yellow_h = 50;
                saturation_yellow_l = 100;
                saturation_yellow_h = 255;
                value_yellow_l = 100;
                value_yellow_h = 255;
            }

            // Extract ROI parameters
            YAML::Node roi = config["roi"];
            if (roi)
            {
                roi_height_factor = roi["height_factor"] ? roi["height_factor"].as<double>() : 0.5;
            }
            else
            {
                ROS_WARN("ROI section not found in color config file. Using default.");
                roi_height_factor = 0.5;
            }
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading color configuration YAML file: %s", e.what());
            ROS_WARN("Using default color parameters");
            // Default white lane values
            hue_white_l = 0;
            hue_white_h = 179;
            saturation_white_l = 0;
            saturation_white_h = 70;
            value_white_l = 165;
            value_white_h = 255;

            // Default yellow lane values
            hue_yellow_l = 10;
            hue_yellow_h = 50;
            saturation_yellow_l = 100;
            saturation_yellow_h = 255;
            value_yellow_l = 100;
            value_yellow_h = 255;

            // Default ROI value
            roi_height_factor = 0.5;
        }
    }

    void loadCrosswalkParameters()
    {
        try
        {
            std::string cross_config_path = ros::package::getPath("action_bt") + "/../config/cross_param.yaml";
            YAML::Node config = YAML::LoadFile(cross_config_path);

            // Load detection threshold for cross_level mode
            crosswalk_threshold_ = config["cross_walking"]["detection_threshold"] ? config["cross_walking"]["detection_threshold"].as<int>() : 900;

            ROS_INFO("Loaded crosswalk threshold from cross_param.yaml: %d", crosswalk_threshold_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading crosswalk configuration: %s", e.what());
            ROS_WARN("Using default crosswalk threshold: %d", crosswalk_threshold_);
            crosswalk_threshold_ = 900; // Default fallback
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!image_received_)
        {
            ROS_INFO("Received first camera image!");
        }

        try
        {
            img_bgr_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            image_received_ = true;
            cv::cvtColor(img_bgr_, img_hsv_, cv::COLOR_BGR2HSV);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    // Callback for odometry data to get yaw information
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
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

        // Store current yaw
        current_yaw_ = yaw;

        // Initialize initial_yaw_ if needed (first time in turn mode)
        if (is_turn_mode_ && !yaw_initialized_)
        {
            initial_yaw_ = yaw;
            yaw_initialized_ = true;
            ROS_INFO("Initial yaw set to: %.2f degrees", initial_yaw_ * 180.0 / M_PI);
        }

        // Record initial yaw for intersection mode
        if (current_mode_ == "intersection" && !intersection_yaw_recorded_ && image_received_)
        {
            intersection_initial_yaw_ = yaw;
            intersection_yaw_recorded_ = true;
            ROS_INFO("Intersection initial yaw recorded: %.2f degrees", intersection_initial_yaw_ * 180.0 / M_PI);
        }

        // Debug yaw information
        if (is_turn_mode_ && yaw_initialized_)
        {
            // Calculate turn angle (handle wrap-around)
            double yaw_diff = current_yaw_ - initial_yaw_;

            // Normalize to [-π, π]
            while (yaw_diff > M_PI)
                yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI)
                yaw_diff += 2 * M_PI;

            // Convert to degrees for easier debugging
            double turn_degrees = yaw_diff * 180.0 / M_PI;

            ROS_INFO_THROTTLE(1.0, "Current yaw: %.2f, Initial: %.2f, Diff: %.2f degrees",
                              current_yaw_ * 180.0 / M_PI, initial_yaw_ * 180.0 / M_PI, turn_degrees);
        }
    }

    void crosswalkPixelCallback(const std_msgs::Int32::ConstPtr &msg)
    {
        current_crosswalk_pixels_ = msg->data;
        crosswalk_pixel_received_ = true;

        // Log crosswalk detection info when in cross_level mode
        if (current_mode_ == "cross_level")
        {
            ROS_INFO_THROTTLE(0.5, "Cross level mode: %d/%d pixels detected",
                              current_crosswalk_pixels_, crosswalk_threshold_);
        }
    }

    // Callback for the traffic sign topic
    void trafficSignCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        // Store the sign data
        last_sign_data_ = msg->data;

        // Log sign detection
        ROS_INFO("SIGN CALLBACK TRIGGERED: data=%d, sign_detected=%s",
                 last_sign_data_, sign_detected_ ? "TRUE" : "FALSE");

        // Check if this is the sign we're waiting for
        if (current_mode_ == "intersection" && !intersection_initial_turn_done_)
        {
            // For new intersection mode, detect left/right signs
            if (last_sign_data_ == 2) // Left sign
            {
                ROS_INFO("Left sign detected for intersection!");
                sign_detected_ = true;
                intersection_turn_direction_ = "left";
                actual_driving_mode_ = "left";
            }
            else if (last_sign_data_ == 3) // Right sign
            {
                ROS_INFO("Right sign detected for intersection!");
                sign_detected_ = true;
                intersection_turn_direction_ = "right";
                actual_driving_mode_ = "right";
            }
        }
        else if (current_mode_ == "intersection" && intersection_initial_turn_done_ && last_sign_data_ == 4)
        {
            // X sign detection (data = 4)
            ROS_INFO("X sign detected! Will retry turn direction: %s", intersection_turn_direction_.c_str());
            intersection_x_sign_seen_ = true;
            intersection_retry_count_ = 0;                       // Reset retry count
            actual_driving_mode_ = intersection_turn_direction_; // Set to retry the same direction
        }
        else if (target_sign_ == "construction" && last_sign_data_ == 1)
        {
            // Construction sign
            ROS_INFO("Construction sign detected!");
            sign_detected_ = true;
        }
        else if (target_sign_ == "tunnel" && last_sign_data_ == 1)
        {
            // Tunnel sign
            ROS_INFO("Tunnel sign detected!");
            sign_detected_ = true;
        }
        else if (target_sign_ == "cross" && last_sign_data_ == 1)
        {
            // Cross (stop) sign
            ROS_INFO("Cross/Stop sign detected!");
            sign_detected_ = true;
        }
        // ADD NEW CONDITION HERE for parking sign detection
        else if (target_sign_ == "parking" && last_sign_data_ == 1)
        {
            // Parking sign
            ROS_INFO("Parking sign detected!");
            sign_detected_ = true;
        }
    }

    // Launch sign detection node based on target sign or mode
    bool launchSignDetection()
    {
        std::string mission;

        // Determine the mission type based on target sign or mode
        if (current_mode_ == "intersection" || target_sign_ == "intersection")
        {
            mission = "intersection";
        }
        else if (target_sign_ == "construction")
        {
            mission = "construction";
        }
        else if (target_sign_ == "tunnel")
        {
            mission = "tunnel";
        }
        else if (target_sign_ == "cross")
        {
            mission = "level_crossing";
        }
        else if (target_sign_ == "parking")
        {
            mission = "parking"; // Add this line for parking mission
        }
        else
        {
            // No need to launch sign detection for other sign types
            return true;
        }

        ROS_INFO("Launching sign detection for mission: %s", mission.c_str());

        // Create command to launch the sign detection node
        std::string cmd = "roslaunch turtlebot3_autorace_detect detect_sign.launch mission:=" +
                          mission + " & echo 'Using package path:' && echo $ROS_PACKAGE_PATH > /tmp/pkg_path.txt & echo $! > /tmp/sign_detect_pid";

        // Execute command
        int ret = system(cmd.c_str());

        // Check result
        if (ret != 0)
        {
            ROS_ERROR("Failed to launch sign detection: %s (return code: %d)", cmd.c_str(), ret);
            return false;
        }

        // Read the PID from the temp file
        std::ifstream pid_file("/tmp/sign_detect_pid");
        pid_file >> detect_sign_process_id_;
        pid_file.close();

        ROS_INFO("Sign detection launched with PID: %d", detect_sign_process_id_);
        return true;
    }

    // Terminate sign detection process
    void terminateSignDetection()
    {
        if (detect_sign_process_id_ > 0)
        {
            ROS_INFO("Terminating sign detection process (PID: %d)", detect_sign_process_id_);
            kill(detect_sign_process_id_, SIGTERM);
            detect_sign_process_id_ = -1;
        }
    }

    bool performSignFinding()
    {
        // If sign detected, stop sign finding
        if (sign_detected_ && !intersection_turn_direction_.empty())
        {
            in_sign_finding_mode_ = false;
            stopRobot();
            ROS_INFO("Sign detected during finding sequence: %s",
                     intersection_turn_direction_.c_str());
            return true;
        }

        // Check if current movement has timed out
        double movement_elapsed = (ros::Time::now() - sign_finding_start_time_).toSec();

        if (movement_elapsed > sign_finding_timeout_)
        {
            // Movement timed out, switch direction or increment attempts
            if (sign_finding_forward_)
            {
                // Switch to backward
                sign_finding_forward_ = false;
                sign_finding_start_time_ = ros::Time::now();
                ROS_INFO("Sign finding: Switching to backward movement (attempt %d)",
                         sign_finding_attempts_);
            }
            else
            {
                // Completed one forward-backward cycle
                sign_finding_attempts_++;

                if (sign_finding_attempts_ >= 3)
                {
                    // Exceeded maximum attempts, use default direction
                    intersection_turn_direction_ = default_direction_;
                    actual_driving_mode_ = default_direction_;
                    sign_detected_ = true;
                    in_sign_finding_mode_ = false;
                    stopRobot();
                    ROS_INFO("Sign finding failed after 3 attempts, using default direction: %s",
                             default_direction_.c_str());
                    return true;
                }
                else
                {
                    // Start next cycle
                    sign_finding_forward_ = true;
                    sign_finding_start_time_ = ros::Time::now();
                    ROS_INFO("Sign finding: Starting attempt %d", sign_finding_attempts_ + 1);
                }
            }
        }

        // Set movement based on current direction
        geometry_msgs::Twist cmd;
        cmd.linear.x = sign_finding_forward_ ? 0.05 : -0.05; // Slow movement
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);

        ROS_INFO_THROTTLE(1.0, "Sign finding: %s movement, elapsed: %.1f/%.1f, attempt %d/3",
                          sign_finding_forward_ ? "Forward" : "Backward",
                          movement_elapsed, sign_finding_timeout_,
                          sign_finding_attempts_);

        return false;
    }

    // Helper method to stop the robot
    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    void detectLane()
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet on topic: %s", camera_topic_.c_str());
            return;
        }

        // 1. Create masks for white and yellow lanes
        cv::Mat img_white_mask, img_yellow_mask;
        cv::Scalar lower_white = cv::Scalar(hue_white_l, saturation_white_l, value_white_l);
        cv::Scalar upper_white = cv::Scalar(hue_white_h, saturation_white_h, value_white_h);
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l, saturation_yellow_l, value_yellow_l);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h, saturation_yellow_h, value_yellow_h);

        cv::inRange(img_hsv_, lower_white, upper_white, img_white_mask);
        cv::inRange(img_hsv_, lower_yellow, upper_yellow, img_yellow_mask);

        // 2. Combine masks
        img_masked_ = cv::Mat::zeros(img_bgr_.rows, img_bgr_.cols, CV_8UC1);
        cv::bitwise_or(img_white_mask, img_yellow_mask, img_masked_);

        // 3. Apply ROI mask - focus only on the bottom part of the image
        int height = img_masked_.rows;
        int width = img_masked_.cols;

        // Debug information
        ROS_INFO_ONCE("Image dimensions: %d x %d", width, height);

        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);

        // Use the roi_height_factor from config
        int roi_top = height * (1.0 - roi_height_factor);

        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, height));
        roi_points.push_back(cv::Point(width, height));
        roi_points.push_back(cv::Point(width, roi_top));
        roi_points.push_back(cv::Point(0, roi_top));

        cv::fillConvexPoly(roi_mask, roi_points, cv::Scalar(255, 0, 0));
        cv::bitwise_and(img_masked_, roi_mask, img_masked_);

        // 4. Apply simple processing
        cv::GaussianBlur(img_masked_, img_masked_, cv::Size(5, 5), 0);

        // 5. Create a visualization for debugging
        cv::Mat debug_img;
        cv::cvtColor(img_masked_, debug_img, cv::COLOR_GRAY2BGR);

        // Display current mode and last detected sign
        cv::putText(debug_img, "Mode: " + current_mode_, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

        // Special display for intersection mode
        if (current_mode_ == "intersection")
        {
            std::string state_text = "State: ";
            if (!intersection_initial_turn_done_)
            {
                state_text += "Wait for Turn Sign";
            }
            else if (!intersection_x_sign_seen_)
            {
                state_text += "Looking for X Sign";
            }
            else
            {
                state_text += "Final Turn " + intersection_turn_direction_;
            }

            cv::putText(debug_img, state_text, cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            if (intersection_yaw_recorded_)
            {
                double yaw_diff = calculateYawDifference(current_yaw_, intersection_initial_yaw_);
                std::string yaw_text = "Yaw Diff: " + std::to_string(int(yaw_diff)) + " deg";
                cv::putText(debug_img, yaw_text, cv::Point(10, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            }
        }

        if (last_sign_data_ > 0)
        {
            std::string sign_text = "Last Sign: " + std::to_string(last_sign_data_);
            cv::putText(debug_img, sign_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        // Display yaw information for turn modes
        if (is_turn_mode_ && yaw_initialized_)
        {
            double yaw_diff = current_yaw_ - initial_yaw_;

            // Normalize to [-π, π]
            while (yaw_diff > M_PI)
                yaw_diff -= 2 * M_PI;
            while (yaw_diff < -M_PI)
                yaw_diff += 2 * M_PI;

            double turn_degrees = yaw_diff * 180.0 / M_PI;

            std::string yaw_text = "Yaw: " + std::to_string(int(turn_degrees)) + " deg";
            cv::putText(debug_img, yaw_text, cv::Point(width / 2 - 80, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        // 6. Call the mode-specific control method
        moveRobot(img_masked_, debug_img);

        // 7. Display the result
        cv::imshow("Lane Detection", debug_img);
        cv::waitKey(1);
    }

    // Helper function to calculate yaw difference in degrees
    double calculateYawDifference(double current, double initial)
    {
        double diff = current - initial;

        // Normalize to [-π, π]
        while (diff > M_PI)
            diff -= 2 * M_PI;
        while (diff < -M_PI)
            diff += 2 * M_PI;

        // Convert to degrees
        return diff * 180.0 / M_PI;
    }

    // Non-linear gain function
    double applyNonLinearGain(double value, double factor)
    {
        // Apply exponential function for more aggressive response on large offsets
        return copysign(pow(fabs(value), factor), value);
    }

    void moveRobot(const cv::Mat &lane_img, cv::Mat &debug_img)
    {
        int height = lane_img.rows;
        int width = lane_img.cols;
        int center_x = width / 2;

        // Get actual driving mode to use for calculations
        std::string effective_mode = current_mode_;
        std::string lane_follow_mode = "center"; // Default lane following mode

        if (current_mode_ == "cross_level" && crosswalk_stopped_)
        {
            // Robot should be stopped at crosswalk
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd);

            ROS_INFO_THROTTLE(1.0, "Robot stopped at crosswalk: %d/%d pixels",
                              current_crosswalk_pixels_, crosswalk_threshold_);
            return; // Skip normal movement logic
        }

        // For intersection mode, determine the driving behavior
        if (current_mode_ == "intersection")
        {
            if (sign_detected_ && !intersection_initial_turn_done_)
            {
                // After detecting left/right sign, start turning
                effective_mode = actual_driving_mode_; // "left" or "right"
            }
            else if (intersection_x_sign_seen_ && (intersection_retry_count_ <= 1))
            {
                // After seeing X sign, retry the turn
                effective_mode = actual_driving_mode_; // Same direction as before
            }
            else if (turn_left_permanent_)
            {
                effective_mode = "left";
            }
            else if (turn_right_permanent_)
            {
                effective_mode = "right";
            }
            else
            {
                // Otherwise follow center lane
                effective_mode = "center";
            }
        }
        if (current_mode_ == "cross_level")
        {
            effective_mode == "cross_level";
        }

        // For turn modes, determine which lane to follow based on sign parameter
        if (current_mode_ == "just-turn-left" || current_mode_ == "just-turn-right")
        {
            // Use the sign parameter to determine which lane to follow
            if (!target_sign_.empty())
            {
                lane_follow_mode = target_sign_; // Use "left" or "right" from sign parameter
                ROS_INFO_THROTTLE(1.0, "Turn mode %s following %s lane",
                                  current_mode_.c_str(), lane_follow_mode.c_str());
            }
            else
            {
                // Default lane to follow based on turn direction if no sign specified
                lane_follow_mode = (current_mode_ == "just-turn-left") ? "left" : "right";
            }
        }

        // Create multiple regions of interest (ROIs) for better analysis
        int numRegions = 5;
        std::vector<int> region_centers(numRegions, -1);
        std::vector<bool> region_has_left(numRegions, false);
        std::vector<bool> region_has_right(numRegions, false);

        int step = height / (numRegions + 1);

        // Analyze each region to get lane positions
        for (int r = 0; r < numRegions; r++)
        {
            int y = height - (r + 1) * step; // Start from bottom to top
            int left_sum = 0, right_sum = 0;
            int left_weighted_sum = 0, right_weighted_sum = 0;

            // Scan wider area in region
            int scan_width = 20; // Scan width on each side of y

            // Calculate lane points in this region
            for (int offset = -scan_width; offset <= scan_width; offset++)
            {
                int scan_y = y + offset;
                if (scan_y < 0 || scan_y >= height)
                    continue;

                for (int x = 0; x < width; x++)
                {
                    if (lane_img.at<uchar>(scan_y, x) > 0)
                    { // Lane pixel (white)
                        if (x < center_x)
                        {
                            left_sum++;
                            left_weighted_sum += x;
                        }
                        else
                        {
                            right_sum++;
                            right_weighted_sum += x;
                        }
                    }
                }
            }

            int left_lane_x = -1;
            int right_lane_x = -1;

            // Check if left lane is detected
            if (left_sum > 20)
            { // Threshold to confirm lane detection
                left_lane_x = left_weighted_sum / left_sum;
                region_has_left[r] = true;
            }

            // Check if right lane is detected
            if (right_sum > 20)
            { // Threshold to confirm lane detection
                right_lane_x = right_weighted_sum / right_sum;
                region_has_right[r] = true;
            }

            // // Simpan posisi garis opposite saat mode center dan dua garis terdeteksi
            // if (effective_mode == "center" && region_has_left[r] && region_has_right[r])
            // {
            //     recorded_left_lane_x_[r] = left_lane_x - right_lane_x;
            //     recorded_right_lane_x_[r] = right_lane_x - left_lane_x;
            // }
            // ROS_INFO("Left %d = %d ", r, left_lane_x);
            // ROS_INFO("Right %d = %d ", r, right_lane_x);

            // // Kalkulasi center
            // if (region_has_left[r] && region_has_right[r])
            // {
            //     region_centers[r] = (left_lane_x + right_lane_x) / 2;
            // }
            // else if (region_has_right[r])
            // {
            //     int fake_left = right_lane_x + recorded_left_lane_x_[r];
            //     ROS_INFO("Oppose %d = %d ", r, fake_left);
            //     if (fake_left != -1)
            //     {
            //         region_centers[r] = (right_lane_x + fake_left) / 2;
            //         cv::circle(debug_img, cv::Point(fake_left, y), 3, cv::Scalar(255, 0, 255), -1); // fake left
            //     }
            //     else
            //     {
            //         region_centers[r] = right_lane_x; // fallback
            //     }
            // }
            // else if (region_has_left[r])
            // {
            //     int fake_right = left_lane_x + recorded_right_lane_x_[r];
            //     ROS_INFO("Oppose %d = %d ", r, fake_right);
            //     if (fake_right != -1)
            //     {
            //         region_centers[r] = (left_lane_x + fake_right) / 2;
            //         cv::circle(debug_img, cv::Point(fake_right, y), 3, cv::Scalar(255, 255, 0), -1); // fake right
            //     }
            //     else
            //     {
            //         region_centers[r] = left_lane_x; // fallback
            //     }
            // }

            // Calculate region center based on detected lanes and current mode
            if (effective_mode == "center")
            {
                if (region_has_left[r] && region_has_right[r])
                {
                    recorded_left_lane_x_[r] = left_lane_x - right_lane_x;
                    recorded_right_lane_x_[r] = right_lane_x - left_lane_x;
                    region_centers[r] = (left_lane_x + right_lane_x) / 2;

                    // Store the lane width for future calculations
                    if (r == 0)
                    { // Only store from bottom region
                        x_previous_ = (right_lane_x - left_lane_x) / 2;
                    }
                }

                if (!region_has_left[r])
                {
                    region_centers[r] = 160;
                }
                else if (!region_has_right[r])
                {
                    // If right lane detected and we have previous offset,
                    // estimate where left lane should be
                    region_centers[r] = 300;
                }

                ROS_INFO("Left %d = %d ", r, left_lane_x);
                ROS_INFO("Right %d = %d ", r, right_lane_x);
            }

            else if (effective_mode == "cross_level")
            {
                // Cross level mode should follow center lane like center mode
                if (region_has_left[r] && region_has_right[r])
                {
                    recorded_left_lane_x_[r] = left_lane_x - right_lane_x;
                    recorded_right_lane_x_[r] = right_lane_x - left_lane_x;
                    region_centers[r] = (left_lane_x + right_lane_x) / 2;

                    // Store the lane width for future calculations
                    if (r == 0)
                    { // Only store from bottom region
                        x_previous_ = (right_lane_x - left_lane_x) / 2;
                    }
                }

                if (!region_has_left[r])
                {
                    region_centers[r] = 160;
                }
                else if (!region_has_right[r])
                {
                    // If right lane detected and we have previous offset,
                    // estimate where left lane should be
                    region_centers[r] = 300;
                }

                ROS_INFO("Cross Level - Left %d = %d ", r, left_lane_x);
                ROS_INFO("Cross Level - Right %d = %d ", r, right_lane_x);
            }

            else if (effective_mode == "left" ||
                     (is_turn_mode_ && lane_follow_mode == "left"))
            {
                if (current_mode_ != "intersection")
                {
                    if (region_has_left[r] && region_has_right[r])
                    {
                        region_centers[r] = (left_lane_x + right_lane_x) / 2;
                    }

                    if (region_has_left[r])
                    {
                        int fake_right = left_lane_x + recorded_right_lane_x_[r];
                        ROS_INFO("Oppose %d = %d ", r, fake_right);
                        if (fake_right != -1)
                        {
                            region_centers[r] = (left_lane_x + fake_right) / 2;
                            cv::circle(debug_img, cv::Point(fake_right, y), 3, cv::Scalar(255, 255, 0), -1); // fake right
                        }
                        else
                        {
                            region_centers[r] = left_lane_x; // fallback
                        }
                    }
                    else if (region_has_right[r] && recorded_right_lane_x_[r] > 0)
                    {
                        // If right lane detected and we have previous offset,
                        // estimate where left lane should be
                        region_centers[r] = right_lane_x - recorded_right_lane_x_[r];
                    }
                }
                else
                {
                    // Left mode follows left lane with offset
                    if (region_has_left[r])
                    {
                        region_centers[r] = left_lane_x + x_previous_;
                    }
                    else if (region_has_right[r] && recorded_right_lane_x_[r] > 0)
                    {
                        region_centers[r] = 160;
                    }
                    else
                    {
                        // If right lane detected and we have previous offset,
                        // estimate where left lane should be

                        region_centers[r] = 160;
                    }
                }
            }
            else if (effective_mode == "right" ||
                     (is_turn_mode_ && lane_follow_mode == "right"))
            {
                if (current_mode_ != "intersection")
                {
                    // Right mode follows right lane with offset
                    // if (region_has_left[r] && region_has_right[r])
                    // {
                    //     region_centers[r] = (left_lane_x + right_lane_x) / 2;
                    // }
                    if (region_has_right[r])
                    {
                        int fake_left = right_lane_x + recorded_left_lane_x_[r];
                        ROS_INFO("Oppose %d = %d ", r, fake_left);
                        if (fake_left != -1)
                        {
                            region_centers[r] = (right_lane_x + fake_left) / 2;
                            cv::circle(debug_img, cv::Point(fake_left, y), 3, cv::Scalar(255, 0, 255), -1); // fake left
                        }
                        else
                        {
                            region_centers[r] = right_lane_x; // fallback
                        }
                    }
                    else if (region_has_left[r] && recorded_left_lane_x_[r] < 0)
                    {
                        // If left lane detected and we have previous offset,
                        // estimate where right lane should be
                        region_centers[r] = left_lane_x - recorded_left_lane_x_[r];
                    }
                }
                else
                {
                    // Left mode follows left lane with offset
                    if (region_has_right[r])
                    {
                        region_centers[r] = right_lane_x - x_previous_ + 60;
                    }
                    else if (region_has_left[r] && recorded_left_lane_x_[r] < 0)
                    {
                        // If right lane detected and we have previous offset,
                        // estimate where left lane should be
                        region_centers[r] = 300;
                    }
                    else
                    {
                        region_centers[r] = 300;
                    }
                }
            }

            // Debug visualization
            if (region_centers[r] != -1)
            {
                // Draw region center
                cv::circle(debug_img, cv::Point(region_centers[r], y), 5, cv::Scalar(0, 0, 255), -1);

                // Draw line from center to target
                cv::line(debug_img, cv::Point(center_x, y), cv::Point(region_centers[r], y),
                         cv::Scalar(0, 255, 0), 2);
            }

            // Draw detected lane positions
            if (region_has_left[r])
            {
                cv::circle(debug_img, cv::Point(left_lane_x, y), 3, cv::Scalar(255, 0, 0), -1);
            }
            if (region_has_right[r])
            {
                cv::circle(debug_img, cv::Point(right_lane_x, y), 3, cv::Scalar(0, 255, 0), -1);
            }
        }

        // Update both lanes detection flag (for special success condition)
        bool current_both_lanes = false;
        int both_count = 0;
        for (int r = 0; r < numRegions; r++)
        {
            if (region_has_left[r] && region_has_right[r])
            {
                both_count++;
            }
        }

        // If at least 2 regions detect both lanes, consider it as both lanes detected
        if (both_count >= 2)
        {
            current_both_lanes = true;
            cv::putText(debug_img, "BOTH LANES DETECTED", cv::Point(width / 2 - 100, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        // Check for transition in lane detection status for intersection detection
        if (current_both_lanes != previous_both_lanes_)
        {
            // We have a transition (either from both->single or single->both)
            ros::Time current_time = ros::Time::now();

            // Only count transitions with sufficient time gap (0.5 sec) to avoid flickering
            if ((current_time - last_transition_time_).toSec() > 0.5)
            {
                // If we're transitioning to both lanes being detected, increment the counter
                if (current_both_lanes)
                {
                    intersection_flag_++;
                    ROS_INFO("Intersection flag incremented to: %d", intersection_flag_);
                    // if (current_mode_ == "intersection" && intersection_x_sign_seen_ && !intersection_retry_count_ && intersection_initial_turn_done_)
                    // {
                    //     intersection_retry_count_ = 1;
                    // }
                }

                last_transition_time_ = current_time;
            }
        }

        // Display intersection flag count
        std::string flag_text = "Intersection Flag: " + std::to_string(intersection_flag_);
        cv::putText(debug_img, flag_text, cv::Point(10, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

        // Update the previous state and current state
        previous_both_lanes_ = current_both_lanes;
        both_lanes_detected_ = current_both_lanes;

        // Determine turning strategy based on region analysis
        double angular_z = 0.0;
        int valid_regions = 0;
        double total_offset = 0.0;

        // Higher weights for regions closer to robot
        double region_weights[5] = {10.0, 5.0, 3.0, 2.0, 1.0}; // Much higher weights for bottom region

        for (int r = 0; r < numRegions; r++)
        {
            if (region_centers[r] != -1)
            {
                // Offset from image center
                double offset = (region_centers[r] - center_x) / (double)(width / 2); // Normalize to [-1, 1]

                // Apply non-linear factor for increased response to large offsets
                offset = applyNonLinearGain(offset, non_linear_factor);

                // Add to total with weight
                total_offset += offset * region_weights[r];
                valid_regions += region_weights[r];
            }
        }

        if (valid_regions > 0)
        {
            // Calculate weighted average offset
            double avg_offset = total_offset / valid_regions;

            // Apply PID controller
            pid_input(&steering_pid_, avg_offset);
            pid_calculate(&steering_pid_);
            angular_z = -1 * pid_output(&steering_pid_) * max_angular_speed;

            // Limit maximum steering
            if (angular_z > max_angular_speed)
                angular_z = max_angular_speed;
            if (angular_z < -max_angular_speed)
                angular_z = -max_angular_speed;

            ROS_INFO("Mode: %s, Effective Mode: %s, Offset: %.2f, Angular: %.2f",
                     current_mode_.c_str(), effective_mode.c_str(), avg_offset, angular_z);
        }
        else
        {
            ROS_WARN_THROTTLE(1, "No lane detected in any region!");
            angular_z = 0.0;
        }

        // For turning modes, we might want to adjust the linear/angular speed
        // to ensure a tighter turn radius
        // if (is_turn_mode_)
        // {
        //     // Increase angular speed for turning modes to make turns faster
        //     if (current_mode_ == "just-turn-left")
        //     {
        //         // Make sure we're turning left (negative angular_z is right turn in ROS)
        //         if (angular_z > 0)
        //         {
        //             angular_z = std::max(angular_z, max_angular_speed * 0.5); // At least 50% of max turn
        //         }
        //     }
        //     else if (current_mode_ == "just-turn-right")
        //     {
        //         // Make sure we're turning right (positive angular_z is left turn in ROS)
        //         if (angular_z < 0)
        //         {
        //             angular_z = std::min(angular_z, -max_angular_speed * 0.5); // At least 50% of max turn
        //         }
        //     }
        // }

        // Adaptive calculation for linear speed based on steering
        // Reduce speed when turning sharply
        double linear_x = max_linear_speed * (1.0 - 0.7 * fabs(angular_z / max_angular_speed));

        // Ensure minimum speed is maintained
        linear_x = std::max(linear_x, min_linear_speed);

        // For tight turns, reduce speed even more
        if (is_turn_mode_)
        {
            linear_x *= speed_reduction_factor; // Reduce speed for turn modes using factor from config

            // Display lane following mode in debug
            std::string follow_text = "Following: " + lane_follow_mode + " lane";
            cv::putText(debug_img, follow_text, cv::Point(10, 180),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
        }

        // Display PID info on debug image
        std::string pid_text = "PID: e=" + std::to_string(steering_pid_.first_error) +
                               " out=" + std::to_string(steering_pid_.output);
        cv::putText(debug_img, pid_text, cv::Point(10, 210),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

        // Create and send movement command
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;

        // Debug info
        ROS_INFO_THROTTLE(0.5, "CMD: linear.x=%.2f, angular.z=%.2f", cmd.linear.x, cmd.angular.z);

        // Publish command
        cmd_vel_pub_.publish(cmd);
    }

    void executeCB(const msg_file::LaneDetectGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Get parameters from goal
        float duration = goal->duration;
        current_mode_ = goal->mode;
        target_sign_ = goal->sign;
        float speed_override = goal->speed;

        // Override linear speed if provided
        if (speed_override > 0.0)
        {
            max_linear_speed = speed_override;
            ROS_INFO("Linear speed overridden to: %.2f m/s", max_linear_speed);
        }

        // Convert old intersection-dynamic to new intersection mode
        if (current_mode_ == "intersection-dynamic")
        {
            current_mode_ = "intersection";
        }

        // Set default mode if not specified
        if (current_mode_.empty())
        {
            current_mode_ = "center";
        }

        // Set actual driving mode
        actual_driving_mode_ = current_mode_;

        // Reset intersection-specific flags
        intersection_initial_turn_done_ = false;
        intersection_x_sign_seen_ = false;
        intersection_retry_count_ = 0;
        intersection_turn_direction_ = "";
        intersection_yaw_recorded_ = false;

        // Reset intersection enhancement variables
        sign_finding_attempts_ = 0;
        sign_finding_forward_ = true;
        in_sign_finding_mode_ = false;
        ros::Time no_x_wait_start; // For tracking X sign simulation time

        // Check if this is a turning mode
        is_turn_mode_ = (current_mode_ == "just-turn-left" || current_mode_ == "just-turn-right");

        // Reset yaw tracking for turn modes
        if (is_turn_mode_)
        {
            yaw_initialized_ = false;
            ROS_INFO("Turn mode activated: %s", current_mode_.c_str());
        }

        // Initialize x_previous_ with a reasonable value if it's zero
        if (x_previous_ <= 0)
        {
            x_previous_ = 30; // Default reasonable value, adjust based on your robot and lanes
            ROS_INFO("Initializing x_previous_ to default value: %.1f", x_previous_);
        }

        // Reset detection flags
        sign_detected_ = false;
        intersection_flag_ = 0;
        previous_both_lanes_ = false;
        last_transition_time_ = ros::Time::now();
        last_sign_data_ = 0;

        // Reset PID controller
        steering_pid_.sum_error = 0.0;
        steering_pid_.last_error = 0.0;

        ROS_INFO("LaneDetect Server: Mode=%s, Duration=%.2f, Sign=%s",
                 current_mode_.c_str(), duration,
                 target_sign_.empty() ? "none" : target_sign_.c_str());

        ROS_INFO("Linear speed: %.2f m/s (from %s)",
                 max_linear_speed,
                 speed_override > 0.0 ? "goal override" : "YAML config");

        // Launch sign detection if needed
        if (current_mode_ == "intersection" ||
            (target_sign_ == "construction" || target_sign_ == "tunnel" || target_sign_ == "cross"))
        {
            if (!launchSignDetection())
            {
                ROS_ERROR("Failed to launch sign detection");
                as_.setAborted();
                return;
            }
        }

        if (current_mode_ == "cross_level")
        {
        }

        // Main execution loop
        while (ros::ok())
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                terminateSignDetection();
                break;
            }

            // Check success conditions
            bool time_condition = false;
            bool sign_condition = false;
            bool lane_condition = false;
            bool turn_condition = false;
            bool intersection_condition = false;

            // Duration-based success (if duration > 0)
            time_condition = (duration > 0) &&
                             ((ros::Time::now() - start_time_) > ros::Duration(duration));

            // Sign detection success
            sign_condition = !target_sign_.empty() && sign_detected_;

            // Lane detection conditions based on mode
            if (current_mode_ == "left" || current_mode_ == "right")
            {
                // Left/right modes require 1 time detection
                lane_condition = intersection_flag_ >= 1;
                if (lane_condition)
                {
                    ROS_INFO("Left/Right mode: Detected both lanes (flag=%d)", intersection_flag_);
                }
            }

            bool cross_level_condition = false;
            if (current_mode_ == "cross_level")
            {
                if (crosswalk_pixel_received_)
                {
                    if (current_crosswalk_pixels_ >= crosswalk_threshold_ && !crosswalk_stopped_)
                    {
                        // Crosswalk detected - stop the robot
                        crosswalk_stopped_ = true;
                        ROS_INFO("Crosswalk detected! Stopping robot: %d >= %d pixels",
                                 current_crosswalk_pixels_, crosswalk_threshold_);
                    }
                    else if (current_crosswalk_pixels_ < crosswalk_threshold_ && crosswalk_stopped_)
                    {
                        // Crosswalk cleared after being stopped - success!
                        cross_level_condition = true;
                        ROS_INFO("Crosswalk cleared! Condition met: %d < %d pixels",
                                 current_crosswalk_pixels_, crosswalk_threshold_);
                    }
                }
            }

            // Yaw-based turn success condition
            if (is_turn_mode_ && yaw_initialized_)
            {
                // Calculate turn angle (handle wrap-around)
                double yaw_diff = current_yaw_ - initial_yaw_;

                // Normalize to [-π, π]
                while (yaw_diff > M_PI)
                    yaw_diff -= 2 * M_PI;
                while (yaw_diff < -M_PI)
                    yaw_diff += 2 * M_PI;

                // Convert to degrees
                double turn_degrees = fabs(yaw_diff) * 180.0 / M_PI;

                // Check if we've turned 88-92 degrees
                if (turn_degrees >= 88.0 && turn_degrees <= 92.0)
                {
                    // Also check if turn direction matches mode
                    if ((current_mode_ == "just-turn-right" && yaw_diff < 0) ||
                        (current_mode_ == "just-turn-left" && yaw_diff > 0))
                    {
                        turn_condition = true;
                        ROS_INFO("Turn completed: %.2f degrees in %s mode",
                                 turn_degrees, current_mode_.c_str());
                    }
                }
            }

            // Intersection mode success condition
            if (current_mode_ == "intersection")
            {
                // Check the state of intersection mode
                if (!intersection_initial_turn_done_ && sign_detected_ && !intersection_turn_direction_.empty())
                {
                    // Just detected initial turn sign, start turning
                    ROS_INFO("Starting initial turn: %s", intersection_turn_direction_.c_str());
                }
                else if (!intersection_initial_turn_done_ && intersection_flag_ == 0 && sign_detected_)
                {
                    if (last_sign_data_ == 2)
                    {
                        actual_driving_mode_ = "left"; // Switch to center following
                        turn_left_permanent_ = true;
                        ROS_INFO("Set Turn_Left_Permanent");
                    }
                    else if (last_sign_data_ == 3)
                    {
                        actual_driving_mode_ = "right"; // Switch to center following
                        turn_right_permanent_ = true;
                        ROS_INFO("Set Turn_Right_Permanent");
                    }
                }
                else if (!intersection_initial_turn_done_ && intersection_flag_ >= 1 && sign_detected_)
                {
                    // Complete initial turn after seeing flag once
                    intersection_initial_turn_done_ = true;
                    intersection_flag_ = 0; // Reset flag counter
                    if (last_sign_data_ == 2)
                    {
                        actual_driving_mode_ = "left"; // Switch to center following
                        turn_left_permanent_ = true;
                        ROS_INFO("Set Turn_Left_Permanent");
                    }
                    else if (last_sign_data_ == 3)
                    {
                        actual_driving_mode_ = "right"; // Switch to center following
                        turn_right_permanent_ = true;
                        ROS_INFO("Set Turn_Right_Permanent");
                    }

                    ROS_INFO("Initial turn completed, switching to center mode to find X sign");
                }
                else if (intersection_initial_turn_done_ && intersection_x_sign_seen_ && intersection_flag_ >= 1)
                {
                    // After X sign and retry turn with one flag
                    // Now check for both lanes AND yaw condition
                    if (both_lanes_detected_ && intersection_yaw_recorded_)
                    {
                        double yaw_diff = calculateYawDifference(current_yaw_, intersection_initial_yaw_);

                        // Check if we've turned approximately 180 degrees (with tolerance)
                        if (fabs(yaw_diff - (0.0)) <= yaw_tolerance_)
                        {
                            ROS_INFO("Intersection success: both lanes detected and yaw diff=%.2f degrees", yaw_diff);
                            intersection_condition = true;
                        }
                        else
                        {
                            ROS_INFO_THROTTLE(1.0, "Both lanes detected but yaw not correct yet: %.2f degrees", yaw_diff);
                            // Continue following both lanes
                            if (turn_left_permanent_)
                            {
                                actual_driving_mode_ = "left";
                            }
                            else if (turn_right_permanent_)
                            {
                                actual_driving_mode_ = "right";
                            }
                            // else
                            // {
                            //     actual_driving_mode_ = "center";
                            // }
                        }
                    }
                }
            }
            if (current_mode_ == "intersection")
            {
                if (time_condition || intersection_condition)
                {
                    if (time_condition)
                        ROS_INFO("Duration completed: %.2f seconds", duration);
                    if (intersection_condition)
                        ROS_INFO("Intersection completed successfully with correct yaw");

                    // Reset yaw values for future use
                    if (intersection_condition)
                    {
                        intersection_yaw_recorded_ = false;
                        intersection_initial_yaw_ = 0.0;
                    }

                    success = true;
                    break;
                }
            }
            else if (current_mode_ == "cross_level")
            {
                if (time_condition || cross_level_condition)
                {
                    if (time_condition)
                        ROS_INFO("Duration completed: %.2f seconds", duration);
                    if (cross_level_condition)
                        ROS_INFO("Intersection completed successfully with detecting cross level");

                    success = true;
                    break;
                }
            }
            else
            {
                if (time_condition || sign_condition || lane_condition || turn_condition || intersection_condition)
                {
                    if (time_condition)
                        ROS_INFO("Duration completed: %.2f seconds", duration);
                    if (sign_condition)
                        ROS_INFO("Target sign detected: %s", target_sign_.c_str());
                    if (lane_condition)
                        ROS_INFO("Lane condition met (flag=%d) while in %s mode",
                                 intersection_flag_, current_mode_.c_str());
                    if (turn_condition)
                        ROS_INFO("Turn completed at target angle");
                    if (intersection_condition)
                        ROS_INFO("Intersection completed successfully with correct yaw");

                    // Reset yaw values for future use
                    if (intersection_condition)
                    {
                        intersection_yaw_recorded_ = false;
                        intersection_initial_yaw_ = 0.0;
                    }

                    success = true;
                    break;
                }
            }

            // Special handlers for intersection mode state transitions
            if (current_mode_ == "intersection")
            {
                // SIGN FINDING SEQUENCE - Add our new code here
                // Check for the initial sequence (before detecting any sign)
                if (!intersection_initial_turn_done_ && !sign_detected_)
                {
                    double yaw_diff = current_yaw_ - initial_yaw_;

                    // Normalize to [-π, π]
                    while (yaw_diff > M_PI)
                        yaw_diff -= 2 * M_PI;
                    while (yaw_diff < -M_PI)
                        yaw_diff += 2 * M_PI;

                    // If sign-finding is enabled and we've waited a bit without detecting a sign
                    if (use_sign_finding_ &&
                        !in_sign_finding_mode_ &&
                        both_lanes_detected_ &&                           // Only start when both lanes detected
                        fabs(fabs(yaw_diff) - (180.0)) <= yaw_tolerance_) // negative : counterclockwise
                    // (ros::Time::now() - start_time_).toSec() > 5.0)
                    {
                        // Start sign-finding mode
                        in_sign_finding_mode_ = true;
                        sign_finding_start_time_ = ros::Time::now();
                        sign_finding_forward_ = true;
                        sign_finding_attempts_ = 1;
                        ROS_INFO("Starting sign-finding sequence");
                    }

                    // If in sign-finding mode, perform the movement sequence
                    if (in_sign_finding_mode_)
                    {
                        // This will handle the sign-finding movement and logic
                        bool sign_found = performSignFinding();

                        if (sign_found)
                        {
                            ROS_INFO("Sign found or defaulted to: %s", intersection_turn_direction_.c_str());
                        }
                        else
                        {
                            // Skip the normal lane following when in sign-finding mode
                            // Update feedback
                            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();
                            as_.publishFeedback(feedback_);
                            r.sleep();
                            continue;
                        }
                    }
                }

                // X SIGN BYPASS - Add our code for handling X sign toggle
                // Handle the state after initial turn completed but before X sign
                if (intersection_initial_turn_done_ && !intersection_x_sign_seen_)
                {
                    if (!use_x_sign_)
                    {
                        // Skip X sign detection and simulate after a brief wait
                        if (!no_x_wait_start.isValid())
                        {
                            no_x_wait_start = ros::Time::now();
                            ROS_INFO("X sign detection disabled, will simulate after %.1f seconds",
                                     sign_no_x_wait_time_);
                        }

                        if ((ros::Time::now() - no_x_wait_start).toSec() >= sign_no_x_wait_time_)
                        {
                            // WAY 1
                            // intersection_flag_ = 0;
                            // current_mode_ = intersection_turn_direction_;

                            // WAY 2
                            intersection_x_sign_seen_ = true;
                            intersection_retry_count_ = 0; // Reset retry count
                            actual_driving_mode_ = intersection_turn_direction_;

                            ROS_INFO("Simulated X sign detection, continuing with %s turn",
                                     intersection_turn_direction_.c_str());
                        }
                    }
                }

                // EXISTING LOGIC - Keep the original state transitions
                if (!intersection_initial_turn_done_ && sign_detected_ && intersection_flag_ >= 1)
                {
                    // Check if initial turn has one flag
                    intersection_initial_turn_done_ = true;
                    intersection_flag_ = 0; // Reset counter
                    if (turn_left_permanent_)
                    {
                        actual_driving_mode_ = "left";
                    }
                    else if (turn_right_permanent_)
                    {
                        actual_driving_mode_ = "right";
                    }

                    ROS_INFO("Initial turn done, looking for X sign");
                }
                else if (intersection_initial_turn_done_ && intersection_x_sign_seen_ && intersection_retry_count_ == 0)
                {
                    // Start retry turn
                    // intersection_flag_ = 0;
                    actual_driving_mode_ = intersection_turn_direction_;
                    intersection_retry_count_ = 1;
                    ROS_INFO("Starting retry turn: %s", intersection_turn_direction_.c_str());
                }
            }

            ROS_INFO_THROTTLE(1.0, "Status: time_condition=%s, sign_condition=%s, intersection_condition=%s",
                              time_condition ? "true" : "false",
                              sign_condition ? "true" : "false",
                              intersection_condition ? "true" : "false");

            // Perform lane detection and robot control
            detectLane();

            // Calculate time elapsed for feedback
            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();

            // Publish feedback
            as_.publishFeedback(feedback_);

            r.sleep();
        }

        // Stop the robot when done
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Stopping robot");

        // Terminate sign detection if it was launched
        terminateSignDetection();

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
    ros::init(argc, argv, "lane_detect_server");
    LaneDetectServer laneDetectServer("lane_detect");
    // ros::AsyncSpinner spinner(4); // atau 2, tergantung CPU
    // spinner.start();
    // ros::waitForShutdown();
    ros::spin();
    return 0;
}