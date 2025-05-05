#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/LaneDetectAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>
#include <cmath>                    // For math functions
#include <nav_msgs/Odometry.h>      // For odometry data
#include <tf/transform_datatypes.h> // For quaternion to Euler conversion
#include <yaml-cpp/yaml.h>          // For YAML parsing
#include <ros/package.h>            // For package path resolution
#include <fstream>                  // For file handling
#include <signal.h>                 // For process termination

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
    std::string current_mode_;        // Current operation mode: "center", "left", "right", "intersection", etc.
    std::string actual_driving_mode_; // The actual driving mode (used by intersection)
    double x_previous_;               // Stored distance from lane to center
    bool both_lanes_detected_;        // Flag for detecting both lanes

    // Yaw tracking for turn modes
    double initial_yaw_;   // Initial yaw angle when turn started
    double current_yaw_;   // Current yaw angle
    bool is_turn_mode_;    // Whether we're in a turning mode
    bool yaw_initialized_; // Whether initial yaw has been set

    // Intersection detection variables
    int intersection_flag_;          // Counter for intersection detection
    bool previous_both_lanes_;       // Previous state of both lanes detection
    ros::Time last_transition_time_; // Time of last lane detection state change

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

    // Configuration file paths
    std::string speed_config_path_;
    std::string color_config_path_;

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
                                         detect_sign_process_id_(-1)
    {
        // Get camera topic from parameter server
        nh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_projected_compensated");

        // Define config file paths (using the src/config directory)
        speed_config_path_ = ros::package::getPath("action_bt") + "/../config/speed_conf.yaml";
        color_config_path_ = ros::package::getPath("action_bt") + "/../config/color_lane.yaml";

        // Load configuration parameters
        loadSpeedParameters();
        loadColorParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &LaneDetectServer::imageCallback, this);

        // Subscribe to the traffic sign topic
        traffic_sign_sub_ = nh_.subscribe("/detect/traffic_sign", 10, &LaneDetectServer::trafficSignCallback, this);

        // Subscribe to odometry for yaw tracking
        odom_sub_ = nh_.subscribe("/odom", 10, &LaneDetectServer::odomCallback, this);

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

    // Callback for the traffic sign topic
    void trafficSignCallback(const std_msgs::UInt8::ConstPtr &msg)
    {
        // Store the sign data
        last_sign_data_ = msg->data;

        // Log sign detection
        ROS_INFO("SIGN CALLBACK TRIGGERED: data=%d, sign_detected=%s",
                 last_sign_data_, sign_detected_ ? "TRUE" : "FALSE");

        // Check if this is the sign we're waiting for
        if (target_sign_ == "intersection" && last_sign_data_ == 1)
        {
            // Intersection sign
            ROS_INFO("Intersection sign detected!");
            sign_detected_ = true;
        }
        else if (target_sign_ == "intersection-dynamic")
        {
            if (last_sign_data_ == 2) // Left sign
            {
                ROS_INFO("Left sign detected for intersection-dynamic!");
                sign_detected_ = true;
                actual_driving_mode_ = "left";
            }
            else if (last_sign_data_ == 3) // Right sign
            {
                ROS_INFO("Right sign detected for intersection-dynamic!");
                sign_detected_ = true;
                actual_driving_mode_ = "right";
            }
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
    }

    // Launch sign detection node based on target sign
    bool launchSignDetection()
    {
        std::string mission;

        // Determine the mission type based on target sign
        if (target_sign_ == "intersection" || target_sign_ == "intersection-dynamic")
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
            mission = "stop";
        }
        else
        {
            // No need to launch sign detection for other sign types
            return true;
        }

        ROS_INFO("Launching sign detection for mission: %s", mission.c_str());

        // Create command to launch the sign detection node
        std::string cmd = "roslaunch turtlebot3_autorace_detect detect_sign.launch mission:=" +
                          mission + " & echo $! > /tmp/sign_detect_pid";

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

        if (current_mode_ == "intersection-dynamic" && !actual_driving_mode_.empty())
        {
            std::string drive_text = "Driving: " + actual_driving_mode_;
            cv::putText(debug_img, drive_text, cv::Point(width / 2 - 80, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
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

        if (current_mode_ == "intersection-dynamic" && !actual_driving_mode_.empty())
        {
            // If in intersection-dynamic mode, use the detected direction for actual driving
            effective_mode = actual_driving_mode_;
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

            // Calculate region center based on detected lanes and current mode
            if (effective_mode == "center")
            {
                if (region_has_left[r] && region_has_right[r])
                {
                    // Both lanes detected, center is midpoint
                    region_centers[r] = (left_lane_x + right_lane_x) / 2;

                    // Store the lane width for future calculations
                    if (r == 0)
                    { // Only store from bottom region
                        x_previous_ = (right_lane_x - left_lane_x) / 2;
                    }
                }
                else if (region_has_left[r])
                {
                    // Only left lane, estimate center
                    region_centers[r] = left_lane_x + x_previous_;
                }
                else if (region_has_right[r])
                {
                    // Only right lane, estimate center
                    region_centers[r] = right_lane_x - x_previous_;
                }
            }
            else if (effective_mode == "left" ||
                     (is_turn_mode_ && lane_follow_mode == "left"))
            {
                // Left mode follows left lane with offset
                if (region_has_left[r])
                {
                    region_centers[r] = left_lane_x + x_previous_;
                }
                else if (region_has_right[r] && x_previous_ > 0)
                {
                    // If right lane detected and we have previous offset,
                    // estimate where left lane should be
                    region_centers[r] = right_lane_x - 2 * x_previous_;
                }
            }
            else if (effective_mode == "right" ||
                     (is_turn_mode_ && lane_follow_mode == "right"))
            {
                // Right mode follows right lane with offset
                if (region_has_right[r])
                {
                    region_centers[r] = right_lane_x - x_previous_;
                }
                else if (region_has_left[r] && x_previous_ > 0)
                {
                    // If left lane detected and we have previous offset,
                    // estimate where right lane should be
                    region_centers[r] = left_lane_x + 2 * x_previous_;
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
                }

                last_transition_time_ = current_time;
            }
        }

        // Display intersection flag count
        std::string flag_text = "Intersection Flag: " + std::to_string(intersection_flag_);
        cv::putText(debug_img, flag_text, cv::Point(10, 90),
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

            // Convert offset to steering value with sensitivity
            angular_z = -1 * avg_offset * max_angular_speed * steering_sensitivity;

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
        if (is_turn_mode_)
        {
            // Increase angular speed for turning modes to make turns faster
            if (current_mode_ == "just-turn-left")
            {
                // Make sure we're turning left (negative angular_z is right turn in ROS)
                if (angular_z > 0)
                {
                    angular_z = std::max(angular_z, max_angular_speed * 0.5); // At least 50% of max turn
                }
            }
            else if (current_mode_ == "just-turn-right")
            {
                // Make sure we're turning right (positive angular_z is left turn in ROS)
                if (angular_z < 0)
                {
                    angular_z = std::min(angular_z, -max_angular_speed * 0.5); // At least 50% of max turn
                }
            }
        }

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
            cv::putText(debug_img, follow_text, cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
        }

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

        // Set default mode if not specified
        if (current_mode_.empty())
        {
            current_mode_ = "center";
        }

        // Set actual driving mode
        actual_driving_mode_ = current_mode_;

        // If intersection-dynamic mode, start with center until a sign is detected
        if (current_mode_ == "intersection-dynamic")
        {
            actual_driving_mode_ = "center";
        }

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

        ROS_INFO("LaneDetect Server: Mode=%s, Duration=%.2f, Sign=%s",
                 current_mode_.c_str(), duration,
                 target_sign_.empty() ? "none" : target_sign_.c_str());

        // Launch sign detection if needed
        if (!target_sign_.empty() &&
            (target_sign_ == "intersection" ||
             target_sign_ == "intersection-dynamic" ||
             target_sign_ == "construction" ||
             target_sign_ == "tunnel" ||
             target_sign_ == "cross"))
        {
            if (!launchSignDetection())
            {
                ROS_ERROR("Failed to launch sign detection for %s", target_sign_.c_str());
                as_.setAborted();
                return;
            }
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

            // Duration-based success (if duration > 0)
            bool time_condition = (duration > 0) &&
                                  ((ros::Time::now() - start_time_) > ros::Duration(duration));

            // Sign detection success
            bool sign_condition = !target_sign_.empty() && sign_detected_;

            // Lane detection conditions based on mode
            bool lane_condition = false;

            if (current_mode_ == "intersection-dynamic")
            {
                // Intersection mode requires 2 times detection
                lane_condition = intersection_flag_ >= 2;
                if (lane_condition)
                {
                    ROS_INFO("Intersection mode: Detected both lanes twice (flag=%d)", intersection_flag_);
                }
            }
            else if (current_mode_ == "left" || current_mode_ == "right")
            {
                // Left/right modes require 1 time detection
                lane_condition = intersection_flag_ >= 1;
                if (lane_condition)
                {
                    ROS_INFO("Left/Right mode: Detected both lanes (flag=%d)", intersection_flag_);
                }
            }

            // Yaw-based turn success condition
            bool turn_condition = false;
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

            if (time_condition || sign_condition || lane_condition || turn_condition)
            {
                if (time_condition)
                    ROS_INFO("Duration completed: %.2f seconds", duration);
                if (sign_condition)
                    ROS_INFO("Target sign detected: %s", target_sign_.c_str());
                if (lane_condition)
                    ROS_INFO("Intersection detected (flag=%d) while in %s mode",
                             intersection_flag_, current_mode_.c_str());
                if (turn_condition)
                    ROS_INFO("Turn completed at target angle");

                success = true;

                break;
            }
            ROS_INFO_THROTTLE(1.0, "Status: time_condition=%s, sign_condition=%s, target_sign=%s, sign_detected=%s",
                              time_condition ? "true" : "false",
                              sign_condition ? "true" : "false",
                              target_sign_.c_str(),
                              sign_detected_ ? "true" : "false");

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
    ros::spin();
    return 0;
}