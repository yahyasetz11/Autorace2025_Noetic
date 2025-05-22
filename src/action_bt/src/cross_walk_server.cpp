#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/CrossWalkAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <cmath>

// Simple PID controller structure
struct PIDController
{
    double kp, ki, kd;
    double setpoint;
    double last_error;
    double sum_error;

    PIDController() : kp(0), ki(0), kd(0), setpoint(0), last_error(0), sum_error(0) {}

    void setGains(double p, double i, double d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

    double calculate(double input)
    {
        double error = setpoint - input;
        sum_error += error;
        double d_error = error - last_error;
        last_error = error;

        return kp * error + ki * sum_error + kd * d_error;
    }
};

class CrossWalkServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::CrossWalkAction> as_;
    std::string action_name_;
    msg_file::CrossWalkFeedback feedback_;
    msg_file::CrossWalkResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber front_image_sub_; // For crosswalk detection
    image_transport::Subscriber lane_image_sub_;  // For lane detection

    // Parameters for crosswalk detection
    std::string camera_topic_front_; // Front camera topic
    std::string camera_topic_lane_;  // Lane camera topic
    double forward_speed_;
    double max_angular_speed_;
    double forward_timeout_;
    int detection_threshold_;
    int cleared_threshold_;
    std::string detection_method_;

    // Configuration file paths
    std::string cross_config_path_;
    std::string lane_color_config_path_;
    std::string speed_config_path_;

    // Crosswalk ROI parameters
    int crosswalk_roi_x_, crosswalk_roi_y_;
    int crosswalk_roi_width_, crosswalk_roi_height_;

    // Lane detection parameters
    int hue_white_l_, hue_white_h_;
    int saturation_white_l_, saturation_white_h_;
    int value_white_l_, value_white_h_;
    int hue_yellow_l_, hue_yellow_h_;
    int saturation_yellow_l_, saturation_yellow_h_;
    int value_yellow_l_, value_yellow_h_;
    double lane_roi_height_factor_;

    // HSV thresholds for red and white crosswalk detection
    int hue_red_low1_, hue_red_high1_;
    int hue_red_low2_, hue_red_high2_;
    int saturation_red_low_, saturation_red_high_;
    int value_red_low_, value_red_high_;
    int hue_white_low_, hue_white_high_;
    int saturation_white_low_, saturation_white_high_;
    int value_white_low_, value_white_high_;

    // PID controller for lane following
    PIDController steering_pid_;

    // Variables for processing
    cv::Mat img_bgr_front_;     // Front camera image for crosswalk detection
    cv::Mat img_hsv_front_;     // Front camera HSV
    cv::Mat img_bgr_lane_;      // Lane camera image for lane detection
    cv::Mat img_hsv_lane_;      // Lane camera HSV
    bool front_image_received_; // Front camera image received flag
    bool lane_image_received_;  // Lane camera image received flag
    bool crossing_detected_;
    bool crossing_cleared_;
    ros::Time detection_time_;
    ros::Time start_time_;

public:
    CrossWalkServer(std::string name) : as_(nh_, name, boost::bind(&CrossWalkServer::executeCB, this, _1), false),
                                        action_name_(name),
                                        it_(nh_),
                                        front_image_received_(false),
                                        lane_image_received_(false),
                                        crossing_detected_(false),
                                        crossing_cleared_(false)
    {
        // Load parameters from YAML file
        loadParameters();

        // Initialize PID controller
        steering_pid_.setpoint = 0.0; // We want to center the robot

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers for both cameras
        ROS_INFO("Subscribing to front camera topic: %s", camera_topic_front_.c_str());
        ROS_INFO("Subscribing to lane camera topic: %s", camera_topic_lane_.c_str());
        front_image_sub_ = it_.subscribe(camera_topic_front_, 1, &CrossWalkServer::frontImageCallback, this);
        lane_image_sub_ = it_.subscribe(camera_topic_lane_, 1, &CrossWalkServer::laneImageCallback, this);

        as_.start();
        ROS_INFO("CrossWalk Action Server with Dual Camera Lane Detection Started");
    }

    ~CrossWalkServer()
    {
        stopRobot();
    }

    void loadParameters()
    {
        // Define config file paths
        std::string package_path = ros::package::getPath("action_bt");
        cross_config_path_ = package_path + "/../config/cross_param.yaml";
        lane_color_config_path_ = package_path + "/../config/color_lane.yaml";
        speed_config_path_ = package_path + "/../config/speed_conf.yaml";

        // Load crosswalk-specific parameters
        loadCrosswalkParameters();

        // Load lane detection parameters
        loadLaneColorParameters();

        // Load speed and PID parameters
        loadSpeedParameters();

        ROS_INFO("Successfully loaded parameters from multiple config files");
        ROS_INFO("  Cross config: %s", cross_config_path_.c_str());
        ROS_INFO("  Lane color config: %s", lane_color_config_path_.c_str());
        ROS_INFO("  Speed config: %s", speed_config_path_.c_str());
    }

    void loadCrosswalkParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(cross_config_path_);

            // Extract crosswalk-specific parameters
            camera_topic_front_ = config["cross_walking"]["camera_topic_front"] ? config["cross_walking"]["camera_topic_front"].as<std::string>() : "/camera/image";
            camera_topic_lane_ = config["cross_walking"]["camera_topic_lane"] ? config["cross_walking"]["camera_topic_lane"].as<std::string>() : "/camera/image_projected_compensated";
            forward_speed_ = config["cross_walking"]["forward_speed"] ? config["cross_walking"]["forward_speed"].as<double>() : 0.08;
            forward_timeout_ = config["cross_walking"]["forward_timeout"] ? config["cross_walking"]["forward_timeout"].as<double>() : 30.0;

            // Crosswalk detection parameters
            detection_threshold_ = config["cross_walking"]["detection_threshold"] ? config["cross_walking"]["detection_threshold"].as<int>() : 200;
            cleared_threshold_ = config["cross_walking"]["cleared_threshold"] ? config["cross_walking"]["cleared_threshold"].as<int>() : 50;
            detection_method_ = config["cross_walking"]["detection_method"] ? config["cross_walking"]["detection_method"].as<std::string>() : "combined";

            // Crosswalk ROI parameters
            crosswalk_roi_x_ = config["cross_walking"]["crosswalk_roi_x"] ? config["cross_walking"]["crosswalk_roi_x"].as<int>() : 120;
            crosswalk_roi_y_ = config["cross_walking"]["crosswalk_roi_y"] ? config["cross_walking"]["crosswalk_roi_y"].as<int>() : 150;
            crosswalk_roi_width_ = config["cross_walking"]["crosswalk_roi_width"] ? config["cross_walking"]["crosswalk_roi_width"].as<int>() : 80;
            crosswalk_roi_height_ = config["cross_walking"]["crosswalk_roi_height"] ? config["cross_walking"]["crosswalk_roi_height"].as<int>() : 40;

            // Crosswalk color thresholds
            hue_red_low1_ = config["cross_walking"]["hue_red_low1"] ? config["cross_walking"]["hue_red_low1"].as<int>() : 0;
            hue_red_high1_ = config["cross_walking"]["hue_red_high1"] ? config["cross_walking"]["hue_red_high1"].as<int>() : 10;
            hue_red_low2_ = config["cross_walking"]["hue_red_low2"] ? config["cross_walking"]["hue_red_low2"].as<int>() : 170;
            hue_red_high2_ = config["cross_walking"]["hue_red_high2"] ? config["cross_walking"]["hue_red_high2"].as<int>() : 180;
            saturation_red_low_ = config["cross_walking"]["saturation_red_low"] ? config["cross_walking"]["saturation_red_low"].as<int>() : 100;
            saturation_red_high_ = config["cross_walking"]["saturation_red_high"] ? config["cross_walking"]["saturation_red_high"].as<int>() : 255;
            value_red_low_ = config["cross_walking"]["value_red_low"] ? config["cross_walking"]["value_red_low"].as<int>() : 100;
            value_red_high_ = config["cross_walking"]["value_red_high"] ? config["cross_walking"]["value_red_high"].as<int>() : 255;

            hue_white_low_ = config["cross_walking"]["hue_white_low"] ? config["cross_walking"]["hue_white_low"].as<int>() : 0;
            hue_white_high_ = config["cross_walking"]["hue_white_high"] ? config["cross_walking"]["hue_white_high"].as<int>() : 180;
            saturation_white_low_ = config["cross_walking"]["saturation_white_low"] ? config["cross_walking"]["saturation_white_low"].as<int>() : 0;
            saturation_white_high_ = config["cross_walking"]["saturation_white_high"] ? config["cross_walking"]["saturation_white_high"].as<int>() : 30;
            value_white_low_ = config["cross_walking"]["value_white_low"] ? config["cross_walking"]["value_white_low"].as<int>() : 200;
            value_white_high_ = config["cross_walking"]["value_white_high"] ? config["cross_walking"]["value_white_high"].as<int>() : 255;

            ROS_INFO("Loaded crosswalk parameters from %s", cross_config_path_.c_str());
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading crosswalk configuration YAML file: %s", e.what());
            ROS_WARN("Using default crosswalk parameters");

            // Set crosswalk defaults
            camera_topic_front_ = "/camera/image";
            camera_topic_lane_ = "/camera/image_projected_compensated";
            forward_speed_ = 0.08;
            forward_timeout_ = 30.0;
            detection_threshold_ = 200;
            cleared_threshold_ = 50;
            detection_method_ = "combined";
            crosswalk_roi_x_ = 120;
            crosswalk_roi_y_ = 150;
            crosswalk_roi_width_ = 80;
            crosswalk_roi_height_ = 40;
        }
    }

    void loadLaneColorParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(lane_color_config_path_);

            // Extract white lane parameters
            if (config["white_lane"])
            {
                auto white_lane = config["white_lane"];
                hue_white_l_ = white_lane["hue"]["low"] ? white_lane["hue"]["low"].as<int>() : 0;
                hue_white_h_ = white_lane["hue"]["high"] ? white_lane["hue"]["high"].as<int>() : 179;
                saturation_white_l_ = white_lane["saturation"]["low"] ? white_lane["saturation"]["low"].as<int>() : 0;
                saturation_white_h_ = white_lane["saturation"]["high"] ? white_lane["saturation"]["high"].as<int>() : 70;
                value_white_l_ = white_lane["value"]["low"] ? white_lane["value"]["low"].as<int>() : 165;
                value_white_h_ = white_lane["value"]["high"] ? white_lane["value"]["high"].as<int>() : 255;
            }
            else
            {
                ROS_WARN("White lane section not found in color config file. Using defaults.");
                hue_white_l_ = 0;
                hue_white_h_ = 179;
                saturation_white_l_ = 0;
                saturation_white_h_ = 70;
                value_white_l_ = 165;
                value_white_h_ = 255;
            }

            // Extract yellow lane parameters
            if (config["yellow_lane"])
            {
                auto yellow_lane = config["yellow_lane"];
                hue_yellow_l_ = yellow_lane["hue"]["low"] ? yellow_lane["hue"]["low"].as<int>() : 10;
                hue_yellow_h_ = yellow_lane["hue"]["high"] ? yellow_lane["hue"]["high"].as<int>() : 50;
                saturation_yellow_l_ = yellow_lane["saturation"]["low"] ? yellow_lane["saturation"]["low"].as<int>() : 100;
                saturation_yellow_h_ = yellow_lane["saturation"]["high"] ? yellow_lane["saturation"]["high"].as<int>() : 255;
                value_yellow_l_ = yellow_lane["value"]["low"] ? yellow_lane["value"]["low"].as<int>() : 100;
                value_yellow_h_ = yellow_lane["value"]["high"] ? yellow_lane["value"]["high"].as<int>() : 255;
            }
            else
            {
                ROS_WARN("Yellow lane section not found in color config file. Using defaults.");
                hue_yellow_l_ = 10;
                hue_yellow_h_ = 50;
                saturation_yellow_l_ = 100;
                saturation_yellow_h_ = 255;
                value_yellow_l_ = 100;
                value_yellow_h_ = 255;
            }

            // Extract ROI parameters
            if (config["roi"])
            {
                lane_roi_height_factor_ = config["roi"]["height_factor"] ? config["roi"]["height_factor"].as<double>() : 0.5;
            }
            else
            {
                ROS_WARN("ROI section not found in color config file. Using default.");
                lane_roi_height_factor_ = 0.5;
            }

            ROS_INFO("Loaded lane color parameters from %s", lane_color_config_path_.c_str());
            ROS_INFO("  White lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                     hue_white_l_, hue_white_h_, saturation_white_l_, saturation_white_h_, value_white_l_, value_white_h_);
            ROS_INFO("  Yellow lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                     hue_yellow_l_, hue_yellow_h_, saturation_yellow_l_, saturation_yellow_h_, value_yellow_l_, value_yellow_h_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading lane color configuration YAML file: %s", e.what());
            ROS_WARN("Using default lane color parameters");

            // Set lane color defaults
            hue_white_l_ = 0;
            hue_white_h_ = 179;
            saturation_white_l_ = 0;
            saturation_white_h_ = 70;
            value_white_l_ = 165;
            value_white_h_ = 255;
            hue_yellow_l_ = 10;
            hue_yellow_h_ = 50;
            saturation_yellow_l_ = 100;
            saturation_yellow_h_ = 255;
            value_yellow_l_ = 100;
            value_yellow_h_ = 255;
            lane_roi_height_factor_ = 0.5;
        }
    }

    void loadSpeedParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(speed_config_path_);

            // Extract speed parameters
            max_angular_speed_ = config["max_angular_speed"] ? config["max_angular_speed"].as<double>() : 1.5;

            // Extract PID parameters
            double pid_kp = config["pid_kp"] ? config["pid_kp"].as<double>() : 5.0;
            double pid_ki = config["pid_ki"] ? config["pid_ki"].as<double>() : 0.0;
            double pid_kd = config["pid_kd"] ? config["pid_kd"].as<double>() : 0.0;

            // Set PID gains (note: kp is already negative in speed_conf.yaml)
            steering_pid_.setGains(pid_kp, pid_ki, pid_kd);

            ROS_INFO("Loaded speed parameters from %s", speed_config_path_.c_str());
            ROS_INFO("  max_angular_speed: %.2f rad/s", max_angular_speed_);
            ROS_INFO("  PID parameters: kp=%.3f, ki=%.3f, kd=%.3f", pid_kp, pid_ki, pid_kd);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading speed configuration YAML file: %s", e.what());
            ROS_WARN("Using default speed parameters");

            // Set speed defaults
            max_angular_speed_ = 1.5;
            steering_pid_.setGains(-5.0, 0.0, 0.7);
        }
    }

    void frontImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!front_image_received_)
        {
            ROS_INFO("Received first front camera image!");
        }

        try
        {
            img_bgr_front_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            front_image_received_ = true;
            cv::cvtColor(img_bgr_front_, img_hsv_front_, cv::COLOR_BGR2HSV);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert front image from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void laneImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        if (!lane_image_received_)
        {
            ROS_INFO("Received first lane camera image!");
        }

        try
        {
            img_bgr_lane_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            lane_image_received_ = true;
            cv::cvtColor(img_bgr_lane_, img_hsv_lane_, cv::COLOR_BGR2HSV);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert lane image from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    int detectCrossWalk(cv::Mat &debug_img)
    {
        if (!front_image_received_)
        {
            ROS_WARN_THROTTLE(1, "No front camera image received yet!");
            return 0;
        }

        // Define crosswalk ROI
        cv::Rect crosswalk_roi(crosswalk_roi_x_, crosswalk_roi_y_, crosswalk_roi_width_, crosswalk_roi_height_);

        // Create masks for red color (using two ranges since red wraps around in HSV)
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(img_hsv_front_,
                    cv::Scalar(hue_red_low1_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high1_, saturation_red_high_, value_red_high_),
                    red_mask1);
        cv::inRange(img_hsv_front_,
                    cv::Scalar(hue_red_low2_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high2_, saturation_red_high_, value_red_high_),
                    red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);

        // Final mask based on detection method
        cv::Mat final_mask;
        if (detection_method_ == "red_only")
        {
            final_mask = red_mask.clone();
        }
        else
        {
            // Combined red and white masks
            cv::Mat white_mask;
            cv::inRange(img_hsv_front_,
                        cv::Scalar(hue_white_low_, saturation_white_low_, value_white_low_),
                        cv::Scalar(hue_white_high_, saturation_white_high_, value_white_high_),
                        white_mask);
            cv::bitwise_or(red_mask, white_mask, final_mask);
        }

        // Apply crosswalk ROI to the mask
        cv::Mat roi_mask = final_mask(crosswalk_roi);
        int pixel_count = cv::countNonZero(roi_mask);

        // Visualization on FRONT camera image
        if (!debug_img.empty())
        {
            // Draw crosswalk ROI
            cv::rectangle(debug_img, crosswalk_roi, cv::Scalar(0, 255, 0), 2);

            // Show crosswalk detection info
            std::string count_text = "Crosswalk Pixels: " + std::to_string(pixel_count);
            cv::putText(debug_img, count_text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            // Show timeout information
            double time_elapsed = (ros::Time::now() - start_time_).toSec();
            std::string timeout_text = "Time: " + std::to_string((int)time_elapsed) + "/" + std::to_string((int)forward_timeout_) + "s";
            cv::putText(debug_img, timeout_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            // Show camera source
            cv::putText(debug_img, "FRONT CAMERA", cv::Point(10, debug_img.rows - 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
        }

        return pixel_count;
    }

    void followLaneAndDetectCrosswalk(cv::Mat &lane_debug_img)
    {
        if (!lane_image_received_)
        {
            ROS_WARN_THROTTLE(1, "No lane camera image received yet!");
            return;
        }

        int height = img_bgr_lane_.rows;
        int width = img_bgr_lane_.cols;
        int center_x = width / 2;

        // Create masks for white and yellow lanes using LANE camera image
        cv::Mat img_white_mask, img_yellow_mask;
        cv::Scalar lower_white = cv::Scalar(hue_white_l_, saturation_white_l_, value_white_l_);
        cv::Scalar upper_white = cv::Scalar(hue_white_h_, saturation_white_h_, value_white_h_);
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l_, saturation_yellow_l_, value_yellow_l_);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h_, saturation_yellow_h_, value_yellow_h_);

        cv::inRange(img_hsv_lane_, lower_white, upper_white, img_white_mask);
        cv::inRange(img_hsv_lane_, lower_yellow, upper_yellow, img_yellow_mask);

        // Combine masks
        cv::Mat lane_mask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::bitwise_or(img_white_mask, img_yellow_mask, lane_mask);

        // Apply lane ROI mask
        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);
        int roi_top = height * (1.0 - lane_roi_height_factor_);
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, height));
        roi_points.push_back(cv::Point(width, height));
        roi_points.push_back(cv::Point(width, roi_top));
        roi_points.push_back(cv::Point(0, roi_top));
        cv::fillConvexPoly(roi_mask, roi_points, cv::Scalar(255, 0, 0));
        cv::bitwise_and(lane_mask, roi_mask, lane_mask);

        // Apply Gaussian blur
        cv::GaussianBlur(lane_mask, lane_mask, cv::Size(5, 5), 0);

        // Calculate lane center using multiple regions
        int numRegions = 5;
        double total_offset = 0.0;
        int valid_regions = 0;
        double region_weights[3] = {3.0, 2.0, 1.0}; // Higher weight for bottom regions

        for (int r = 0; r < numRegions; r++)
        {
            int y = height - (r + 1) * (height / (numRegions + 1));
            int left_sum = 0, right_sum = 0;
            int left_weighted_sum = 0, right_weighted_sum = 0;

            // Scan for lane pixels in this region
            for (int x = 0; x < width; x++)
            {
                if (lane_mask.at<uchar>(y, x) > 0)
                {
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

            // Calculate lane center for this region if both lanes detected
            if (left_sum > 10 && right_sum > 10)
            {
                int left_lane_x = left_weighted_sum / left_sum;
                int right_lane_x = right_weighted_sum / right_sum;
                int lane_center = (left_lane_x + right_lane_x) / 2;

                // Calculate offset from image center, normalized to [-1, 1]
                double offset = (lane_center - center_x) / (double)(width / 2);
                total_offset += offset * region_weights[r];
                valid_regions += region_weights[r];

                // Debug visualization on LANE camera image
                if (!lane_debug_img.empty())
                {
                    cv::circle(lane_debug_img, cv::Point(left_lane_x, y), 3, cv::Scalar(255, 0, 0), -1);  // Blue for left
                    cv::circle(lane_debug_img, cv::Point(right_lane_x, y), 3, cv::Scalar(0, 255, 0), -1); // Green for right
                    cv::circle(lane_debug_img, cv::Point(lane_center, y), 5, cv::Scalar(0, 0, 255), -1);  // Red for center

                    // Draw line from image center to lane center
                    cv::line(lane_debug_img, cv::Point(center_x, y), cv::Point(lane_center, y), cv::Scalar(255, 255, 0), 2);
                }
            }
            else if (left_sum > 10 || right_sum > 10)
            {
                // If only one lane detected, use it with default offset
                int single_lane_x = (left_sum > 10) ? (left_weighted_sum / left_sum) : (right_weighted_sum / right_sum);

                // Estimate lane center based on single lane
                int estimated_center = (left_sum > 10) ? single_lane_x + 100 : single_lane_x - 100; // Rough estimate
                double offset = (estimated_center - center_x) / (double)(width / 2);
                total_offset += offset * region_weights[r] * 0.5; // Lower confidence
                valid_regions += region_weights[r] * 0.5;

                // Debug visualization for single lane
                if (!lane_debug_img.empty())
                {
                    cv::Scalar color = (left_sum > 10) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
                    cv::circle(lane_debug_img, cv::Point(single_lane_x, y), 3, color, -1);
                    cv::circle(lane_debug_img, cv::Point(estimated_center, y), 5, cv::Scalar(0, 255, 255), -1); // Cyan for estimated
                }
            }
        }

        // Calculate steering command
        double angular_z = 0.0;
        if (valid_regions > 0)
        {
            double avg_offset = total_offset / valid_regions;
            angular_z = steering_pid_.calculate(avg_offset);

            // Limit angular velocity
            if (angular_z > max_angular_speed_)
                angular_z = max_angular_speed_;
            if (angular_z < -max_angular_speed_)
                angular_z = -max_angular_speed_;

            ROS_INFO_THROTTLE(0.5, "Lane offset: %.3f, Angular: %.3f", avg_offset, angular_z);
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "No lane detected! Moving straight.");
            angular_z = 0.0;
        }

        // Calculate linear velocity (reduce when turning)
        double linear_x = forward_speed_ * (1.0 - 0.5 * fabs(angular_z / max_angular_speed_));

        // Send movement command
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd);

        // Debug info on LANE camera image
        if (!lane_debug_img.empty())
        {
            // Draw lane ROI
            cv::line(lane_debug_img, cv::Point(0, roi_top), cv::Point(width, roi_top), cv::Scalar(255, 255, 255), 2);

            // Show lane following info
            std::string lane_text = "Lane: L=" + std::to_string(linear_x).substr(0, 4) +
                                    " A=" + std::to_string(angular_z).substr(0, 4);
            cv::putText(lane_debug_img, lane_text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

            // Show valid regions count
            std::string regions_text = "Valid regions: " + std::to_string(valid_regions);
            cv::putText(lane_debug_img, regions_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

            // Show camera source
            cv::putText(lane_debug_img, "LANE CAMERA", cv::Point(10, lane_debug_img.rows - 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
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

    void executeCB(const msg_file::CrossWalkGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        crossing_detected_ = false;
        crossing_cleared_ = false;

        ros::Rate r(10); // 10 Hz loop rate

        ROS_INFO("CrossWalk action with lane following started");

        // Wait for both camera images
        while ((!front_image_received_ || !lane_image_received_) && ros::ok())
        {
            if ((ros::Time::now() - start_time_).toSec() > 5.0)
            {
                ROS_ERROR("Camera images not received after 5 seconds. Front: %s, Lane: %s",
                          front_image_received_ ? "OK" : "NO",
                          lane_image_received_ ? "OK" : "NO");
                as_.setAborted();
                return;
            }
            ROS_INFO_THROTTLE(1, "Waiting for camera images... Front: %s, Lane: %s",
                              front_image_received_ ? "OK" : "waiting",
                              lane_image_received_ ? "OK" : "waiting");
            ros::spinOnce();
            r.sleep();
        }

        enum State
        {
            APPROACHING,  // Following lane until crossing detected
            WAITING,      // Stopped at crossing
            MOVING_AFTER, // Moving forward after crossing cleared
            COMPLETED     // Action completed
        };

        State current_state = APPROACHING;
        ros::Time cleared_time;

        while (ros::ok() && current_state != COMPLETED)
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                stopRobot();
                break;
            }

            // Create debug images for both cameras
            cv::Mat front_debug_img, lane_debug_img;
            int pixel_count = 0;

            if (front_image_received_)
            {
                front_debug_img = img_bgr_front_.clone();
                pixel_count = detectCrossWalk(front_debug_img);
            }

            if (lane_image_received_)
            {
                lane_debug_img = img_bgr_lane_.clone();
            }

            // State machine
            switch (current_state)
            {
            case APPROACHING:
            {
                // Check for timeout first
                double time_elapsed = (ros::Time::now() - start_time_).toSec();
                if (time_elapsed >= forward_timeout_)
                {
                    ROS_INFO("Forward timeout reached (%.2f seconds). No crossing detected, completing mission successfully.", time_elapsed);
                    current_state = COMPLETED;
                    break;
                }

                // Follow lane while looking for crosswalk
                if (lane_image_received_)
                {
                    followLaneAndDetectCrosswalk(lane_debug_img);
                }

                // Check if crossing detected
                if (pixel_count >= detection_threshold_)
                {
                    ROS_INFO("Crossing detected! Stopping.");
                    stopRobot();
                    crossing_detected_ = true;
                    detection_time_ = ros::Time::now();
                    current_state = WAITING;
                }
                break;
            }

            case WAITING:
            {
                // Wait until crossing cleared
                if (pixel_count <= cleared_threshold_)
                {
                    ROS_INFO("Crossing cleared! Moving forward.");
                    crossing_cleared_ = true;
                    cleared_time = ros::Time::now();
                    current_state = MOVING_AFTER;
                }
                break;
            }

            case MOVING_AFTER:
            {
                // Continue lane following for 1 second after crossing cleared
                if (lane_image_received_)
                {
                    followLaneAndDetectCrosswalk(lane_debug_img);
                }

                if ((ros::Time::now() - cleared_time).toSec() >= 1.0)
                {
                    ROS_INFO("Finished moving after crossing. Mission complete.");
                    stopRobot();
                    current_state = COMPLETED;
                }
                break;
            }

            case COMPLETED:
                break;
            }

            // Show debug visualization for both cameras
            if (!front_debug_img.empty())
            {
                // Add state information to front debug image
                std::string state_text = "State: ";
                switch (current_state)
                {
                case APPROACHING:
                    state_text += "APPROACHING";
                    break;
                case WAITING:
                    state_text += "WAITING";
                    break;
                case MOVING_AFTER:
                    state_text += "MOVING_AFTER";
                    break;
                case COMPLETED:
                    state_text += "COMPLETED";
                    break;
                }
                int height = front_debug_img.rows;
                cv::putText(front_debug_img, state_text, cv::Point(10, height - 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                cv::imshow("CrossWalk Detection (Front Camera)", front_debug_img);
            }

            if (!lane_debug_img.empty())
            {
                cv::imshow("Lane Following (Lane Camera)", lane_debug_img);
            }

            if (front_debug_img.empty() && lane_debug_img.empty())
            {
                ROS_WARN_THROTTLE(2.0, "No debug images available - no camera data received");
            }

            cv::waitKey(1);

            // Publish feedback
            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();
            feedback_.current_state = static_cast<uint8_t>(current_state);
            feedback_.pixel_count = pixel_count;
            as_.publishFeedback(feedback_);

            r.sleep();
        }

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
    ros::init(argc, argv, "cross_walk_server");
    CrossWalkServer crossWalkServer("cross_walk");
    ros::spin();
    return 0;
}