#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/TrafficLightAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class TrafficLightServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::TrafficLightAction> as_;
    std::string action_name_;
    msg_file::TrafficLightFeedback feedback_;
    msg_file::TrafficLightResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // Parameters
    std::string camera_topic_;
    double green_confirmation_time_;
    int red_threshold_;
    int green_threshold_;

    // ROI parameters
    int roi_x_;
    int roi_y_;
    int roi_width_;
    int roi_height_;

    // HSV thresholds for red and green
    int hue_red_low1_, hue_red_high1_; // First range for red (0-10)
    int hue_red_low2_, hue_red_high2_; // Second range for red (170-180)
    int saturation_red_low_, saturation_red_high_;
    int value_red_low_, value_red_high_;

    int hue_green_low_, hue_green_high_;
    int saturation_green_low_, saturation_green_high_;
    int value_green_low_, value_green_high_;

    // Variables for processing
    cv::Mat img_bgr_;
    bool image_received_;
    ros::Time start_time_;
    ros::Time green_start_time_;
    bool green_confirmed_;
    std::string current_state_;

public:
    TrafficLightServer(std::string name) : as_(nh_, name, boost::bind(&TrafficLightServer::executeCB, this, _1), false),
                                           action_name_(name),
                                           it_(nh_),
                                           image_received_(false),
                                           green_confirmed_(false),
                                           current_state_("NONE")
    {
        // Load parameters from YAML file
        loadParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &TrafficLightServer::imageCallback, this);

        as_.start();
        ROS_INFO("Traffic Light Action Server Started");

        // Display loaded parameters
        ROS_INFO("Traffic Light Parameters:");
        ROS_INFO("  ROI: x=%d, y=%d, w=%d, h=%d", roi_x_, roi_y_, roi_width_, roi_height_);
        ROS_INFO("  Red HSV: H1=[%d,%d], H2=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 hue_red_low1_, hue_red_high1_, hue_red_low2_, hue_red_high2_,
                 saturation_red_low_, saturation_red_high_,
                 value_red_low_, value_red_high_);
        ROS_INFO("  Green HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 hue_green_low_, hue_green_high_,
                 saturation_green_low_, saturation_green_high_,
                 value_green_low_, value_green_high_);
        ROS_INFO("  Red threshold: %d pixels", red_threshold_);
        ROS_INFO("  Green threshold: %d pixels", green_threshold_);
        ROS_INFO("  Green confirmation time: %.2f seconds", green_confirmation_time_);
    }

    void loadParameters()
    {
        try
        {
            // Get the package path
            std::string package_path = ros::package::getPath("action_bt");

            // Build the path to the config file
            std::string config_path = package_path + "/../config/traffic_light_param.yaml";

            ROS_INFO("Loading parameters from: %s", config_path.c_str());

            // Load the YAML file
            YAML::Node config = YAML::LoadFile(config_path);

            // Extract parameters with defaults
            camera_topic_ = config["traffic_light"]["camera_topic"] ? config["traffic_light"]["camera_topic"].as<std::string>() : "/camera/image_projected_compensated";

            green_confirmation_time_ = config["traffic_light"]["green_confirmation_time"] ? config["traffic_light"]["green_confirmation_time"].as<double>() : 1.0;
            red_threshold_ = config["traffic_light"]["red_threshold"] ? config["traffic_light"]["red_threshold"].as<int>() : 100;
            green_threshold_ = config["traffic_light"]["green_threshold"] ? config["traffic_light"]["green_threshold"].as<int>() : 100;

            // ROI parameters
            roi_x_ = config["traffic_light"]["roi_x"] ? config["traffic_light"]["roi_x"].as<int>() : 120;
            roi_y_ = config["traffic_light"]["roi_y"] ? config["traffic_light"]["roi_y"].as<int>() : 80;
            roi_width_ = config["traffic_light"]["roi_width"] ? config["traffic_light"]["roi_width"].as<int>() : 80;
            roi_height_ = config["traffic_light"]["roi_height"] ? config["traffic_light"]["roi_height"].as<int>() : 60;

            // Red color thresholds (two ranges because red wraps around in HSV)
            hue_red_low1_ = config["traffic_light"]["hue_red_low1"] ? config["traffic_light"]["hue_red_low1"].as<int>() : 0;
            hue_red_high1_ = config["traffic_light"]["hue_red_high1"] ? config["traffic_light"]["hue_red_high1"].as<int>() : 10;
            hue_red_low2_ = config["traffic_light"]["hue_red_low2"] ? config["traffic_light"]["hue_red_low2"].as<int>() : 170;
            hue_red_high2_ = config["traffic_light"]["hue_red_high2"] ? config["traffic_light"]["hue_red_high2"].as<int>() : 180;
            saturation_red_low_ = config["traffic_light"]["saturation_red_low"] ? config["traffic_light"]["saturation_red_low"].as<int>() : 100;
            saturation_red_high_ = config["traffic_light"]["saturation_red_high"] ? config["traffic_light"]["saturation_red_high"].as<int>() : 255;
            value_red_low_ = config["traffic_light"]["value_red_low"] ? config["traffic_light"]["value_red_low"].as<int>() : 100;
            value_red_high_ = config["traffic_light"]["value_red_high"] ? config["traffic_light"]["value_red_high"].as<int>() : 255;

            // Green color thresholds
            hue_green_low_ = config["traffic_light"]["hue_green_low"] ? config["traffic_light"]["hue_green_low"].as<int>() : 40;
            hue_green_high_ = config["traffic_light"]["hue_green_high"] ? config["traffic_light"]["hue_green_high"].as<int>() : 80;
            saturation_green_low_ = config["traffic_light"]["saturation_green_low"] ? config["traffic_light"]["saturation_green_low"].as<int>() : 100;
            saturation_green_high_ = config["traffic_light"]["saturation_green_high"] ? config["traffic_light"]["saturation_green_high"].as<int>() : 255;
            value_green_low_ = config["traffic_light"]["value_green_low"] ? config["traffic_light"]["value_green_low"].as<int>() : 100;
            value_green_high_ = config["traffic_light"]["value_green_high"] ? config["traffic_light"]["value_green_high"].as<int>() : 255;

            ROS_INFO("Successfully loaded parameters from YAML");
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading YAML file: %s", e.what());
            ROS_WARN("Using default parameters");

            // Set defaults in case of error
            camera_topic_ = "/camera/image_projected_compensated";
            green_confirmation_time_ = 1.0;
            red_threshold_ = 100;
            green_threshold_ = 100;

            // ROI parameters
            roi_x_ = 120;
            roi_y_ = 80;
            roi_width_ = 80;
            roi_height_ = 60;

            // Default red color thresholds
            hue_red_low1_ = 0;
            hue_red_high1_ = 10;
            hue_red_low2_ = 170;
            hue_red_high2_ = 180;
            saturation_red_low_ = 100;
            saturation_red_high_ = 255;
            value_red_low_ = 100;
            value_red_high_ = 255;

            // Default green color thresholds
            hue_green_low_ = 40;
            hue_green_high_ = 80;
            saturation_green_low_ = 100;
            saturation_green_high_ = 255;
            value_green_low_ = 100;
            value_green_high_ = 255;
        }
    }

    ~TrafficLightServer()
    {
        // Ensure the robot stops when the server shuts down
        stopRobot();
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
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
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

    std::pair<int, int> detectTrafficLight(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return std::make_pair(0, 0);
        }

        // Create HSV image for color processing
        cv::Mat img_hsv;
        cv::cvtColor(img_bgr_, img_hsv, cv::COLOR_BGR2HSV);

        // Define ROI
        cv::Rect roi(roi_x_, roi_y_, roi_width_, roi_height_);
        cv::Mat roi_hsv = img_hsv(roi);

        // Create masks for red color (using two ranges since red wraps around in HSV)
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(roi_hsv,
                    cv::Scalar(hue_red_low1_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high1_, saturation_red_high_, value_red_high_),
                    red_mask1);
        cv::inRange(roi_hsv,
                    cv::Scalar(hue_red_low2_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high2_, saturation_red_high_, value_red_high_),
                    red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);

        // Create mask for green color
        cv::Mat green_mask;
        cv::inRange(roi_hsv,
                    cv::Scalar(hue_green_low_, saturation_green_low_, value_green_low_),
                    cv::Scalar(hue_green_high_, saturation_green_high_, value_green_high_),
                    green_mask);

        // Count non-zero pixels in each mask
        int red_pixels = cv::countNonZero(red_mask);
        int green_pixels = cv::countNonZero(green_mask);

        // Create debug visualization
        if (!debug_img.empty())
        {
            // Draw ROI on debug image
            cv::rectangle(debug_img, roi, cv::Scalar(255, 255, 255), 2);

            // Show state
            std::string state_text = "State: " + current_state_;
            cv::putText(debug_img, state_text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            // Show pixel counts
            std::string red_text = "Red pixels: " + std::to_string(red_pixels) +
                                   " (threshold: " + std::to_string(red_threshold_) + ")";
            cv::putText(debug_img, red_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

            std::string green_text = "Green pixels: " + std::to_string(green_pixels) +
                                     " (threshold: " + std::to_string(green_threshold_) + ")";
            cv::putText(debug_img, green_text, cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            // If green confirmation is in progress, show time
            if (current_state_ == "GREEN" && !green_confirmed_)
            {
                double time_left = green_confirmation_time_ -
                                   (ros::Time::now() - green_start_time_).toSec();
                std::string confirm_text = "Confirming green: " +
                                           std::to_string(time_left).substr(0, 4) + "s";
                cv::putText(debug_img, confirm_text, cv::Point(10, 120),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }

            // Show color detection in ROI
            cv::Mat red_roi = cv::Mat::zeros(roi.height, roi.width, CV_8UC3);
            cv::Mat green_roi = cv::Mat::zeros(roi.height, roi.width, CV_8UC3);

            // Convert masks to color
            cv::cvtColor(red_mask, red_roi, cv::COLOR_GRAY2BGR);
            cv::cvtColor(green_mask, green_roi, cv::COLOR_GRAY2BGR);

            // Tint masks with color
            red_roi = red_roi & cv::Scalar(0, 0, 255);
            green_roi = green_roi & cv::Scalar(0, 255, 0);

            // Display small masks in corner of image
            int display_width = roi.width / 2;
            int display_height = roi.height / 2;

            cv::Mat red_small, green_small;
            cv::resize(red_roi, red_small, cv::Size(display_width, display_height));
            cv::resize(green_roi, green_small, cv::Size(display_width, display_height));

            debug_img(cv::Rect(0, 0, display_width, display_height)) =
                debug_img(cv::Rect(0, 0, display_width, display_height)) * 0.5 + red_small * 0.5;

            debug_img(cv::Rect(display_width, 0, display_width, display_height)) =
                debug_img(cv::Rect(display_width, 0, display_width, display_height)) * 0.5 + green_small * 0.5;

            // Show visualization
            cv::imshow("Traffic Light Detection", debug_img);
            cv::waitKey(1);
        }

        return std::make_pair(red_pixels, green_pixels);
    }

    void executeCB(const msg_file::TrafficLightGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Get goal parameters
        float timeout = goal->timeout;
        if (timeout <= 0.0f)
        {
            timeout = 30.0f; // Default timeout if not specified
        }

        // Reset state
        current_state_ = "NONE";
        green_confirmed_ = false;

        ROS_INFO("Traffic Light Server: Timeout=%.2f seconds", timeout);

        // Wait until we receive camera data
        ros::Time wait_start = ros::Time::now();
        while (!image_received_ && ros::ok())
        {
            if ((ros::Time::now() - wait_start).toSec() > 5.0)
            {
                ROS_ERROR("No camera image received after 5 seconds!");
                result_.success = false;
                result_.final_state = "NONE";
                as_.setAborted(result_);
                return;
            }
            ROS_INFO_THROTTLE(1, "Waiting for camera image...");
            r.sleep();
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
                return;
            }

            // Create debug image
            cv::Mat debug_img;
            if (image_received_)
            {
                debug_img = img_bgr_.clone();
            }

            // Detect traffic light
            auto [red_pixels, green_pixels] = detectTrafficLight(debug_img);

            // Determine current state
            std::string previous_state = current_state_;

            if (red_pixels > red_threshold_)
            {
                current_state_ = "RED";
                stopRobot(); // Always stop at red light
                green_confirmed_ = false;
            }
            else if (green_pixels > green_threshold_)
            {
                current_state_ = "GREEN";

                // If first time seeing green, start confirmation timer
                if (previous_state != "GREEN")
                {
                    green_start_time_ = ros::Time::now();
                    green_confirmed_ = false;
                    ROS_INFO("Green light detected - starting confirmation timer");
                }

                // Check if green has been confirmed
                if (!green_confirmed_ &&
                    (ros::Time::now() - green_start_time_).toSec() >= green_confirmation_time_)
                {
                    green_confirmed_ = true;
                    ROS_INFO("Green light confirmed for %.2f seconds", green_confirmation_time_);
                }
            }
            else
            {
                current_state_ = "NONE";
                green_confirmed_ = false;
            }

            // Update feedback
            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();
            feedback_.current_state = current_state_;
            feedback_.red_pixels = red_pixels;
            feedback_.green_pixels = green_pixels;
            as_.publishFeedback(feedback_);

            // Check success conditions

            // 1. Green light is confirmed
            if (current_state_ == "GREEN" && green_confirmed_)
            {
                ROS_INFO("Green light confirmed - success!");
                success = true;
                result_.success = true;
                result_.final_state = "GREEN";
                as_.setSucceeded(result_);
                return;
            }

            // 2. Timeout if no traffic light detected
            if (current_state_ == "NONE" &&
                (ros::Time::now() - start_time_).toSec() > timeout)
            {
                ROS_INFO("Timeout after %.2f seconds with no traffic light detected", timeout);
                success = true;
                result_.success = true;
                result_.final_state = "TIMEOUT";
                as_.setSucceeded(result_);
                return;
            }

            r.sleep();
        }

        // Should never reach here unless ROS is shutdown
        result_.success = false;
        result_.final_state = current_state_;
        as_.setAborted(result_);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_light_server");
    TrafficLightServer trafficLightServer("traffic_light");
    ros::spin();
    return 0;
}