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
    image_transport::Subscriber image_sub_;

    // Parameters for detection
    std::string camera_topic_;
    double forward_speed_;
    int detection_threshold_;
    int cleared_threshold_;
    int roi_x_;
    int roi_y_;
    int roi_width_;
    int roi_height_;
    std::string detection_method_; // "combined" or "red_only"

    // HSV thresholds for red and white
    int hue_red_low1_, hue_red_high1_; // First range for red (0-10)
    int hue_red_low2_, hue_red_high2_; // Second range for red (170-180)
    int saturation_red_low_, saturation_red_high_;
    int value_red_low_, value_red_high_;

    int hue_white_low_, hue_white_high_;
    int saturation_white_low_, saturation_white_high_;
    int value_white_low_, value_white_high_;

    // Variables for processing
    cv::Mat img_bgr_;
    cv::Mat img_hsv_;
    bool image_received_;
    bool crossing_detected_;
    bool crossing_cleared_;
    ros::Time detection_time_;
    ros::Time start_time_;

public:
    CrossWalkServer(std::string name) : as_(nh_, name, boost::bind(&CrossWalkServer::executeCB, this, _1), false),
                                        action_name_(name),
                                        it_(nh_),
                                        image_received_(false),
                                        crossing_detected_(false),
                                        crossing_cleared_(false)
    {
        // Load parameters from YAML file
        loadParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &CrossWalkServer::imageCallback, this);

        as_.start();
        ROS_INFO("CrossWalk Action Server Started");
    }

    ~CrossWalkServer()
    {
        // Ensure the robot stops when the server shuts down
        stopRobot();
    }

    void loadParameters()
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
            std::string config_path = package_path + "/../config/cross_param.yaml";

            // Load the YAML file
            YAML::Node config = YAML::LoadFile(config_path);

            // Extract parameters with defaults
            camera_topic_ = config["cross_walking"]["camera_topic"] ? config["cross_walking"]["camera_topic"].as<std::string>() : "/camera/image_projected_compensated";

            forward_speed_ = config["cross_walking"]["forward_speed"] ? config["cross_walking"]["forward_speed"].as<double>() : 0.08;

            detection_threshold_ = config["cross_walking"]["detection_threshold"] ? config["cross_walking"]["detection_threshold"].as<int>() : 200;

            cleared_threshold_ = config["cross_walking"]["cleared_threshold"] ? config["cross_walking"]["cleared_threshold"].as<int>() : 50;

            // Detection method - default to combined
            detection_method_ = config["cross_walking"]["detection_method"] ? config["cross_walking"]["detection_method"].as<std::string>() : "combined";

            // ROI parameters
            roi_x_ = config["cross_walking"]["roi_x"] ? config["cross_walking"]["roi_x"].as<int>() : 120;
            roi_y_ = config["cross_walking"]["roi_y"] ? config["cross_walking"]["roi_y"].as<int>() : 150;
            roi_width_ = config["cross_walking"]["roi_width"] ? config["cross_walking"]["roi_width"].as<int>() : 80;
            roi_height_ = config["cross_walking"]["roi_height"] ? config["cross_walking"]["roi_height"].as<int>() : 40;

            // Red color thresholds (two ranges because red wraps around in HSV)
            hue_red_low1_ = config["cross_walking"]["hue_red_low1"] ? config["cross_walking"]["hue_red_low1"].as<int>() : 0;
            hue_red_high1_ = config["cross_walking"]["hue_red_high1"] ? config["cross_walking"]["hue_red_high1"].as<int>() : 10;
            hue_red_low2_ = config["cross_walking"]["hue_red_low2"] ? config["cross_walking"]["hue_red_low2"].as<int>() : 170;
            hue_red_high2_ = config["cross_walking"]["hue_red_high2"] ? config["cross_walking"]["hue_red_high2"].as<int>() : 180;
            saturation_red_low_ = config["cross_walking"]["saturation_red_low"] ? config["cross_walking"]["saturation_red_low"].as<int>() : 100;
            saturation_red_high_ = config["cross_walking"]["saturation_red_high"] ? config["cross_walking"]["saturation_red_high"].as<int>() : 255;
            value_red_low_ = config["cross_walking"]["value_red_low"] ? config["cross_walking"]["value_red_low"].as<int>() : 100;
            value_red_high_ = config["cross_walking"]["value_red_high"] ? config["cross_walking"]["value_red_high"].as<int>() : 255;

            // White color thresholds
            hue_white_low_ = config["cross_walking"]["hue_white_low"] ? config["cross_walking"]["hue_white_low"].as<int>() : 0;
            hue_white_high_ = config["cross_walking"]["hue_white_high"] ? config["cross_walking"]["hue_white_high"].as<int>() : 180;
            saturation_white_low_ = config["cross_walking"]["saturation_white_low"] ? config["cross_walking"]["saturation_white_low"].as<int>() : 0;
            saturation_white_high_ = config["cross_walking"]["saturation_white_high"] ? config["cross_walking"]["saturation_white_high"].as<int>() : 30;
            value_white_low_ = config["cross_walking"]["value_white_low"] ? config["cross_walking"]["value_white_low"].as<int>() : 200;
            value_white_high_ = config["cross_walking"]["value_white_high"] ? config["cross_walking"]["value_white_high"].as<int>() : 255;

            ROS_INFO("Loaded parameters from %s", config_path.c_str());
            ROS_INFO("  camera_topic: %s", camera_topic_.c_str());
            ROS_INFO("  forward_speed: %.2f", forward_speed_);
            ROS_INFO("  detection_threshold: %d", detection_threshold_);
            ROS_INFO("  cleared_threshold: %d", cleared_threshold_);
            ROS_INFO("  detection_method: %s", detection_method_.c_str());
            ROS_INFO("  ROI: x=%d, y=%d, w=%d, h=%d", roi_x_, roi_y_, roi_width_, roi_height_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading configuration YAML file: %s", e.what());
            ROS_WARN("Using default parameters");

            // Set default values
            camera_topic_ = "/camera/image_projected_compensated";
            forward_speed_ = 0.08;
            detection_threshold_ = 200;
            cleared_threshold_ = 50;
            detection_method_ = "combined";
            roi_x_ = 120;
            roi_y_ = 150;
            roi_width_ = 80;
            roi_height_ = 40;

            // Default red color thresholds
            hue_red_low1_ = 0;
            hue_red_high1_ = 10;
            hue_red_low2_ = 170;
            hue_red_high2_ = 180;
            saturation_red_low_ = 100;
            saturation_red_high_ = 255;
            value_red_low_ = 100;
            value_red_high_ = 255;

            // Default white color thresholds
            hue_white_low_ = 0;
            hue_white_high_ = 180;
            saturation_white_low_ = 0;
            saturation_white_high_ = 30;
            value_white_low_ = 200;
            value_white_high_ = 255;
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

    int detectCrossWalk()
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet!");
            return 0;
        }

        // Create debug image
        cv::Mat debug_img = img_bgr_.clone();

        // Define ROI
        cv::Rect roi(roi_x_, roi_y_, roi_width_, roi_height_);

        // Draw ROI on debug image
        cv::rectangle(debug_img, roi, cv::Scalar(0, 255, 0), 2);

        // Create masks for red color (using two ranges since red wraps around in HSV)
        cv::Mat red_mask1, red_mask2, red_mask;
        cv::inRange(img_hsv_,
                    cv::Scalar(hue_red_low1_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high1_, saturation_red_high_, value_red_high_),
                    red_mask1);
        cv::inRange(img_hsv_,
                    cv::Scalar(hue_red_low2_, saturation_red_low_, value_red_low_),
                    cv::Scalar(hue_red_high2_, saturation_red_high_, value_red_high_),
                    red_mask2);
        cv::bitwise_or(red_mask1, red_mask2, red_mask);

        // Final mask based on detection method
        cv::Mat final_mask;

        if (detection_method_ == "red_only")
        {
            // Use only red mask
            final_mask = red_mask.clone();

            // Display method on debug image
            cv::putText(debug_img, "Method: RED ONLY", cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            // Default: Use combined red and white masks (the original method)
            cv::Mat white_mask;
            cv::inRange(img_hsv_,
                        cv::Scalar(hue_white_low_, saturation_white_low_, value_white_low_),
                        cv::Scalar(hue_white_high_, saturation_white_high_, value_white_high_),
                        white_mask);

            // Combine red and white masks
            cv::bitwise_or(red_mask, white_mask, final_mask);

            // Display method on debug image
            cv::putText(debug_img, "Method: RED + WHITE", cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
        }

        // Apply ROI to the mask
        cv::Mat roi_mask = final_mask(roi);

        // Count non-zero pixels in the ROI
        int pixel_count = cv::countNonZero(roi_mask);

        // Display visualization
        // Show pixel count
        std::string count_text = "Pixel count: " + std::to_string(pixel_count);
        cv::putText(debug_img, count_text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // Show detection status
        std::string status;
        cv::Scalar status_color;

        if (crossing_detected_ && !crossing_cleared_)
        {
            status = "STOPPING AT CROSSING";
            status_color = cv::Scalar(0, 0, 255); // Red color
        }
        else if (crossing_detected_ && crossing_cleared_)
        {
            status = "CROSSING CLEARED";
            status_color = cv::Scalar(0, 255, 0); // Green color
        }
        else
        {
            status = "LOOKING FOR CROSSING";
            status_color = cv::Scalar(255, 255, 0); // Yellow color
        }

        cv::putText(debug_img, status, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2);

        // Show threshold values
        std::string threshold_text = "Detection: " + std::to_string(detection_threshold_) +
                                     ", Cleared: " + std::to_string(cleared_threshold_);
        cv::putText(debug_img, threshold_text, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        // Show result image
        cv::imshow("CrossWalk Detection", debug_img);

        // Also show the masks for debugging
        cv::Mat roi_debug = roi_mask.clone();
        cv::resize(roi_debug, roi_debug, cv::Size(roi_width_ * 3, roi_height_ * 3));
        cv::imshow("ROI Mask", roi_debug);

        cv::waitKey(1);

        return pixel_count;
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
        cmd.linear.x = forward_speed_;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    void executeCB(const msg_file::CrossWalkGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        crossing_detected_ = false;
        crossing_cleared_ = false;

        ros::Rate r(10); // 10 Hz loop rate

        ROS_INFO("CrossWalk action started");
        ROS_INFO("Using detection method: %s", detection_method_.c_str());

        // Wait for first image to arrive
        while (!image_received_ && ros::ok())
        {
            if ((ros::Time::now() - start_time_).toSec() > 5.0)
            {
                ROS_ERROR("No camera image received after 5 seconds. Aborting mission.");
                as_.setAborted();
                return;
            }
            ROS_INFO_THROTTLE(1, "Waiting for camera image...");
            ros::spinOnce();
            r.sleep();
        }

        enum State
        {
            APPROACHING,  // Moving forward until crossing detected
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

            // Process image to detect crossing
            int pixel_count = detectCrossWalk();

            // State machine for crossing behavior
            switch (current_state)
            {
            case APPROACHING:
                // Move forward until crossing detected
                moveForward();

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

            case WAITING:
                // Wait until crossing cleared
                if (pixel_count <= cleared_threshold_)
                {
                    ROS_INFO("Crossing cleared! Moving forward.");
                    crossing_cleared_ = true;
                    cleared_time = ros::Time::now();
                    moveForward();
                    current_state = MOVING_AFTER;
                }
                break;

            case MOVING_AFTER:
                // Move forward for 1 second after crossing cleared
                if ((ros::Time::now() - cleared_time).toSec() >= 1.0)
                {
                    ROS_INFO("Finished moving after crossing. Mission complete.");
                    stopRobot();
                    current_state = COMPLETED;
                }
                break;

            case COMPLETED:
                // Should never reach this case due to loop condition
                break;
            }

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