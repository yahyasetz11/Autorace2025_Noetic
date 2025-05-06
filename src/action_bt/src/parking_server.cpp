#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/ParkingAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <cmath>

class ParkingServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::ParkingAction> as_;
    std::string action_name_;
    msg_file::ParkingFeedback feedback_;
    msg_file::ParkingResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scan_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    tf::TransformListener listener_;

    // Parameters
    std::string camera_topic_;
    double max_linear_speed_;
    double max_angular_speed_;
    double scan_threshold_distance_;
    double forward_speed_;
    double backward_speed_;
    double rotation_speed_;
    double parking_forward_distance_;
    double sleep_time_;

    // Color thresholds for lane detection
    int hue_yellow_l_, hue_yellow_h_;
    int saturation_yellow_l_, saturation_yellow_h_;
    int value_yellow_l_, value_yellow_h_;

    // Variables
    cv::Mat img_bgr_;
    bool image_received_;
    ros::Time start_time_;
    bool obstacle_left_;
    bool obstacle_right_;
    bool yellow_lanes_detected_;

    // Laser scan data
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_;

    // Load parameters from YAML
    void loadParameters()
    {
        try
        {
            // Get the package path
            std::string package_path = ros::package::getPath("action_bt");

            // Build the path to the config file
            std::string config_path = package_path + "/../config/parking_param.yaml";

            ROS_INFO("Loading parameters from: %s", config_path.c_str());

            // Load the YAML file
            YAML::Node config = YAML::LoadFile(config_path);

            // Extract parameters with defaults
            camera_topic_ = config["parking"]["camera_topic"] ? config["parking"]["camera_topic"].as<std::string>() : "/camera/image_projected_compensated";

            max_linear_speed_ = config["parking"]["max_linear_speed"] ? config["parking"]["max_linear_speed"].as<double>() : 0.08;
            max_angular_speed_ = config["parking"]["max_angular_speed"] ? config["parking"]["max_angular_speed"].as<double>() : 1.0;
            forward_speed_ = config["parking"]["forward_speed"] ? config["parking"]["forward_speed"].as<double>() : 0.12;
            backward_speed_ = config["parking"]["backward_speed"] ? config["parking"]["backward_speed"].as<double>() : 0.12;
            rotation_speed_ = config["parking"]["rotation_speed"] ? config["parking"]["rotation_speed"].as<double>() : 0.6;
            parking_forward_distance_ = config["parking"]["parking_forward_distance"] ? config["parking"]["parking_forward_distance"].as<double>() : 0.8;
            sleep_time_ = config["parking"]["sleep_time"] ? config["parking"]["sleep_time"].as<double>() : 3.0;
            scan_threshold_distance_ = config["parking"]["scan_threshold_distance"] ? config["parking"]["scan_threshold_distance"].as<double>() : 0.5;

            // Color thresholds for yellow lane detection
            hue_yellow_l_ = config["parking"]["hue_yellow_l"] ? config["parking"]["hue_yellow_l"].as<int>() : 20;
            hue_yellow_h_ = config["parking"]["hue_yellow_h"] ? config["parking"]["hue_yellow_h"].as<int>() : 40;
            saturation_yellow_l_ = config["parking"]["saturation_yellow_l"] ? config["parking"]["saturation_yellow_l"].as<int>() : 100;
            saturation_yellow_h_ = config["parking"]["saturation_yellow_h"] ? config["parking"]["saturation_yellow_h"].as<int>() : 255;
            value_yellow_l_ = config["parking"]["value_yellow_l"] ? config["parking"]["value_yellow_l"].as<int>() : 100;
            value_yellow_h_ = config["parking"]["value_yellow_h"] ? config["parking"]["value_yellow_h"].as<int>() : 255;

            ROS_INFO("Successfully loaded parameters from YAML");
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading YAML file: %s", e.what());
            ROS_WARN("Using default parameters");

            // Set defaults in case of error
            camera_topic_ = "/camera/image_projected_compensated";
            max_linear_speed_ = 0.08;
            max_angular_speed_ = 1.0;
            forward_speed_ = 0.12;
            backward_speed_ = 0.12;
            rotation_speed_ = 0.6;
            parking_forward_distance_ = 0.8;
            sleep_time_ = 3.0;
            scan_threshold_distance_ = 0.5;

            // Default yellow color thresholds
            hue_yellow_l_ = 20;
            hue_yellow_h_ = 40;
            saturation_yellow_l_ = 100;
            saturation_yellow_h_ = 255;
            value_yellow_l_ = 100;
            value_yellow_h_ = 255;
        }
    }

public:
    ParkingServer(std::string name) : as_(nh_, name, boost::bind(&ParkingServer::executeCB, this, _1), false),
                                      action_name_(name),
                                      it_(nh_),
                                      image_received_(false),
                                      scan_received_(false)
    {
        // Load parameters from YAML file
        loadParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &ParkingServer::imageCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &ParkingServer::scanCallback, this);

        as_.start();
        ROS_INFO("Parking Action Server Started");

        // Display loaded parameters
        ROS_INFO("Parking Parameters:");
        ROS_INFO("  max_linear_speed: %.2f", max_linear_speed_);
        ROS_INFO("  max_angular_speed: %.2f", max_angular_speed_);
        ROS_INFO("  scan_threshold_distance: %.2f", scan_threshold_distance_);
        ROS_INFO("  forward_speed: %.2f", forward_speed_);
        ROS_INFO("  backward_speed: %.2f", backward_speed_);
        ROS_INFO("  rotation_speed: %.2f", rotation_speed_);
        ROS_INFO("  parking_forward_distance: %.2f", parking_forward_distance_);
        ROS_INFO("  sleep_time: %.2f", sleep_time_);
        ROS_INFO("  Yellow HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 hue_yellow_l_, hue_yellow_h_,
                 saturation_yellow_l_, saturation_yellow_h_,
                 value_yellow_l_, value_yellow_h_);
    }

    ~ParkingServer()
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

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        latest_scan_ = *scan;
        scan_received_ = true;
    }

    bool detectObstacles(cv::Mat &debug_img)
    {
        if (!scan_received_)
        {
            ROS_WARN_THROTTLE(1, "No laser scan data received yet.");
            return false;
        }

        // Check for obstacles on left and right sides
        int left_index = latest_scan_.ranges.size() / 4;      // Approximately 90 degrees (left side)
        int right_index = latest_scan_.ranges.size() * 3 / 4; // Approximately 270 degrees (right side)
        int range = 10;                                       // Number of rays to check on each side

        // Check left side
        obstacle_left_ = false;
        for (int i = left_index - range / 2; i <= left_index + range / 2; i++)
        {
            int idx = (i + latest_scan_.ranges.size()) % latest_scan_.ranges.size(); // Handle wrap-around
            if (latest_scan_.ranges[idx] < scan_threshold_distance_ &&
                !std::isinf(latest_scan_.ranges[idx]) &&
                !std::isnan(latest_scan_.ranges[idx]))
            {
                obstacle_left_ = true;
                ROS_INFO("Obstacle detected on left side at %.2f meters", latest_scan_.ranges[idx]);
                break;
            }
        }

        // Check right side
        obstacle_right_ = false;
        for (int i = right_index - range / 2; i <= right_index + range / 2; i++)
        {
            int idx = (i + latest_scan_.ranges.size()) % latest_scan_.ranges.size(); // Handle wrap-around
            if (latest_scan_.ranges[idx] < scan_threshold_distance_ &&
                !std::isinf(latest_scan_.ranges[idx]) &&
                !std::isnan(latest_scan_.ranges[idx]))
            {
                obstacle_right_ = true;
                ROS_INFO("Obstacle detected on right side at %.2f meters", latest_scan_.ranges[idx]);
                break;
            }
        }

        // Visualization for debug image
        if (debug_img.data)
        {
            std::string status_left = obstacle_left_ ? "OBSTACLE" : "CLEAR";
            std::string status_right = obstacle_right_ ? "OBSTACLE" : "CLEAR";

            cv::putText(debug_img, "Left: " + status_left, cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        obstacle_left_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);

            cv::putText(debug_img, "Right: " + status_right, cv::Point(10, 150),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        obstacle_right_ ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
        }

        return obstacle_left_ || obstacle_right_;
    }

    bool detectYellowLanes(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return false;
        }

        // Create HSV image for color processing
        cv::Mat img_hsv;
        cv::cvtColor(img_bgr_, img_hsv, cv::COLOR_BGR2HSV);

        // Create mask for yellow lanes using configured HSV values
        cv::Mat yellow_mask;
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l_, saturation_yellow_l_, value_yellow_l_);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h_, saturation_yellow_h_, value_yellow_h_);
        cv::inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);

        // Apply ROI mask - focus only on the bottom half of the image
        int height = yellow_mask.rows;
        int width = yellow_mask.cols;
        int center_x = width / 2;

        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, height));
        roi_points.push_back(cv::Point(width, height));
        roi_points.push_back(cv::Point(width, height / 2));
        roi_points.push_back(cv::Point(0, height / 2));
        cv::fillConvexPoly(roi_mask, roi_points, cv::Scalar(255, 0, 0));
        cv::bitwise_and(yellow_mask, roi_mask, yellow_mask);

        // Apply image processing
        cv::GaussianBlur(yellow_mask, yellow_mask, cv::Size(5, 5), 0);

        // Check for yellow lanes on both sides
        int left_yellow_count = 0;
        int right_yellow_count = 0;
        int y_check = height - 20; // Check near the bottom

        // Count yellow pixels on left and right sides
        for (int x = 0; x < center_x; x++)
        {
            if (yellow_mask.at<uchar>(y_check, x) > 0)
            {
                left_yellow_count++;
            }
        }

        for (int x = center_x; x < width; x++)
        {
            if (yellow_mask.at<uchar>(y_check, x) > 0)
            {
                right_yellow_count++;
            }
        }

        // Determine if both lanes are detected
        bool both_lanes = (left_yellow_count > 10 && right_yellow_count > 10);
        yellow_lanes_detected_ = both_lanes;

        // Visualization
        if (debug_img.data)
        {
            cv::Mat yellow_viz;
            cv::cvtColor(yellow_mask, yellow_viz, cv::COLOR_GRAY2BGR);

            // Draw detection line
            cv::line(debug_img, cv::Point(0, y_check), cv::Point(width, y_check),
                     cv::Scalar(0, 255, 255), 2);

            // Add text about detection
            std::string lane_text = "Yellow Lanes: " + std::string(both_lanes ? "DETECTED" : "NOT DETECTED");
            cv::putText(debug_img, lane_text, cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

            // Show yellow mask in corner
            cv::Mat small_yellow;
            cv::resize(yellow_viz, small_yellow, cv::Size(width / 4, height / 4));
            debug_img(cv::Rect(0, 0, width / 4, height / 4)) = small_yellow * 0.7;
        }

        return both_lanes;
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Robot stopped");
    }

    void rotate(double angle)
    {
        geometry_msgs::Twist cmd;
        double angular_speed = rotation_speed_;
        if (angle < 0)
            angular_speed = -angular_speed;

        // Convert angle to radians
        double angle_rad = fabs(angle) * M_PI * 1.2 / 180.0;

        // Set the angular speed
        cmd.angular.z = angular_speed;

        // Calculate time to rotate
        double time_to_rotate = angle_rad / fabs(angular_speed);

        ROS_INFO("Rotating %.2f degrees at %.2f rad/s for %.2f seconds",
                 angle, angular_speed, time_to_rotate);

        // Start rotation
        ros::Time start = ros::Time::now();
        ros::Rate rate(50); // 50Hz

        while ((ros::Time::now() - start).toSec() < time_to_rotate && ros::ok())
        {
            cmd_vel_pub_.publish(cmd);
            rate.sleep();
            ros::spinOnce();
        }

        // Stop rotation
        stopRobot();
        ROS_INFO("Rotation completed");
    }

    void moveForward(double distance, double speed)
    {
        // Calculate time to move forward
        double time_to_move = distance / fabs(speed);

        ROS_INFO("Moving %.2f meters at %.2f m/s for %.2f seconds",
                 distance, speed, time_to_move);

        // Create and send movement command
        geometry_msgs::Twist cmd;
        cmd.linear.x = speed;
        cmd.angular.z = 0.0;

        // Start movement
        ros::Time start = ros::Time::now();
        ros::Rate rate(50); // 50Hz

        while ((ros::Time::now() - start).toSec() < time_to_move && ros::ok())
        {
            cmd_vel_pub_.publish(cmd);
            rate.sleep();
            ros::spinOnce();
        }

        // Stop movement
        stopRobot();
        ROS_INFO("Movement completed");
    }

    void moveBackward(double distance)
    {
        moveForward(distance, -backward_speed_);
    }

    void parking_move_dynamic()
    {
        ROS_INFO("Executing dynamic parking move");

        // Get debug image for visualization
        cv::Mat debug_img;
        if (image_received_)
        {
            debug_img = img_bgr_.clone();
        }

        // Detect obstacles to determine parking side
        detectObstacles(debug_img);

        // Determine direction based on detected obstacles
        std::string direction;
        if (obstacle_left_ && !obstacle_right_)
        {
            direction = "right";
            ROS_INFO("Obstacle on left, parking on right side");
        }
        else if (!obstacle_left_ && obstacle_right_)
        {
            direction = "left";
            ROS_INFO("Obstacle on right, parking on left side");
        }
        else if (obstacle_left_ && obstacle_right_)
        {
            ROS_WARN("Obstacles detected on both sides! Cannot park safely.");
            return;
        }
        else
        {
            // Default to right if no obstacles detected
            direction = "right";
            ROS_INFO("No obstacles detected, defaulting to right side parking");
        }

        // *** NEW STEP: Move forward for 1 second to ensure proper alignment ***
        ROS_INFO("Moving forward for 1 second to ensure proper alignment");
        geometry_msgs::Twist cmd;
        cmd.linear.x = forward_speed_;
        cmd.angular.z = 0.0;
        ros::Time start_time = ros::Time::now();
        ros::Rate delay_rate(20); // 20Hz rate

        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0)
        {
            cmd_vel_pub_.publish(cmd);
            delay_rate.sleep();
        }

        // Stop after delay
        stopRobot();
        ros::Duration(0.5).sleep(); // Small pause

        // Measure the distance behind using LIDAR (180 degrees - rear)
        double rear_distance = 0;
        if (scan_received_)
        {
            int rear_index = latest_scan_.ranges.size() / 2; // 180 degrees (back)
            int range = 10;                                  // Check a few rays around the back to get more reliable measurement
            int valid_readings = 0;

            for (int i = rear_index - range / 2; i <= rear_index + range / 2; i++)
            {
                int idx = (i + latest_scan_.ranges.size()) % latest_scan_.ranges.size(); // Handle wrap-around
                if (!std::isinf(latest_scan_.ranges[idx]) && !std::isnan(latest_scan_.ranges[idx]))
                {
                    rear_distance += latest_scan_.ranges[idx];
                    valid_readings++;
                }
            }

            if (valid_readings > 0)
            {
                rear_distance /= valid_readings; // Average of valid readings
                ROS_INFO("Measured rear distance: %.2f meters", rear_distance);
            }
            else
            {
                // If no valid readings, use a default value
                rear_distance = 1.0; // Default 1 meter
                ROS_WARN("No valid rear distance measurements, using default: %.2f meters", rear_distance);
            }
        }
        else
        {
            // If no scan received, use a default
            rear_distance = 1.0;
            ROS_WARN("No scan data received, using default rear distance: %.2f meters", rear_distance);
        }

        // Turn based on direction
        double angle = 90.0; // Left rotation
        if (direction == "right")
        {
            angle = -90.0; // Right rotation
        }
        rotate(angle);

        // Move forward for parking distance (could be parameter from yaml)
        // Using a fraction of the measured rear distance to ensure we don't hit anything
        double forward_distance = std::min(parking_forward_distance_, rear_distance * 0.7);
        ROS_INFO("Moving forward %.2f meters into parking spot", forward_distance);
        moveForward(forward_distance, forward_speed_);

        // Wait in parking spot
        ROS_INFO("Waiting in parking spot for %.1f seconds", sleep_time_);
        ros::Duration(sleep_time_).sleep();

        // Reverse out by the same distance
        ROS_INFO("Backing out of parking spot");
        moveForward(forward_distance, -backward_speed_);

        // Turn back to face original direction
        rotate(angle); // Same angle brings us back to original orientation

        // Move forward to exit completely
        ROS_INFO("Moving forward to clear parking area");
        moveForward(0.15, forward_speed_);
    }

    void followCenterLane(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return;
        }

        // Create HSV image for color processing
        cv::Mat img_hsv;
        cv::cvtColor(img_bgr_, img_hsv, cv::COLOR_BGR2HSV);

        // Create mask for yellow lanes using configured HSV values
        cv::Mat yellow_mask;
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l_, saturation_yellow_l_, value_yellow_l_);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h_, saturation_yellow_h_, value_yellow_h_);
        cv::inRange(img_hsv, lower_yellow, upper_yellow, yellow_mask);

        // Apply ROI mask for the bottom part of the image
        int height = yellow_mask.rows;
        int width = yellow_mask.cols;
        int center_x = width / 2;

        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);
        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, height));
        roi_points.push_back(cv::Point(width, height));
        roi_points.push_back(cv::Point(width, height / 2));
        roi_points.push_back(cv::Point(0, height / 2));
        cv::fillConvexPoly(roi_mask, roi_points, cv::Scalar(255, 0, 0));
        cv::bitwise_and(yellow_mask, roi_mask, yellow_mask);

        // Process yellow mask
        cv::GaussianBlur(yellow_mask, yellow_mask, cv::Size(5, 5), 0);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Filter and process contours
        std::vector<cv::Point> left_points, right_points;
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (contours[i].size() < 10)
                continue; // Filter small contours

            // Find the center of the contour
            cv::Moments m = cv::moments(contours[i]);
            if (m.m00 == 0)
                continue;

            int cx = int(m.m10 / m.m00);

            // Classify as left or right lane
            if (cx < center_x)
            {
                // Left lane
                for (auto &pt : contours[i])
                {
                    left_points.push_back(pt);
                }
            }
            else
            {
                // Right lane
                for (auto &pt : contours[i])
                {
                    right_points.push_back(pt);
                }
            }
        }

        // Draw contours for visualization
        if (debug_img.data)
        {
            if (!left_points.empty())
            {
                std::vector<std::vector<cv::Point>> left_contours = {left_points};
                cv::drawContours(debug_img, left_contours, 0, cv::Scalar(255, 0, 0), 2);
            }

            if (!right_points.empty())
            {
                std::vector<std::vector<cv::Point>> right_contours = {right_points};
                cv::drawContours(debug_img, right_contours, 0, cv::Scalar(0, 0, 255), 2);
            }
        }

        // Calculate the center line
        double left_avg_x = 0, right_avg_x = 0;
        if (!left_points.empty())
        {
            for (auto &pt : left_points)
            {
                left_avg_x += pt.x;
            }
            left_avg_x /= left_points.size();
        }
        else
        {
            left_avg_x = width * 0.2; // Default if no left lane detected
        }

        if (!right_points.empty())
        {
            for (auto &pt : right_points)
            {
                right_avg_x += pt.x;
            }
            right_avg_x /= right_points.size();
        }
        else
        {
            right_avg_x = width * 0.8; // Default if no right lane detected
        }

        // Calculate center point and error
        double center_point = (left_avg_x + right_avg_x) / 2.0;
        double error = (center_point - center_x) / (double)(width / 2); // Normalize to [-1, 1]

        // Draw center line for visualization
        if (debug_img.data)
        {
            cv::line(debug_img, cv::Point(center_point, height), cv::Point(center_point, height / 2),
                     cv::Scalar(0, 255, 0), 2);
            cv::line(debug_img, cv::Point(center_x, height), cv::Point(center_x, height / 2),
                     cv::Scalar(255, 255, 0), 1); // Reference center

            // Add error text
            std::string error_text = "Error: " + std::to_string(error);
            cv::putText(debug_img, error_text, cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        // Calculate angular velocity based on error
        double angular_z = -error * max_angular_speed_;

        // Limit maximum steering
        if (angular_z > max_angular_speed_)
            angular_z = max_angular_speed_;
        if (angular_z < -max_angular_speed_)
            angular_z = -max_angular_speed_;

        // Calculate linear velocity (reduce when turning)
        double linear_x = max_linear_speed_ * (1.0 - 0.7 * fabs(error));

        // Create and send movement command
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;
        cmd_vel_pub_.publish(cmd);

        ROS_INFO_THROTTLE(0.5, "Following center lane: linear.x=%.2f, angular.z=%.2f, error=%.2f",
                          cmd.linear.x, cmd.angular.z, error);
    }

    void executeCB(const msg_file::ParkingGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Reset detection flags
        obstacle_left_ = false;
        obstacle_right_ = false;
        yellow_lanes_detected_ = false;

        ROS_INFO("ParkingServer: Starting mission with dynamic mode");

        // Add timeout for detection
        ros::Time start_detection_time = ros::Time::now();
        double max_detection_time = 30.0; // 30 seconds timeout

        // Phase 1: Follow center lane until detecting obstacles
        ROS_INFO("Phase 1: Following center lane until obstacle detection");
        bool obstacles_detected = false;

        while (ros::ok() && !obstacles_detected)
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            // Check for timeout
            if ((ros::Time::now() - start_detection_time).toSec() > max_detection_time)
            {
                ROS_WARN("Timed out waiting for obstacle detection. Proceeding anyway.");
                obstacles_detected = true; // Force to proceed
                break;
            }

            // Create debug image
            cv::Mat debug_img;
            if (image_received_)
            {
                debug_img = img_bgr_.clone();

                // Display current mode
                cv::putText(debug_img, "Mode: Dynamic", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            }

            // Check for obstacles on both sides
            if (detectObstacles(debug_img))
            {
                obstacles_detected = true;
                ROS_INFO("Obstacles detected, stopping center lane following");
                stopRobot();
            }
            else
            {
                // Continue following center lane
                followCenterLane(debug_img);
            }

            // Show debug image
            if (image_received_)
            {
                cv::imshow("Parking Debug", debug_img);
                cv::waitKey(1);
            }

            // Calculate time elapsed for feedback
            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();
            as_.publishFeedback(feedback_);

            ros::spinOnce();
            r.sleep();
        }

        // Phase 2: Execute dynamic parking
        if (obstacles_detected && success)
        {
            ROS_INFO("Phase 2: Executing dynamic parking");
            parking_move_dynamic();
        }

        // Final status
        if (success)
        {
            result_.success = true;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
        else
        {
            result_.success = false;
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_server");
    ParkingServer parkingServer("parking");
    ros::spin();
    return 0;
}