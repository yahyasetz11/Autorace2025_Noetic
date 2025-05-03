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

    // Color thresholds
    int hue_yellow_l_, hue_yellow_h_;
    int saturation_yellow_l_, saturation_yellow_h_;
    int value_yellow_l_, value_yellow_h_;
    int hue_white_l_, hue_white_h_;
    int saturation_white_l_, saturation_white_h_;
    int value_white_l_, value_white_h_;

    // Variables
    cv::Mat img_bgr_;
    cv::Mat img_hsv_;
    bool image_received_;
    ros::Time start_time_;
    std::string current_mode_;
    bool obstacle_left_;
    bool obstacle_right_;
    bool white_dashed_line_detected_;
    bool border_line_detected_;
    bool yellow_lanes_detected_;

    // Laser scan data
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_;

public:
    ParkingServer(std::string name) : as_(nh_, name, boost::bind(&ParkingServer::executeCB, this, _1), false),
                                      action_name_(name),
                                      it_(nh_),
                                      image_received_(false),
                                      scan_received_(false)
    {
        // Load parameters from parameter server
        nh_.param<std::string>("/parking/camera_topic", camera_topic_, "/camera/image_projected_compensated");
        nh_.param<double>("/parking/max_linear_speed", max_linear_speed_, 0.08);
        nh_.param<double>("/parking/max_angular_speed", max_angular_speed_, 1.0);
        nh_.param<double>("/parking/scan_threshold_distance", scan_threshold_distance_, 0.5);
        nh_.param<double>("/parking/forward_speed", forward_speed_, 0.1);
        nh_.param<double>("/parking/backward_speed", backward_speed_, 0.1);
        nh_.param<double>("/parking/rotation_speed", rotation_speed_, 0.5);
        nh_.param<double>("/parking/parking_forward_distance", parking_forward_distance_, 0.5);
        nh_.param<double>("/parking/sleep_time", sleep_time_, 3.0);

        // Color thresholds
        nh_.param<int>("/parking/hue_yellow_l", hue_yellow_l_, 20);
        nh_.param<int>("/parking/hue_yellow_h", hue_yellow_h_, 40);
        nh_.param<int>("/parking/saturation_yellow_l", saturation_yellow_l_, 100);
        nh_.param<int>("/parking/saturation_yellow_h", saturation_yellow_h_, 255);
        nh_.param<int>("/parking/value_yellow_l", value_yellow_l_, 100);
        nh_.param<int>("/parking/value_yellow_h", value_yellow_h_, 255);
        nh_.param<int>("/parking/hue_white_l", hue_white_l_, 0);
        nh_.param<int>("/parking/hue_white_h", hue_white_h_, 179);
        nh_.param<int>("/parking/saturation_white_l", saturation_white_l_, 0);
        nh_.param<int>("/parking/saturation_white_h", saturation_white_h_, 70);
        nh_.param<int>("/parking/value_white_l", value_white_l_, 165);
        nh_.param<int>("/parking/value_white_h", value_white_h_, 255);

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &ParkingServer::imageCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &ParkingServer::scanCallback, this);

        as_.start();
        ROS_INFO("Parking Action Server Started");
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
            cv::cvtColor(img_bgr_, img_hsv_, cv::COLOR_BGR2HSV);
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

        // Create mask for yellow lanes
        cv::Mat yellow_mask;
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l_, saturation_yellow_l_, value_yellow_l_);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h_, saturation_yellow_h_, value_yellow_h_);
        cv::inRange(img_hsv_, lower_yellow, upper_yellow, yellow_mask);

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

    bool detectWhiteDashedLine(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return false;
        }

        // Create a more robust white mask with wider thresholds
        cv::Mat white_mask;
        // Widen the thresholds a bit to be more tolerant
        cv::Scalar lower_white = cv::Scalar(hue_white_l_, saturation_white_l_, value_white_l_ - 10); // Lower value threshold
        cv::Scalar upper_white = cv::Scalar(hue_white_h_, saturation_white_h_ + 20, value_white_h_); // Higher saturation
        cv::inRange(img_hsv_, lower_white, upper_white, white_mask);

        // Apply ROI mask - make it wider to detect from different angles
        int height = white_mask.rows;
        int width = white_mask.cols;
        int roi_height = height / 4; // 25% of image height
        int roi_y = height - roi_height;

        // Using a wider ROI (1/2 the width to 3/4 the width)
        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::rectangle(roi_mask, cv::Rect(width / 8, roi_y, width * 3 / 4, roi_height), cv::Scalar(255), -1);
        cv::bitwise_and(white_mask, roi_mask, white_mask);

        // Better noise removal
        cv::GaussianBlur(white_mask, white_mask, cv::Size(5, 5), 0);

        // Morphological operations to help identify dashed lines
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, kernel);

        // Multiple detection approaches for robustness

        // 1. Check for white pixel density in the ROI
        int white_pixels = cv::countNonZero(white_mask);
        int roi_area = roi_height * width * 3 / 4;
        double white_density = (double)white_pixels / roi_area;

        // 2. Use connected components to check for broken/dashed lines
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(white_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Count components of suitable size for dashed lines
        int dashed_segments = 0;
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > 20 && area < 800)
            { // Wider size range
                dashed_segments++;

                // Draw contours
                if (debug_img.data)
                {
                    cv::drawContours(debug_img, std::vector<std::vector<cv::Point>>{contour}, 0,
                                     cv::Scalar(0, 0, 255), 2);
                }
            }
        }

        // 3. Use Hough Lines to detect line segments
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(white_mask, lines, 1, CV_PI / 180, 30, 20, 10); // More sensitive parameters

        int horizontal_line_segments = 0;
        for (const auto &line : lines)
        {
            double angle = atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI;
            if (fabs(angle) < 30)
            { // Horizontal-ish
                horizontal_line_segments++;

                // Draw lines
                if (debug_img.data)
                {
                    cv::line(debug_img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                             cv::Scalar(0, 255, 255), 2);
                }
            }
        }

        // Combine detection methods for a more robust decision
        bool dashed_line_detected = false;

        if ((white_density > 0.05 && white_density < 0.25) || // Moderate white density
            dashed_segments >= 2 ||                           // Multiple segments
            horizontal_line_segments >= 2)
        { // Multiple horizontal lines
            dashed_line_detected = true;
        }

        white_dashed_line_detected_ = dashed_line_detected;

        // Enhanced visualization
        if (debug_img.data)
        {
            // Draw ROI
            cv::rectangle(debug_img, cv::Rect(width / 8, roi_y, width * 3 / 4, roi_height),
                          cv::Scalar(0, 0, 255), 2);

            // Add detailed text about detection
            std::string line_text = "Dashed Line: " + std::string(dashed_line_detected ? "DETECTED" : "NOT DETECTED");
            cv::putText(debug_img, line_text, cv::Point(10, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

            std::string density_text = "Density: " + std::to_string(white_density).substr(0, 5);
            cv::putText(debug_img, density_text, cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            std::string segments_text = "Segments: " + std::to_string(dashed_segments);
            cv::putText(debug_img, segments_text, cv::Point(10, 120),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            // Show white mask in corner
            cv::Mat white_viz;
            cv::cvtColor(white_mask, white_viz, cv::COLOR_GRAY2BGR);
            cv::Mat small_white;
            cv::resize(white_viz, small_white, cv::Size(width / 4, height / 4));
            debug_img(cv::Rect(width * 3 / 4, 0, width / 4, height / 4)) = small_white * 0.7;
        }

        return dashed_line_detected;
    }

    bool detectBorderLine(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return false;
        }

        // Create mask for white lines (border is white)
        cv::Mat white_mask;
        cv::Scalar lower_white = cv::Scalar(hue_white_l_, saturation_white_l_, value_white_l_);
        cv::Scalar upper_white = cv::Scalar(hue_white_h_, saturation_white_h_, value_white_h_);
        cv::inRange(img_hsv_, lower_white, upper_white, white_mask);

        // IMPORTANT CHANGE: Apply ROI mask - focus ONLY on the UPPER part of the bottom half
        // This helps distinguish the border line from the dashed line
        int height = white_mask.rows;
        int width = white_mask.cols;
        int roi_height = height / 6;         // Changed from height/3 to height/6
        int roi_y = height - 2 * roi_height; // Looking higher up than before

        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::rectangle(roi_mask, cv::Rect(0, roi_y, width, roi_height), cv::Scalar(255), -1);
        cv::bitwise_and(white_mask, roi_mask, white_mask);

        // Apply processing
        cv::GaussianBlur(white_mask, white_mask, cv::Size(5, 5), 0);

        // Use Hough Transform to detect lines
        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(white_mask, lines, 1, CV_PI / 180, 50, 80, 10); // Increased minLineLength from 50 to 80

        // Count horizontal-ish lines (border lines should be horizontal)
        int horizontal_lines = 0;
        for (size_t i = 0; i < lines.size(); i++)
        {
            cv::Vec4i l = lines[i];
            double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;

            // Check if line is approximately horizontal (-20 to +20 degrees - stricter than before)
            if (fabs(angle) < 20) // Changed from 30 to 20
            {
                // Check if the line is long enough (border lines are continuous)
                double length = sqrt(pow(l[2] - l[0], 2) + pow(l[3] - l[1], 2));
                if (length > width * 0.3) // Only count lines that are at least 30% of image width
                {
                    horizontal_lines++;

                    // Draw detected horizontal lines
                    if (debug_img.data)
                    {
                        cv::line(debug_img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
        }

        bool border_detected = horizontal_lines > 0;
        border_line_detected_ = border_detected;

        // Visualization
        if (debug_img.data)
        {
            // Draw ROI
            cv::rectangle(debug_img, cv::Rect(0, roi_y, width, roi_height),
                          cv::Scalar(255, 0, 0), 2);

            // Add text about detection
            std::string border_text = "Border Line: " + std::string(border_detected ? "DETECTED" : "NOT DETECTED");
            cv::putText(debug_img, border_text, cv::Point(10, 90),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        return border_detected;
    }

    void followCenterLane(cv::Mat &debug_img)
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet.");
            return;
        }

        // Create mask for yellow lanes
        cv::Mat yellow_mask;
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l_, saturation_yellow_l_, value_yellow_l_);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h_, saturation_yellow_h_, value_yellow_h_);
        cv::inRange(img_hsv_, lower_yellow, upper_yellow, yellow_mask);

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
        double angle_rad = fabs(angle) * M_PI * 1.1 / 180.0;

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

        ROS_INFO("Moving forward %.2f meters at %.2f m/s for %.2f seconds",
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
        ROS_INFO("Forward movement completed");
    }

    void moveBackward(double distance)
    {
        moveForward(distance, -backward_speed_);
    }

    void parking_move(std::string direction)
    {
        ROS_INFO("Executing parking move for %s side", direction.c_str());

        // Instead of separate rotation and forward movement,
        // we'll apply both linear and angular velocities simultaneously

        double angular_velocity = rotation_speed_;
        if (direction == "right")
        {
            angular_velocity = -rotation_speed_; // Negative for right turn
        }

        // Setup for border detection
        cv::Mat debug_img;
        ros::Rate rate(20); // 20 Hz for more responsive detection
        border_line_detected_ = false;

        // Keep track of accumulated angle to know when we've reached ~90 degrees
        double start_time = ros::Time::now().toSec();
        double current_time = start_time;
        double angle_moved = 0.0;         // In radians
        double target_angle = M_PI / 2.0; // 90 degrees in radians

        ROS_INFO("Starting curved parking maneuver");

        // Motion continues until either:
        // 1. Border line detected, or
        // 2. We've rotated approximately 90 degrees
        while (ros::ok() && !border_line_detected_ && angle_moved < target_angle)
        {
            // Process incoming messages
            ros::spinOnce();

            // Check for border line
            if (image_received_)
            {
                debug_img = img_bgr_.clone();
                detectBorderLine(debug_img);

                // Display debug image
                cv::imshow("Parking Debug", debug_img);
                cv::waitKey(1);
            }

            // Calculate current angle moved
            current_time = ros::Time::now().toSec();
            double elapsed_time = current_time - start_time;
            angle_moved = fabs(angular_velocity) * elapsed_time;

            // Adjust linear velocity as we turn - start faster, slow down as we approach 90 degrees
            double progress = angle_moved / target_angle; // 0.0 to 1.0
            double linear_velocity = forward_speed_ * (1.0 - 0.7 * progress);

            // Create and send curved motion command
            geometry_msgs::Twist cmd;
            cmd.linear.x = linear_velocity;
            cmd.angular.z = angular_velocity;
            cmd_vel_pub_.publish(cmd);

            // Log progress
            if (int(elapsed_time * 2) % 2 == 0)
            { // Log every ~0.5 seconds
                ROS_INFO_THROTTLE(0.5, "Progress: %.2f%%, Angle: %.1f degrees, Linear: %.2f m/s",
                                  progress * 100.0, angle_moved * 180.0 / M_PI, linear_velocity);
            }

            rate.sleep();
        }

        // Stop robot
        stopRobot();

        // If stopped due to border detection
        if (border_line_detected_)
        {
            ROS_INFO("Border line detected, stopping curved motion");
        }
        else
        {
            ROS_INFO("Completed ~90 degree turn with curved trajectory");
        }

        // If we haven't seen a border line yet, move forward a bit more
        if (!border_line_detected_)
        {
            ROS_INFO("No border detected during curved motion, moving forward a bit more");
            double moved_distance = 0.0;
            double step_distance = 0.05;
            double max_forward_distance = 0.3;

            while (ros::ok() && !border_line_detected_ && moved_distance < max_forward_distance)
            {
                // Check for border line
                ros::spinOnce();
                if (image_received_)
                {
                    debug_img = img_bgr_.clone();
                    detectBorderLine(debug_img);
                    cv::imshow("Parking Debug", debug_img);
                    cv::waitKey(1);
                }

                // Move forward
                geometry_msgs::Twist cmd;
                cmd.linear.x = forward_speed_ * 0.7; // Reduced speed
                cmd.angular.z = 0.0;
                cmd_vel_pub_.publish(cmd);

                moved_distance += step_distance;
                rate.sleep();
            }

            stopRobot();
        }

        // Wait in parking spot
        ROS_INFO("Waiting in parking spot for %.1f seconds", sleep_time_);
        ros::Duration(sleep_time_).sleep();
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

        // First rotate based on direction
        double angle = 90.0; // Left rotation
        if (direction == "right")
        {
            angle = -90.0; // Right rotation
        }

        rotate(angle);

        // CRITICAL FIX: Add a small forward movement to clear the dashed line
        // This ensures we're not detecting the dashed line as a border
        ROS_INFO("Moving forward a short distance to clear dashed line");
        moveForward(0.2, forward_speed_); // Move 20 cm forward

        // Now move forward until detecting border line
        cv::Mat debug_img2;
        ros::Rate rate(10); // 10 Hz
        double moved_distance = 0.0;
        double step_distance = 0.05;       // Move in small increments
        double max_forward_distance = 0.8; // Increased from 0.5 to 0.8

        border_line_detected_ = false;

        ROS_INFO("Moving forward until detecting border line");

        // Clear any cached detections
        ros::spinOnce();

        while (ros::ok() && !border_line_detected_ && moved_distance < max_forward_distance)
        {
            // Check for border line
            if (image_received_)
            {
                debug_img2 = img_bgr_.clone();
                detectBorderLine(debug_img2);

                // Display debug image
                cv::imshow("Parking Debug", debug_img2);
                cv::waitKey(1);
            }

            // Move forward a small step
            geometry_msgs::Twist cmd;
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd);

            moved_distance += step_distance;
            ros::spinOnce();
            rate.sleep();
        }

        // Stop after reaching border or maximum distance
        stopRobot();

        if (border_line_detected_)
        {
            ROS_INFO("Border line detected, stopping forward movement");
        }
        else
        {
            ROS_INFO("Reached maximum parking distance");
        }

        // Wait in parking spot
        ROS_INFO("Waiting in parking spot for %.1f seconds", sleep_time_);
        ros::Duration(sleep_time_).sleep();
    }

    void get_out()
    {
        ROS_INFO("Executing get_out maneuver");

        // CRUCIAL: Reset detection flags to avoid false positives from previous steps
        white_dashed_line_detected_ = false;

        // Force a small delay before starting backward movement to ensure
        // camera frames and sensor readings refresh
        ros::Duration(1.0).sleep();
        ros::spinOnce();

        ROS_INFO("Starting backward movement to find dashed line");

        // 1. Move backward until detecting the dashed line
        cv::Mat debug_img;
        double moved_distance = 0.0;
        double step_distance = 0.05; // Move in small increments
        double max_backing_distance = 0.7;

        // Use a reasonable but conservative backward speed
        double safe_backward_speed = 0.1;

        ros::Rate rate(10); // 10 Hz

        // Debug output
        ROS_INFO("Initial state - white_dashed_line_detected_: %s",
                 white_dashed_line_detected_ ? "TRUE" : "FALSE");

        // Main backward movement loop
        while (ros::ok() && !white_dashed_line_detected_ && moved_distance < max_backing_distance)
        {
            // Process any waiting messages to get updated sensor data
            ros::spinOnce();

            // Check for dashed white line
            if (image_received_)
            {
                debug_img = img_bgr_.clone();

                // Clear indication of current phase
                cv::putText(debug_img, "PHASE: BACKING UP", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

                // Log the moved distance
                std::string dist_text = "Moved: " + std::to_string(moved_distance) + " m";
                cv::putText(debug_img, dist_text, cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                // Check for dashed line
                white_dashed_line_detected_ = detectWhiteDashedLine(debug_img);

                if (white_dashed_line_detected_)
                {
                    ROS_INFO("Dashed line detected during backup at distance %.2f m", moved_distance);
                    stopRobot();
                    break;
                }

                // Display debug image
                cv::imshow("Parking Debug", debug_img);
                cv::waitKey(1);
            }

            // Explicitly create a backward movement command
            geometry_msgs::Twist cmd;
            cmd.linear.x = -safe_backward_speed; // Negative for reverse
            cmd.angular.z = 0.0;

            // Logging for velocity
            ROS_INFO_THROTTLE(1.0, "Moving backward at %.2f m/s, moved %.2f m",
                              safe_backward_speed, moved_distance);

            // Publish command
            cmd_vel_pub_.publish(cmd);

            moved_distance += step_distance;
            rate.sleep();
        }

        // Guarantee full stop after backing up
        stopRobot();
        ros::Duration(0.5).sleep();

        if (white_dashed_line_detected_)
        {
            ROS_INFO("Successfully detected dashed line after backing up %.2f meters", moved_distance);
        }
        else
        {
            ROS_WARN("Reached maximum backing distance (%.2f m) without finding dashed line",
                     max_backing_distance);
        }

        // 2. Rotate back (opposite direction of the parking move)
        double angle = -90.0; // Default rotation (right)
        if (current_mode_ == "right" ||
            (current_mode_ == "dynamic" && obstacle_left_ && !obstacle_right_))
        {
            angle = 90.0; // Rotate left to exit right-side parking
        }

        ROS_INFO("Rotating %.1f degrees to exit parking spot", angle);
        rotate(angle);

        // 3. Move forward until detecting yellow lanes
        ROS_INFO("Moving forward to find yellow lanes");
        yellow_lanes_detected_ = false; // Reset detection flag

        moved_distance = 0.0;
        double max_forward_distance = 1.0; // Maximum distance to move forward

        while (ros::ok() && !yellow_lanes_detected_ && moved_distance < max_forward_distance)
        {
            // Get latest camera data
            ros::spinOnce();

            // Check for yellow lanes
            if (image_received_)
            {
                debug_img = img_bgr_.clone();

                // Clear indication of current phase
                cv::putText(debug_img, "PHASE: EXITING", cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                yellow_lanes_detected_ = detectYellowLanes(debug_img);

                // Display debug image
                cv::imshow("Parking Debug", debug_img);
                cv::waitKey(1);
            }

            // Move forward
            geometry_msgs::Twist cmd;
            cmd.linear.x = forward_speed_;
            cmd.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd);

            moved_distance += step_distance;
            rate.sleep();
        }

        // Stop at the end
        stopRobot();

        if (yellow_lanes_detected_)
        {
            ROS_INFO("Yellow lanes detected, parking exit complete");
        }
        else
        {
            ROS_WARN("Could not find yellow lanes after exiting parking spot");
        }
    }

    void executeCB(const msg_file::ParkingGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        // Get parameters from goal
        current_mode_ = goal->mode;

        // Set default mode if not specified or invalid
        if (current_mode_.empty() ||
            (current_mode_ != "left" && current_mode_ != "right" && current_mode_ != "dynamic"))
        {
            current_mode_ = "dynamic";
            ROS_WARN("Invalid mode specified, defaulting to 'dynamic'");
        }

        // Reset detection flags
        obstacle_left_ = false;
        obstacle_right_ = false;
        white_dashed_line_detected_ = false;
        border_line_detected_ = false;
        yellow_lanes_detected_ = false;

        ROS_INFO("ParkingServer: Starting mission with mode=%s", current_mode_.c_str());

        // Add timeout for detection
        ros::Time start_detection_time = ros::Time::now();
        double max_detection_time = 30.0; // 30 seconds timeout

        // Main execution flow
        if (current_mode_ == "dynamic")
        {
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
                    cv::putText(debug_img, "Mode: " + current_mode_, cv::Point(10, 30),
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

                // Phase 3: Execute parking exit
                ROS_INFO("Phase 3: Exiting parking spot");
                get_out();
            }
        }
        else if (current_mode_ == "left" || current_mode_ == "right")
        {
            // Phase 1: Follow center lane until detecting dashed line
            ROS_INFO("Phase 1: Following center lane until dashed line");
            bool dashed_line_detected = false;

            while (ros::ok() && !dashed_line_detected)
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
                    ROS_WARN("Timed out waiting for dashed line detection. Proceeding with parking anyway.");
                    dashed_line_detected = true; // Force to proceed
                    break;
                }

                // Process messages to get latest data
                ros::spinOnce();

                // Create debug image
                cv::Mat debug_img;
                if (image_received_)
                {
                    debug_img = img_bgr_.clone();

                    // Display current mode
                    cv::putText(debug_img, "Mode: " + current_mode_, cv::Point(10, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

                    // IMPORTANT: Always check for dashed line
                    dashed_line_detected = detectWhiteDashedLine(debug_img);

                    if (dashed_line_detected)
                    {
                        ROS_INFO("DASHED LINE DETECTED! Stopping to begin parking.");
                        stopRobot();
                        // Add a small pause to stabilize
                        ros::Duration(0.5).sleep();
                    }

                    // Show debug image
                    cv::imshow("Parking Debug", debug_img);
                    cv::waitKey(1);
                }

                // Continue following center lane if no dashed line detected
                if (!dashed_line_detected)
                {
                    followCenterLane(debug_img);
                }

                // Calculate time elapsed for feedback
                feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();
                as_.publishFeedback(feedback_);

                r.sleep();
            }

            // Make sure to stop before proceeding to parking
            stopRobot();
            ros::Duration(0.5).sleep();

            // Phase 2: Execute fixed direction parking
            if (success)
            { // Only check success, we'll proceed whether dashed line was detected or not
                ROS_INFO("Phase 2: Executing %s parking", current_mode_.c_str());
                parking_move(current_mode_);

                // Phase 3: Execute parking exit
                ROS_INFO("Phase 3: Exiting parking spot");
                get_out();
            }
        }

        // Final status
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
    ros::init(argc, argv, "parking_server");
    ParkingServer parkingServer("parking");
    ros::spin();
    return 0;
}