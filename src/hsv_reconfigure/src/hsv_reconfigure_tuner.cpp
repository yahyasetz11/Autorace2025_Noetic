#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <hsv_reconfigure/HSVTunerConfig.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ros/package.h>

class HSVReconfigureTuner
{
private:
    // ROS NodeHandle
    ros::NodeHandle nh_;

    // Dynamic reconfigure server
    dynamic_reconfigure::Server<hsv_reconfigure::HSVTunerConfig> server_;
    dynamic_reconfigure::Server<hsv_reconfigure::HSVTunerConfig>::CallbackType f_;

    // Current parameters
    hsv_reconfigure::HSVTunerConfig config_;

    // Image transport and subscribers/publishers
    image_transport::ImageTransport it_;
    image_transport::Subscriber lane_image_sub_;  // For lane detection
    image_transport::Subscriber tl_cw_image_sub_; // For traffic light and crosswalk
    ros::Subscriber save_sub_;                    // For save trigger

    // Publishers for masked images
    image_transport::Publisher white_mask_pub_;
    image_transport::Publisher yellow_mask_pub_;
    image_transport::Publisher tl_red_mask_pub_;
    image_transport::Publisher tl_green_mask_pub_;
    image_transport::Publisher cw_red_mask_pub_;

    // Publishers for original and combined mask
    image_transport::Publisher original_pub_;
    image_transport::Publisher combined_mask_pub_;

    // Publishers for pixel counts
    ros::Publisher tl_red_count_pub_;
    ros::Publisher tl_green_count_pub_;
    ros::Publisher cw_red_count_pub_;

    // Current images
    cv::Mat lane_image_;
    cv::Mat tl_cw_image_;
    bool lane_image_received_;
    bool tl_cw_image_received_;

    // Path for saving configurations
    std::string output_dir_;
    std::string src_config_dir_;

    // Camera topics
    std::string lane_camera_topic_;
    std::string tl_cw_camera_topic_;

    // Current mode (lane, traffic_light, or cross_walk)
    std::string current_mode_;

    // Debug information
    int debug_count_;

public:
    HSVReconfigureTuner() : it_(nh_), lane_image_received_(false), tl_cw_image_received_(false), debug_count_(0)
    {
        // Get parameters with default values
        nh_.param<std::string>("lane_camera_topic", lane_camera_topic_, "/camera/image_projected_compensated");
        nh_.param<std::string>("tl_cw_camera_topic", tl_cw_camera_topic_, "/camera/image");
        nh_.param<std::string>("output_dir", output_dir_, ros::package::getPath("hsv_reconfigure") + "/config");
        nh_.param<std::string>("mode", current_mode_, "lane");

        // Find the src/config directory path
        src_config_dir_ = ros::package::getPath("action_bt") + "/../config";
        ROS_INFO("Loading initial parameters from: %s", src_config_dir_.c_str());

        // Set up publishers for masked images
        white_mask_pub_ = it_.advertise("white_mask", 1);
        yellow_mask_pub_ = it_.advertise("yellow_mask", 1);
        tl_red_mask_pub_ = it_.advertise("tl_red_mask", 1);
        tl_green_mask_pub_ = it_.advertise("tl_green_mask", 1);
        cw_red_mask_pub_ = it_.advertise("cw_red_mask", 1);

        // Set up publishers for original and combined mask
        original_pub_ = it_.advertise("original", 1);
        combined_mask_pub_ = it_.advertise("combined_mask", 1);

        // Set up publishers for pixel counts
        tl_red_count_pub_ = nh_.advertise<std_msgs::Int32>("tl_red_count", 1);
        tl_green_count_pub_ = nh_.advertise<std_msgs::Int32>("tl_green_count", 1);
        cw_red_count_pub_ = nh_.advertise<std_msgs::Int32>("cw_red_count", 1);

        // Set up subscriber for save trigger
        save_sub_ = nh_.subscribe("save", 1, &HSVReconfigureTuner::saveCallback, this);

        // Load existing parameters from YAML files
        loadParametersFromYAML();

        // Set up dynamic reconfigure callback
        f_ = boost::bind(&HSVReconfigureTuner::configCallback, this, _1, _2);
        server_.setCallback(f_);

        // Subscribe to camera topics - do this AFTER setting up publishers and loading parameters
        lane_image_sub_ = it_.subscribe(lane_camera_topic_, 1, &HSVReconfigureTuner::laneImageCallback, this);
        tl_cw_image_sub_ = it_.subscribe(tl_cw_camera_topic_, 1, &HSVReconfigureTuner::tlCwImageCallback, this);

        // Print current mode
        ROS_INFO("Current mode: %s", current_mode_.c_str());
        ROS_INFO("Subscribed to lane camera: %s", lane_camera_topic_.c_str());
        ROS_INFO("Subscribed to TL/CW camera: %s", tl_cw_camera_topic_.c_str());
        ROS_INFO("Press 's' to save configurations or publish to /hsv_reconfigure_tuner/save");

        // Create initial empty visualizations
        createInitialVisualizations();
    }

    ~HSVReconfigureTuner()
    {
    }

    void createInitialVisualizations()
    {
        // Create blank visualizations for each mask type until we get real data
        cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);

        // Traffic light visualizations
        cv::Rect tl_roi(config_.tl_roi_x, config_.tl_roi_y,
                        config_.tl_roi_width, config_.tl_roi_height);

        cv::Mat tl_red_viz = blank.clone();
        cv::Mat tl_green_viz = blank.clone();

        // Draw ROI boxes with THICK BRIGHT BORDERS
        cv::rectangle(tl_red_viz, tl_roi, cv::Scalar(0, 255, 255), 3);   // Yellow box
        cv::rectangle(tl_green_viz, tl_roi, cv::Scalar(0, 255, 255), 3); // Yellow box

        // Add labels
        cv::putText(tl_red_viz, "Traffic Light Red Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(tl_green_viz, "Traffic Light Green Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        // Crosswalk visualization
        cv::Rect cw_roi(config_.cw_roi_x, config_.cw_roi_y,
                        config_.cw_roi_width, config_.cw_roi_height);

        cv::Mat cw_red_viz = blank.clone();

        // Draw ROI box with THICK BRIGHT BORDER
        cv::rectangle(cw_red_viz, cw_roi, cv::Scalar(0, 255, 255), 3); // Yellow box

        // Add label
        cv::putText(cw_red_viz, "Crosswalk Red Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        // Publish initial visualizations
        sensor_msgs::ImagePtr tl_red_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_red_viz).toImageMsg();
        sensor_msgs::ImagePtr tl_green_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_green_viz).toImageMsg();
        sensor_msgs::ImagePtr cw_red_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cw_red_viz).toImageMsg();

        tl_red_mask_pub_.publish(tl_red_msg);
        tl_green_mask_pub_.publish(tl_green_msg);
        cw_red_mask_pub_.publish(cw_red_msg);

        // Also publish to combined mask
        sensor_msgs::ImagePtr combined_msg;

        if (current_mode_ == "traffic_light")
        {
            combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_red_viz).toImageMsg();
        }
        else if (current_mode_ == "cross_walk")
        {
            combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cw_red_viz).toImageMsg();
        }
        else
        {
            combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", blank).toImageMsg();
        }

        combined_mask_pub_.publish(combined_msg);

        ROS_INFO("Published initial visualization frames");
    }

    void loadParametersFromYAML()
    {
        // First set default values
        hsv_reconfigure::HSVTunerConfig default_config;
        config_ = default_config;

        // Now try to load from YAML files
        loadColorLaneParameters();
        loadTrafficLightParameters();
        loadCrossWalkParameters();

        // Update the server with the loaded parameters
        server_.updateConfig(config_);
    }

    void loadColorLaneParameters()
    {
        std::string color_lane_path = src_config_dir_ + "/color_lane.yaml";
        try
        {
            YAML::Node config = YAML::LoadFile(color_lane_path);

            // White lane parameters
            if (config["white_lane"])
            {
                auto white = config["white_lane"];
                config_.white_h_min = white["hue"]["low"].as<int>();
                config_.white_h_max = white["hue"]["high"].as<int>();
                config_.white_s_min = white["saturation"]["low"].as<int>();
                config_.white_s_max = white["saturation"]["high"].as<int>();
                config_.white_v_min = white["value"]["low"].as<int>();
                config_.white_v_max = white["value"]["high"].as<int>();
                ROS_INFO("Loaded white lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                         config_.white_h_min, config_.white_h_max,
                         config_.white_s_min, config_.white_s_max,
                         config_.white_v_min, config_.white_v_max);
            }

            // Yellow lane parameters
            if (config["yellow_lane"])
            {
                auto yellow = config["yellow_lane"];
                config_.yellow_h_min = yellow["hue"]["low"].as<int>();
                config_.yellow_h_max = yellow["hue"]["high"].as<int>();
                config_.yellow_s_min = yellow["saturation"]["low"].as<int>();
                config_.yellow_s_max = yellow["saturation"]["high"].as<int>();
                config_.yellow_v_min = yellow["value"]["low"].as<int>();
                config_.yellow_v_max = yellow["value"]["high"].as<int>();
                ROS_INFO("Loaded yellow lane HSV: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                         config_.yellow_h_min, config_.yellow_h_max,
                         config_.yellow_s_min, config_.yellow_s_max,
                         config_.yellow_v_min, config_.yellow_v_max);
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Failed to load color lane parameters: %s", e.what());
        }
    }

    void loadTrafficLightParameters()
    {
        std::string traffic_light_path = src_config_dir_ + "/traffic_light_param.yaml";
        try
        {
            YAML::Node config = YAML::LoadFile(traffic_light_path);

            if (config["traffic_light"])
            {
                auto tl = config["traffic_light"];

                // Red parameters
                config_.tl_red_h_min1 = tl["hue_red_low1"].as<int>();
                config_.tl_red_h_max1 = tl["hue_red_high1"].as<int>();
                config_.tl_red_h_min2 = tl["hue_red_low2"].as<int>();
                config_.tl_red_h_max2 = tl["hue_red_high2"].as<int>();
                config_.tl_red_s_min = tl["saturation_red_low"].as<int>();
                config_.tl_red_s_max = tl["saturation_red_high"].as<int>();
                config_.tl_red_v_min = tl["value_red_low"].as<int>();
                config_.tl_red_v_max = tl["value_red_high"].as<int>();

                // Green parameters
                config_.tl_green_h_min = tl["hue_green_low"].as<int>();
                config_.tl_green_h_max = tl["hue_green_high"].as<int>();
                config_.tl_green_s_min = tl["saturation_green_low"].as<int>();
                config_.tl_green_s_max = tl["saturation_green_high"].as<int>();
                config_.tl_green_v_min = tl["value_green_low"].as<int>();
                config_.tl_green_v_max = tl["value_green_high"].as<int>();

                // ROI parameters
                config_.tl_roi_x = tl["roi_x"].as<int>();
                config_.tl_roi_y = tl["roi_y"].as<int>();
                config_.tl_roi_width = tl["roi_width"].as<int>();
                config_.tl_roi_height = tl["roi_height"].as<int>();

                ROS_INFO("Loaded traffic light parameters successfully.");
                ROS_INFO("Traffic light ROI: x=%d, y=%d, w=%d, h=%d",
                         config_.tl_roi_x, config_.tl_roi_y,
                         config_.tl_roi_width, config_.tl_roi_height);
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Failed to load traffic light parameters: %s", e.what());
        }
    }

    void loadCrossWalkParameters()
    {
        std::string cross_walk_path = src_config_dir_ + "/cross_param.yaml";
        try
        {
            YAML::Node config = YAML::LoadFile(cross_walk_path);

            if (config["cross_walking"])
            {
                auto cw = config["cross_walking"];

                // Red parameters
                config_.cw_red_h_min1 = cw["hue_red_low1"].as<int>();
                config_.cw_red_h_max1 = cw["hue_red_high1"].as<int>();
                config_.cw_red_h_min2 = cw["hue_red_low2"].as<int>();
                config_.cw_red_h_max2 = cw["hue_red_high2"].as<int>();
                config_.cw_red_s_min = cw["saturation_red_low"].as<int>();
                config_.cw_red_s_max = cw["saturation_red_high"].as<int>();
                config_.cw_red_v_min = cw["value_red_low"].as<int>();
                config_.cw_red_v_max = cw["value_red_high"].as<int>();

                // ROI parameters
                config_.cw_roi_x = cw["roi_x"].as<int>();
                config_.cw_roi_y = cw["roi_y"].as<int>();
                config_.cw_roi_width = cw["roi_width"].as<int>();
                config_.cw_roi_height = cw["roi_height"].as<int>();

                ROS_INFO("Loaded crosswalk parameters successfully.");
                ROS_INFO("Crosswalk ROI: x=%d, y=%d, w=%d, h=%d",
                         config_.cw_roi_x, config_.cw_roi_y,
                         config_.cw_roi_width, config_.cw_roi_height);
            }
        }
        catch (const std::exception &e)
        {
            ROS_WARN("Failed to load crosswalk parameters: %s", e.what());
        }
    }

    void configCallback(hsv_reconfigure::HSVTunerConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request");
        config_ = config;

        // Process images based on current mode
        if (current_mode_ == "lane" && lane_image_received_)
        {
            processLaneImage();
        }
        else if (current_mode_ == "traffic_light" && tl_cw_image_received_)
        {
            processTrafficLightImage();
        }
        else if (current_mode_ == "cross_walk" && tl_cw_image_received_)
        {
            processCrossWalkImage();
        }
        else
        {
            // Create blank visualizations with the new ROI coordinates
            createInitialVisualizations();
        }
    }

    void laneImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            debug_count_++;
            if (debug_count_ % 10 == 0)
            {
                ROS_INFO("Lane image callback received frame %d", debug_count_);
            }

            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            lane_image_ = cv_ptr->image.clone();
            lane_image_received_ = true;

            // Process the image if in lane mode
            if (current_mode_ == "lane")
            {
                processLaneImage();
            }

            // Check for key press in OpenCV window
            int key = cv::waitKey(1);
            if (key == 's' || key == 'S')
            {
                saveParameters();
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception for lane image: %s", e.what());
        }
    }

    void tlCwImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // Increment counter for debugging
            debug_count_++;
            if (debug_count_ % 10 == 0)
            {
                ROS_INFO("TL/CW image callback received frame %d", debug_count_);
            }

            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            tl_cw_image_ = cv_ptr->image.clone();
            tl_cw_image_received_ = true;

            // Process the image based on mode
            if (current_mode_ == "traffic_light")
            {
                processTrafficLightImage();
            }
            else if (current_mode_ == "cross_walk")
            {
                processCrossWalkImage();
            }

            // Check for key press
            int key = cv::waitKey(1);
            if (key == 's' || key == 'S')
            {
                saveParameters();
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception for TL/CW image: %s", e.what());
        }
    }

    void saveCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        ROS_INFO("Save request received via ROS topic");
        saveParameters();
    }

    void processLaneImage()
    {
        if (!lane_image_received_)
        {
            ROS_WARN_THROTTLE(5, "Cannot process lane image - no image received yet");
            return;
        }

        // Convert BGR to HSV
        cv::Mat hsv_image;
        cv::cvtColor(lane_image_, hsv_image, cv::COLOR_BGR2HSV);

        // Create masks for lane detection
        cv::Mat white_mask, yellow_mask;

        // White lane mask
        cv::Scalar white_lower(config_.white_h_min, config_.white_s_min, config_.white_v_min);
        cv::Scalar white_upper(config_.white_h_max, config_.white_s_max, config_.white_v_max);
        cv::inRange(hsv_image, white_lower, white_upper, white_mask);

        // Yellow lane mask
        cv::Scalar yellow_lower(config_.yellow_h_min, config_.yellow_s_min, config_.yellow_v_min);
        cv::Scalar yellow_upper(config_.yellow_h_max, config_.yellow_s_max, config_.yellow_v_max);
        cv::inRange(hsv_image, yellow_lower, yellow_upper, yellow_mask);

        // Create combined mask for visualization
        cv::Mat combined_mask = cv::Mat::zeros(lane_image_.size(), CV_8UC3);

        // Create visualization masks from the binary masks
        cv::Mat white_mask_viz, yellow_mask_viz;
        cv::cvtColor(white_mask, white_mask_viz, cv::COLOR_GRAY2BGR);
        cv::cvtColor(yellow_mask, yellow_mask_viz, cv::COLOR_GRAY2BGR);

        // Add lane mode text to the visualizations
        cv::putText(white_mask_viz, "White Lane Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        cv::putText(yellow_mask_viz, "Yellow Lane Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

        // Add white lane to combined mask (in white)
        for (int y = 0; y < white_mask.rows; y++)
        {
            for (int x = 0; x < white_mask.cols; x++)
            {
                if (white_mask.at<uchar>(y, x) > 0)
                {
                    combined_mask.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // White in BGR
                }
            }
        }

        // Add yellow lane to combined mask (in yellow)
        for (int y = 0; y < yellow_mask.rows; y++)
        {
            for (int x = 0; x < yellow_mask.cols; x++)
            {
                if (yellow_mask.at<uchar>(y, x) > 0)
                {
                    combined_mask.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 255); // Yellow in BGR
                }
            }
        }

        // Add text with current mode
        cv::putText(combined_mask, "Mode: Lane Detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

        // Publish masked images with visualizations
        sensor_msgs::ImagePtr white_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", white_mask_viz).toImageMsg();
        sensor_msgs::ImagePtr yellow_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", yellow_mask_viz).toImageMsg();

        white_mask_pub_.publish(white_msg);
        yellow_mask_pub_.publish(yellow_msg);

        // Publish original and combined mask
        sensor_msgs::ImagePtr original_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", lane_image_).toImageMsg();
        sensor_msgs::ImagePtr combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", combined_mask).toImageMsg();

        original_pub_.publish(original_msg);
        combined_mask_pub_.publish(combined_msg);
    }

    void processTrafficLightImage()
    {
        if (!tl_cw_image_received_)
        {
            ROS_WARN_THROTTLE(5, "Cannot process traffic light image - no image received yet");
            return;
        }

        // Convert BGR to HSV
        cv::Mat hsv_image;
        cv::cvtColor(tl_cw_image_, hsv_image, cv::COLOR_BGR2HSV);

        // Create masks for traffic light detection
        cv::Mat tl_red_mask1, tl_red_mask2, tl_red_mask, tl_green_mask;

        // Traffic light red mask (two ranges)
        cv::Scalar tl_red_lower1(config_.tl_red_h_min1, config_.tl_red_s_min, config_.tl_red_v_min);
        cv::Scalar tl_red_upper1(config_.tl_red_h_max1, config_.tl_red_s_max, config_.tl_red_v_max);
        cv::Scalar tl_red_lower2(config_.tl_red_h_min2, config_.tl_red_s_min, config_.tl_red_v_min);
        cv::Scalar tl_red_upper2(config_.tl_red_h_max2, config_.tl_red_s_max, config_.tl_red_v_max);

        cv::inRange(hsv_image, tl_red_lower1, tl_red_upper1, tl_red_mask1);
        cv::inRange(hsv_image, tl_red_lower2, tl_red_upper2, tl_red_mask2);
        cv::bitwise_or(tl_red_mask1, tl_red_mask2, tl_red_mask);

        // Traffic light green mask
        cv::Scalar tl_green_lower(config_.tl_green_h_min, config_.tl_green_s_min, config_.tl_green_v_min);
        cv::Scalar tl_green_upper(config_.tl_green_h_max, config_.tl_green_s_max, config_.tl_green_v_max);
        cv::inRange(hsv_image, tl_green_lower, tl_green_upper, tl_green_mask);

        // Copy the original image for visualization
        cv::Mat visImage = tl_cw_image_.clone();

        // Create ROI for traffic light
        cv::Rect tl_roi(config_.tl_roi_x, config_.tl_roi_y,
                        config_.tl_roi_width, config_.tl_roi_height);

        // Ensure ROI is within image bounds
        tl_roi = tl_roi & cv::Rect(0, 0, tl_cw_image_.cols, tl_cw_image_.rows);

        // Draw ROI rectangle on visualization image
        cv::rectangle(visImage, tl_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // Calculate pixel counts for traffic light detection
        cv::Mat tl_red_roi = tl_red_mask(tl_roi);
        cv::Mat tl_green_roi = tl_green_mask(tl_roi);

        int tl_red_count = cv::countNonZero(tl_red_roi);
        int tl_green_count = cv::countNonZero(tl_green_roi);

        // Add text with pixel counts
        std::string tl_red_text = "TL Red Count: " + std::to_string(tl_red_count);
        std::string tl_green_text = "TL Green Count: " + std::to_string(tl_green_count);

        cv::putText(visImage, "Mode: Traffic Light Detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(visImage, tl_red_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(visImage, tl_green_text, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Highlight detected pixels in the ROI
        cv::Mat roi_area = visImage(tl_roi);
        for (int y = 0; y < roi_area.rows; y++)
        {
            for (int x = 0; x < roi_area.cols; x++)
            {
                if (tl_red_roi.rows > y && tl_red_roi.cols > x && tl_red_roi.at<uchar>(y, x) > 0)
                {
                    roi_area.at<cv::Vec3b>(y, x)[0] = 0;   // B
                    roi_area.at<cv::Vec3b>(y, x)[1] = 0;   // G
                    roi_area.at<cv::Vec3b>(y, x)[2] = 255; // R
                }
                else if (tl_green_roi.rows > y && tl_green_roi.cols > x && tl_green_roi.at<uchar>(y, x) > 0)
                {
                    roi_area.at<cv::Vec3b>(y, x)[0] = 0;   // B
                    roi_area.at<cv::Vec3b>(y, x)[1] = 255; // G
                    roi_area.at<cv::Vec3b>(y, x)[2] = 0;   // R
                }
            }
        }

        // Create combined mask to show the detection clearly
        cv::Mat combined_mask = cv::Mat::zeros(tl_cw_image_.size(), CV_8UC3);

        // Create red visualization (convert mask to color)
        cv::Mat red_viz = cv::Mat::zeros(tl_cw_image_.size(), CV_8UC3);
        for (int y = 0; y < tl_red_mask.rows; y++)
        {
            for (int x = 0; x < tl_red_mask.cols; x++)
            {
                if (tl_red_mask.at<uchar>(y, x) > 0)
                {
                    red_viz.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
                }
            }
        }

        // Create green visualization
        cv::Mat green_viz = cv::Mat::zeros(tl_cw_image_.size(), CV_8UC3);
        for (int y = 0; y < tl_green_mask.rows; y++)
        {
            for (int x = 0; x < tl_green_mask.cols; x++)
            {
                if (tl_green_mask.at<uchar>(y, x) > 0)
                {
                    green_viz.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0); // Green
                }
            }
        }

        // Add both visualizations to combined mask
        cv::add(combined_mask, red_viz, combined_mask);
        cv::add(combined_mask, green_viz, combined_mask);

        // Draw ROI on combined mask
        cv::rectangle(combined_mask, tl_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // Add text to combined mask
        cv::putText(combined_mask, "Mode: Traffic Light Detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(combined_mask, tl_red_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(combined_mask, tl_green_text, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Convert the binary masks to BGR for drawing the ROI box
        cv::Mat tl_red_mask_viz, tl_green_mask_viz;
        cv::cvtColor(tl_red_mask, tl_red_mask_viz, cv::COLOR_GRAY2BGR);
        cv::cvtColor(tl_green_mask, tl_green_mask_viz, cv::COLOR_GRAY2BGR);

        // Add text labels to the mask visualizations
        cv::putText(tl_red_mask_viz, "Traffic Light Red Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(tl_green_mask_viz, "Traffic Light Green Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Add pixel count information
        std::string tl_red_count_text = "Red Count: " + std::to_string(tl_red_count);
        std::string tl_green_count_text = "Green Count: " + std::to_string(tl_green_count);

        cv::putText(tl_red_mask_viz, tl_red_count_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        cv::putText(tl_green_mask_viz, tl_green_count_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        // Draw ROI box on the mask visualizations with thick borders
        cv::rectangle(tl_red_mask_viz, tl_roi, cv::Scalar(0, 255, 255), 3);   // Yellow box with thickness 3
        cv::rectangle(tl_green_mask_viz, tl_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // If masks are empty, make the ROI box more visible
        if (tl_red_count == 0)
        {
            // Add a bright outline inside the ROI to make it visible on black background
            cv::rectangle(tl_red_mask_viz, tl_roi, cv::Scalar(255, 255, 255), 1); // White inner box
        }

        if (tl_green_count == 0)
        {
            // Add a bright outline inside the ROI to make it visible on black background
            cv::rectangle(tl_green_mask_viz, tl_roi, cv::Scalar(255, 255, 255), 1); // White inner box
        }

        // Publish masked images with ROI box
        sensor_msgs::ImagePtr tl_red_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_red_mask_viz).toImageMsg();
        sensor_msgs::ImagePtr tl_green_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_green_mask_viz).toImageMsg();

        tl_red_mask_pub_.publish(tl_red_msg);
        tl_green_mask_pub_.publish(tl_green_msg);

        // Publish original and visualization
        sensor_msgs::ImagePtr original_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_cw_image_).toImageMsg();
        sensor_msgs::ImagePtr combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", combined_mask).toImageMsg();
        sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImage).toImageMsg();

        original_pub_.publish(original_msg);
        combined_mask_pub_.publish(combined_msg);

        // Publish pixel counts
        std_msgs::Int32 tl_red_count_msg, tl_green_count_msg;
        tl_red_count_msg.data = tl_red_count;
        tl_green_count_msg.data = tl_green_count;

        tl_red_count_pub_.publish(tl_red_count_msg);
        tl_green_count_pub_.publish(tl_green_count_msg);

        // Debug output
        if (debug_count_ % 30 == 0)
        {
            ROS_INFO("Traffic Light processing: Red count=%d, Green count=%d",
                     tl_red_count, tl_green_count);
        }
    }

    void processCrossWalkImage()
    {
        if (!tl_cw_image_received_)
        {
            ROS_WARN_THROTTLE(5, "Cannot process crosswalk image - no image received yet");
            return;
        }

        // Convert BGR to HSV
        cv::Mat hsv_image;
        cv::cvtColor(tl_cw_image_, hsv_image, cv::COLOR_BGR2HSV);

        // Create masks for crosswalk detection
        cv::Mat cw_red_mask1, cw_red_mask2, cw_red_mask;

        // Crosswalk red mask (two ranges)
        cv::Scalar cw_red_lower1(config_.cw_red_h_min1, config_.cw_red_s_min, config_.cw_red_v_min);
        cv::Scalar cw_red_upper1(config_.cw_red_h_max1, config_.cw_red_s_max, config_.cw_red_v_max);
        cv::Scalar cw_red_lower2(config_.cw_red_h_min2, config_.cw_red_s_min, config_.cw_red_v_min);
        cv::Scalar cw_red_upper2(config_.cw_red_h_max2, config_.cw_red_s_max, config_.cw_red_v_max);

        cv::inRange(hsv_image, cw_red_lower1, cw_red_upper1, cw_red_mask1);
        cv::inRange(hsv_image, cw_red_lower2, cw_red_upper2, cw_red_mask2);
        cv::bitwise_or(cw_red_mask1, cw_red_mask2, cw_red_mask);

        // Copy the original image for visualization
        cv::Mat visImage = tl_cw_image_.clone();

        // Create ROI for crosswalk
        cv::Rect cw_roi(config_.cw_roi_x, config_.cw_roi_y,
                        config_.cw_roi_width, config_.cw_roi_height);

        // Ensure ROI is within image bounds
        cw_roi = cw_roi & cv::Rect(0, 0, tl_cw_image_.cols, tl_cw_image_.rows);

        // Draw ROI rectangle on visualization image
        cv::rectangle(visImage, cw_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // Calculate pixel count for crosswalk detection
        cv::Mat cw_red_roi = cw_red_mask(cw_roi);
        int cw_red_count = cv::countNonZero(cw_red_roi);

        // Add text with pixel count
        std::string cw_red_text = "CW Red Count: " + std::to_string(cw_red_count);

        cv::putText(visImage, "Mode: Cross Walk Detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(visImage, cw_red_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Highlight detected pixels in the ROI
        cv::Mat roi_area = visImage(cw_roi);
        for (int y = 0; y < roi_area.rows; y++)
        {
            for (int x = 0; x < roi_area.cols; x++)
            {
                if (cw_red_roi.rows > y && cw_red_roi.cols > x && cw_red_roi.at<uchar>(y, x) > 0)
                {
                    roi_area.at<cv::Vec3b>(y, x)[0] = 0;   // B
                    roi_area.at<cv::Vec3b>(y, x)[1] = 0;   // G
                    roi_area.at<cv::Vec3b>(y, x)[2] = 255; // R
                }
            }
        }

        // Create combined mask for clear visualization
        cv::Mat combined_mask = cv::Mat::zeros(tl_cw_image_.size(), CV_8UC3);

        // Create red visualization
        for (int y = 0; y < cw_red_mask.rows; y++)
        {
            for (int x = 0; x < cw_red_mask.cols; x++)
            {
                if (cw_red_mask.at<uchar>(y, x) > 0)
                {
                    combined_mask.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255); // Red
                }
            }
        }

        // Draw ROI on combined mask
        cv::rectangle(combined_mask, cw_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // Add text to combined mask
        cv::putText(combined_mask, "Mode: Cross Walk Detection", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::putText(combined_mask, cw_red_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Convert the binary mask to BGR for drawing the ROI box
        cv::Mat cw_red_mask_viz;
        cv::cvtColor(cw_red_mask, cw_red_mask_viz, cv::COLOR_GRAY2BGR);

        // Add text labels to the mask visualization
        cv::putText(cw_red_mask_viz, "Crosswalk Red Mask", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Add pixel count information
        std::string cw_red_count_text = "Red Count: " + std::to_string(cw_red_count);
        cv::putText(cw_red_mask_viz, cw_red_count_text, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

        // Draw ROI box on the mask visualization with thick border
        cv::rectangle(cw_red_mask_viz, cw_roi, cv::Scalar(0, 255, 255), 3); // Yellow box with thickness 3

        // If mask is empty, make the ROI box more visible
        if (cw_red_count == 0)
        {
            // Add a bright outline inside the ROI to make it visible on black background
            cv::rectangle(cw_red_mask_viz, cw_roi, cv::Scalar(255, 255, 255), 1); // White inner box
        }

        // Publish masked image with ROI box
        sensor_msgs::ImagePtr cw_red_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cw_red_mask_viz).toImageMsg();
        cw_red_mask_pub_.publish(cw_red_msg);

        // Publish original and visualization
        sensor_msgs::ImagePtr original_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tl_cw_image_).toImageMsg();
        sensor_msgs::ImagePtr combined_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", combined_mask).toImageMsg();
        sensor_msgs::ImagePtr viz_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImage).toImageMsg();

        original_pub_.publish(original_msg);
        combined_mask_pub_.publish(combined_msg);

        // Publish pixel count
        std_msgs::Int32 cw_red_count_msg;
        cw_red_count_msg.data = cw_red_count;
        cw_red_count_pub_.publish(cw_red_count_msg);

        // Debug output
        if (debug_count_ % 30 == 0)
        {
            ROS_INFO("Cross Walk processing: Red count=%d", cw_red_count);
        }
    }

    void saveParameters()
    {
        ROS_INFO("Saving HSV parameters to config directory");

        // Make sure directory exists
        std::string command = "mkdir -p " + output_dir_;
        system(command.c_str());

        // Save configurations to YAML files
        saveColorLaneConfig();
        saveTrafficLightConfig();
        saveCrossWalkConfig();

        ROS_INFO("All parameters saved successfully!");
    }

    void saveColorLaneConfig()
    {
        std::string color_lane_path = output_dir_ + "/color_lane.yaml";
        std::ofstream file(color_lane_path);

        if (file.is_open())
        {
            file << "# HSV Parameters for Lane Detection saved by hsv_reconfigure\n";

            // White lane parameters
            file << "white_lane:\n";
            file << "  hue:\n";
            file << "    low: " << config_.white_h_min << "\n";
            file << "    high: " << config_.white_h_max << "\n";
            file << "  saturation:\n";
            file << "    low: " << config_.white_s_min << "\n";
            file << "    high: " << config_.white_s_max << "\n";
            file << "  value:\n";
            file << "    low: " << config_.white_v_min << "\n";
            file << "    high: " << config_.white_v_max << "\n";

            // Yellow lane parameters
            file << "\n# HSV thresholds for yellow lane\n";
            file << "yellow_lane:\n";
            file << "  hue:\n";
            file << "    low: " << config_.yellow_h_min << "\n";
            file << "    high: " << config_.yellow_h_max << "\n";
            file << "  saturation:\n";
            file << "    low: " << config_.yellow_s_min << "\n";
            file << "    high: " << config_.yellow_s_max << "\n";
            file << "  value:\n";
            file << "    low: " << config_.yellow_v_min << "\n";
            file << "    high: " << config_.yellow_v_max << "\n";

            // ROI settings
            file << "\n# ROI settings\n";
            file << "roi:\n";
            file << "  height_factor: 0.5 # Portion of the image height to use for ROI (from bottom)\n";

            file.close();
            ROS_INFO("Lane detection parameters saved to %s", color_lane_path.c_str());
        }
        else
        {
            ROS_ERROR("Failed to open file for writing: %s", color_lane_path.c_str());
        }
    }

    void saveTrafficLightConfig()
    {
        std::string traffic_light_path = output_dir_ + "/traffic_light_param.yaml";
        std::ofstream file(traffic_light_path);

        if (file.is_open())
        {
            file << "traffic_light:\n";
            file << "  # Camera topic\n";
            file << "  camera_topic: \"" << tl_cw_camera_topic_ << "\"\n\n";

            file << "  # Timing parameters\n";
            file << "  green_confirmation_time: 1.0 # Time in seconds to confirm green light\n\n";

            file << "  # Detection thresholds\n";
            file << "  red_threshold: 75 # Minimum number of red pixels to detect a red light\n";
            file << "  green_threshold: 75 # Minimum number of green pixels to detect a green light\n\n";

            file << "  # ROI parameters\n";
            file << "  roi_x: " << config_.tl_roi_x << " # X position of ROI\n";
            file << "  roi_y: " << config_.tl_roi_y << " # Y position of ROI\n";
            file << "  roi_width: " << config_.tl_roi_width << " # Width of ROI\n";
            file << "  roi_height: " << config_.tl_roi_height << " # Height of ROI\n\n";

            file << "  # Color thresholds for red (two ranges because red wraps in HSV)\n";
            file << "  hue_red_low1: " << config_.tl_red_h_min1 << "\n";
            file << "  hue_red_high1: " << config_.tl_red_h_max1 << "\n";
            file << "  hue_red_low2: " << config_.tl_red_h_min2 << "\n";
            file << "  hue_red_high2: " << config_.tl_red_h_max2 << "\n";
            file << "  saturation_red_low: " << config_.tl_red_s_min << "\n";
            file << "  saturation_red_high: " << config_.tl_red_s_max << "\n";
            file << "  value_red_low: " << config_.tl_red_v_min << "\n";
            file << "  value_red_high: " << config_.tl_red_v_max << "\n\n";

            file << "  # Color thresholds for green\n";
            file << "  hue_green_low: " << config_.tl_green_h_min << "\n";
            file << "  hue_green_high: " << config_.tl_green_h_max << "\n";
            file << "  saturation_green_low: " << config_.tl_green_s_min << "\n";
            file << "  saturation_green_high: " << config_.tl_green_s_max << "\n";
            file << "  value_green_low: " << config_.tl_green_v_min << "\n";
            file << "  value_green_high: " << config_.tl_green_v_max << "\n";

            file.close();
            ROS_INFO("Traffic light parameters saved to %s", traffic_light_path.c_str());
        }
        else
        {
            ROS_ERROR("Failed to open file for writing: %s", traffic_light_path.c_str());
        }
    }

    void saveCrossWalkConfig()
    {
        std::string cross_walk_path = output_dir_ + "/cross_param.yaml";
        std::ofstream file(cross_walk_path);

        if (file.is_open())
        {
            file << "cross_walking:\n";
            file << "  # Camera topic\n";
            file << "  camera_topic: \"" << tl_cw_camera_topic_ << "\"\n\n";

            file << "  # Movement parameters\n";
            file << "  forward_speed: 0.08 # Linear speed for forward movement (m/s)\n\n";

            file << "  detection_method: \"red_only\"\n\n";

            file << "  # Detection thresholds\n";
            file << "  detection_threshold: 900 # Pixel count threshold to consider crossing detected\n";
            file << "  cleared_threshold: 50 # Pixel count threshold to consider crossing cleared\n\n";

            file << "  # ROI parameters\n";
            file << "  roi_x: " << config_.cw_roi_x << " # X position of ROI\n";
            file << "  roi_y: " << config_.cw_roi_y << " # Y position of ROI\n";
            file << "  roi_width: " << config_.cw_roi_width << " # Width of ROI\n";
            file << "  roi_height: " << config_.cw_roi_height << " # Height of ROI\n\n";

            file << "  # Color thresholds for red (two ranges because red wraps in HSV)\n";
            file << "  hue_red_low1: " << config_.cw_red_h_min1 << "\n";
            file << "  hue_red_high1: " << config_.cw_red_h_max1 << "\n";
            file << "  saturation_red_low: " << config_.cw_red_s_min << "\n";
            file << "  saturation_red_high: " << config_.cw_red_s_max << "\n";
            file << "  value_red_low: " << config_.cw_red_v_min << "\n";
            file << "  value_red_high: " << config_.cw_red_v_max << "\n\n";

            file << "  hue_red_low2: " << config_.cw_red_h_min2 << "\n";
            file << "  hue_red_high2: " << config_.cw_red_h_max2 << "\n\n";

            file << "  # Color thresholds for white\n";
            file << "  hue_white_low: 0\n";
            file << "  hue_white_high: 180\n";
            file << "  saturation_white_low: 0\n";
            file << "  saturation_white_high: 30\n";
            file << "  value_white_low: 200\n";
            file << "  value_white_high: 255\n";

            file.close();
            ROS_INFO("Cross walk parameters saved to %s", cross_walk_path.c_str());
        }
        else
        {
            ROS_ERROR("Failed to open file for writing: %s", cross_walk_path.c_str());
        }
    }

    void setMode(const std::string &mode)
    {
        if (mode == "lane" || mode == "traffic_light" || mode == "cross_walk")
        {
            current_mode_ = mode;
            ROS_INFO("Switched to mode: %s", current_mode_.c_str());

            // Create initial visualizations based on new mode
            createInitialVisualizations();
        }
        else
        {
            ROS_WARN("Invalid mode: %s. Using current mode: %s", mode.c_str(), current_mode_.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hsv_reconfigure_tuner");

    ros::NodeHandle private_nh("~");
    std::string mode;
    private_nh.param<std::string>("mode", mode, "lane");

    HSVReconfigureTuner tuner;
    tuner.setMode(mode);

    ros::spin();
    return 0;
}