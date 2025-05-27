// Put crosswalk_detector_node.cpp in src/crosswalk_detector/src/#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

class CrosswalkDetectorNode
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // Subscribers
    image_transport::Subscriber image_sub_;

    // Publishers
    ros::Publisher pixel_count_pub_;
    image_transport::Publisher mask_image_pub_;

    // Parameters from YAML
    std::string camera_topic_;
    std::string detection_method_;
    int crosswalk_roi_x_, crosswalk_roi_y_;
    int crosswalk_roi_width_, crosswalk_roi_height_;

    // Color thresholds for red and white crosswalk detection
    int hue_red_low1_, hue_red_high1_;
    int hue_red_low2_, hue_red_high2_;
    int saturation_red_low_, saturation_red_high_;
    int value_red_low_, value_red_high_;
    int hue_white_low_, hue_white_high_;
    int saturation_white_low_, saturation_white_high_;
    int value_white_low_, value_white_high_;

    // Image processing
    cv::Mat img_bgr_;
    cv::Mat img_hsv_;
    bool image_received_;

public:
    CrosswalkDetectorNode() : it_(nh_), image_received_(false)
    {
        // Load parameters from YAML
        loadParameters();

        // Publishers
        pixel_count_pub_ = nh_.advertise<std_msgs::Int32>("/crosswalk/pixel_count", 10);
        mask_image_pub_ = it_.advertise("/crosswalk/mask_image", 1);

        // Subscriber
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &CrosswalkDetectorNode::imageCallback, this, image_transport::TransportHints("compressed"));

        ROS_INFO("Crosswalk Detector Node Started");
        ROS_INFO("Publishing pixel count to: /crosswalk/pixel_count");
        ROS_INFO("Publishing mask image to: /crosswalk/mask_image");
    }

    void loadParameters()
    {
        try
        {
            // Look for config in action_bt package since that's where cross_param.yaml is located
            std::string action_bt_path = ros::package::getPath("action_bt");
            std::string config_path = action_bt_path + "/../config/cross_param.yaml";
            YAML::Node config = YAML::LoadFile(config_path);

            // Camera and detection parameters
            camera_topic_ = config["cross_walking"]["camera_topic_front"] ? config["cross_walking"]["camera_topic_front"].as<std::string>() : "/camera/image";
            detection_method_ = config["cross_walking"]["detection_method"] ? config["cross_walking"]["detection_method"].as<std::string>() : "red_only";

            // ROI parameters
            crosswalk_roi_x_ = config["cross_walking"]["crosswalk_roi_x"] ? config["cross_walking"]["crosswalk_roi_x"].as<int>() : 4;
            crosswalk_roi_y_ = config["cross_walking"]["crosswalk_roi_y"] ? config["cross_walking"]["crosswalk_roi_y"].as<int>() : 90;
            crosswalk_roi_width_ = config["cross_walking"]["crosswalk_roi_width"] ? config["cross_walking"]["crosswalk_roi_width"].as<int>() : 280;
            crosswalk_roi_height_ = config["cross_walking"]["crosswalk_roi_height"] ? config["cross_walking"]["crosswalk_roi_height"].as<int>() : 75;

            // Color thresholds
            hue_red_low1_ = config["cross_walking"]["hue_red_low1"] ? config["cross_walking"]["hue_red_low1"].as<int>() : 0;
            hue_red_high1_ = config["cross_walking"]["hue_red_high1"] ? config["cross_walking"]["hue_red_high1"].as<int>() : 15;
            hue_red_low2_ = config["cross_walking"]["hue_red_low2"] ? config["cross_walking"]["hue_red_low2"].as<int>() : 170;
            hue_red_high2_ = config["cross_walking"]["hue_red_high2"] ? config["cross_walking"]["hue_red_high2"].as<int>() : 180;
            saturation_red_low_ = config["cross_walking"]["saturation_red_low"] ? config["cross_walking"]["saturation_red_low"].as<int>() : 9;
            saturation_red_high_ = config["cross_walking"]["saturation_red_high"] ? config["cross_walking"]["saturation_red_high"].as<int>() : 255;
            value_red_low_ = config["cross_walking"]["value_red_low"] ? config["cross_walking"]["value_red_low"].as<int>() : 19;
            value_red_high_ = config["cross_walking"]["value_red_high"] ? config["cross_walking"]["value_red_high"].as<int>() : 255;

            hue_white_low_ = config["cross_walking"]["hue_white_low"] ? config["cross_walking"]["hue_white_low"].as<int>() : 0;
            hue_white_high_ = config["cross_walking"]["hue_white_high"] ? config["cross_walking"]["hue_white_high"].as<int>() : 180;
            saturation_white_low_ = config["cross_walking"]["saturation_white_low"] ? config["cross_walking"]["saturation_white_low"].as<int>() : 0;
            saturation_white_high_ = config["cross_walking"]["saturation_white_high"] ? config["cross_walking"]["saturation_white_high"].as<int>() : 30;
            value_white_low_ = config["cross_walking"]["value_white_low"] ? config["cross_walking"]["value_white_low"].as<int>() : 200;
            value_white_high_ = config["cross_walking"]["value_white_high"] ? config["cross_walking"]["value_white_high"].as<int>() : 255;

            ROS_INFO("Loaded crosswalk detection parameters");
            ROS_INFO("  Camera: %s", camera_topic_.c_str());
            ROS_INFO("  Method: %s", detection_method_.c_str());
            ROS_INFO("  ROI: [%d,%d,%d,%d]", crosswalk_roi_x_, crosswalk_roi_y_, crosswalk_roi_width_, crosswalk_roi_height_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading YAML: %s", e.what());
            ROS_WARN("Using default parameters");

            // Set defaults
            camera_topic_ = "/camera/image";
            detection_method_ = "red_only";
            crosswalk_roi_x_ = 4;
            crosswalk_roi_y_ = 90;
            crosswalk_roi_width_ = 280;
            crosswalk_roi_height_ = 75;

            // Default color thresholds
            hue_red_low1_ = 0;
            hue_red_high1_ = 15;
            hue_red_low2_ = 170;
            hue_red_high2_ = 180;
            saturation_red_low_ = 9;
            saturation_red_high_ = 255;
            value_red_low_ = 19;
            value_red_high_ = 255;
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
        try
        {
            img_bgr_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::cvtColor(img_bgr_, img_hsv_, cv::COLOR_BGR2HSV);
            image_received_ = true;

            // Process the image
            processImage();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void processImage()
    {
        if (!image_received_)
            return;

        // Define ROI
        cv::Rect crosswalk_roi(crosswalk_roi_x_, crosswalk_roi_y_, crosswalk_roi_width_, crosswalk_roi_height_);

        // Create masks for red color (two ranges since red wraps around in HSV)
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
            final_mask = red_mask.clone();
        }
        else
        {
            // Combined red and white masks
            cv::Mat white_mask;
            cv::inRange(img_hsv_,
                        cv::Scalar(hue_white_low_, saturation_white_low_, value_white_low_),
                        cv::Scalar(hue_white_high_, saturation_white_high_, value_white_high_),
                        white_mask);
            cv::bitwise_or(red_mask, white_mask, final_mask);
        }

        // Apply ROI to the mask
        cv::Mat roi_mask = final_mask(crosswalk_roi);
        int pixel_count = cv::countNonZero(roi_mask);

        // Create visualization mask (full image size for rqt)
        cv::Mat visualization_mask = cv::Mat::zeros(img_bgr_.rows, img_bgr_.cols, CV_8UC1);
        final_mask.copyTo(visualization_mask);

        // Draw ROI rectangle on visualization
        cv::rectangle(visualization_mask, crosswalk_roi, cv::Scalar(128), 2);

        // Publish pixel count
        std_msgs::Int32 count_msg;
        count_msg.data = pixel_count;
        pixel_count_pub_.publish(count_msg);

        // Publish mask image for rqt visualization
        sensor_msgs::ImagePtr mask_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", visualization_mask).toImageMsg();
        mask_msg->header.stamp = ros::Time::now();
        mask_msg->header.frame_id = "camera_frame";
        mask_image_pub_.publish(mask_msg);

        // Log info periodically
        ROS_INFO_THROTTLE(1.0, "Crosswalk detection: %d pixels", pixel_count);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crosswalk_detector_node");
    CrosswalkDetectorNode detector;
    ros::spin();
    return 0;
}