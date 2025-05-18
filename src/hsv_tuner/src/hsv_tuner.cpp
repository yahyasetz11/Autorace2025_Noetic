#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

class HSVTuner
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // OpenCV windows
    const std::string WINDOW_ORIGINAL = "Original Image";
    const std::string WINDOW_WHITE_MASK = "White Lane Mask";
    const std::string WINDOW_YELLOW_MASK = "Yellow Lane Mask";
    const std::string WINDOW_WHITE_CONTROLS = "White HSV Controls";
    const std::string WINDOW_YELLOW_CONTROLS = "Yellow HSV Controls";

    // HSV values for white lane
    int h_white_min, h_white_max;
    int s_white_min, s_white_max;
    int v_white_min, v_white_max;

    // HSV values for yellow lane
    int h_yellow_min, h_yellow_max;
    int s_yellow_min, s_yellow_max;
    int v_yellow_min, v_yellow_max;

    // Output file
    std::string output_file;
    bool save_requested;

    // Latest image
    cv::Mat current_image;
    bool image_received;

public:
    HSVTuner() : it_(nh_), image_received(false), save_requested(false)
    {
        // Get parameters
        std::string camera_topic;
        nh_.param<std::string>("camera_topic", camera_topic, "/camera/image_projected_compensated");
        // nh_.param<std::string>("camera_topic", camera_topic, "/camera/image");
        nh_.param<std::string>("output_file", output_file, "hsv_params.yaml");

        // Set default HSV values before loading from file
        h_white_min = 0;
        h_white_max = 179;
        s_white_min = 0;
        s_white_max = 70;
        v_white_min = 165;
        v_white_max = 255;

        h_yellow_min = 10;
        h_yellow_max = 50;
        s_yellow_min = 100;
        s_yellow_max = 255;
        v_yellow_min = 100;
        v_yellow_max = 255;

        // Load color configuration from YAML
        std::string color_config_path = ros::package::getPath("hsv_tuner") + "/../config/color_lane.yaml";
        ROS_INFO("Loading color config from: %s", color_config_path.c_str());
        loadColorConfig(color_config_path);

        // Subscribe to the camera topic
        image_sub_ = it_.subscribe(camera_topic, 1, &HSVTuner::imageCallback, this);

        // Create OpenCV windows
        cv::namedWindow(WINDOW_ORIGINAL, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_WHITE_MASK, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_YELLOW_MASK, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_WHITE_CONTROLS, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_YELLOW_CONTROLS, cv::WINDOW_NORMAL);

        // Create trackbars for white controls
        cv::createTrackbar("H min", WINDOW_WHITE_CONTROLS, &h_white_min, 179);
        cv::createTrackbar("H max", WINDOW_WHITE_CONTROLS, &h_white_max, 179);
        cv::createTrackbar("S min", WINDOW_WHITE_CONTROLS, &s_white_min, 255);
        cv::createTrackbar("S max", WINDOW_WHITE_CONTROLS, &s_white_max, 255);
        cv::createTrackbar("V min", WINDOW_WHITE_CONTROLS, &v_white_min, 255);
        cv::createTrackbar("V max", WINDOW_WHITE_CONTROLS, &v_white_max, 255);

        // Create save button for white controls
        int save_white_button = 0;
        cv::createTrackbar("Save White Params", WINDOW_WHITE_CONTROLS, &save_white_button, 1, onSaveTrackbar, this);

        // Create trackbars for yellow controls
        cv::createTrackbar("H min", WINDOW_YELLOW_CONTROLS, &h_yellow_min, 179);
        cv::createTrackbar("H max", WINDOW_YELLOW_CONTROLS, &h_yellow_max, 179);
        cv::createTrackbar("S min", WINDOW_YELLOW_CONTROLS, &s_yellow_min, 255);
        cv::createTrackbar("S max", WINDOW_YELLOW_CONTROLS, &s_yellow_max, 255);
        cv::createTrackbar("V min", WINDOW_YELLOW_CONTROLS, &v_yellow_min, 255);
        cv::createTrackbar("V max", WINDOW_YELLOW_CONTROLS, &v_yellow_max, 255);

        // Create save button for yellow controls
        int save_yellow_button = 0;
        cv::createTrackbar("Save Yellow Params", WINDOW_YELLOW_CONTROLS, &save_yellow_button, 1, onSaveTrackbar, this);

        ROS_INFO("HSV Tuner initialized. Subscribed to %s", camera_topic.c_str());
        ROS_INFO("White HSV values: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 h_white_min, h_white_max, s_white_min, s_white_max, v_white_min, v_white_max);
        ROS_INFO("Yellow HSV values: H=[%d,%d], S=[%d,%d], V=[%d,%d]",
                 h_yellow_min, h_yellow_max, s_yellow_min, s_yellow_max, v_yellow_min, v_yellow_max);
        ROS_INFO("Press 's' to save parameters to %s", output_file.c_str());
    }

    ~HSVTuner()
    {
        cv::destroyAllWindows();
    }

    void loadColorConfig(const std::string &config_path)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(config_path);

            if (config["white_lane"])
            {
                auto white = config["white_lane"];
                h_white_min = white["hue"]["low"].as<int>();
                h_white_max = white["hue"]["high"].as<int>();
                s_white_min = white["saturation"]["low"].as<int>();
                s_white_max = white["saturation"]["high"].as<int>();
                v_white_min = white["value"]["low"].as<int>();
                v_white_max = white["value"]["high"].as<int>();
                ROS_INFO("Loaded white lane config successfully");
            }

            if (config["yellow_lane"])
            {
                auto yellow = config["yellow_lane"];
                h_yellow_min = yellow["hue"]["low"].as<int>();
                h_yellow_max = yellow["hue"]["high"].as<int>();
                s_yellow_min = yellow["saturation"]["low"].as<int>();
                s_yellow_max = yellow["saturation"]["high"].as<int>();
                v_yellow_min = yellow["value"]["low"].as<int>();
                v_yellow_max = yellow["value"]["high"].as<int>();
                ROS_INFO("Loaded yellow lane config successfully");
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Failed to load color config: %s", e.what());
            ROS_WARN("Using default HSV values instead");
        }
    }

    // Static callback for the save trackbar
    static void onSaveTrackbar(int value, void *userdata)
    {
        if (value == 1)
        {
            HSVTuner *tuner = reinterpret_cast<HSVTuner *>(userdata);
            tuner->save_requested = true;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            // Convert ROS image to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            current_image = cv_ptr->image.clone();
            image_received = true;

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
        if (!image_received)
            return;

        // Convert BGR to HSV
        cv::Mat hsv_image;
        cv::cvtColor(current_image, hsv_image, cv::COLOR_BGR2HSV);

        // Create mask for white lane
        cv::Mat white_mask;
        cv::Scalar white_lower(h_white_min, s_white_min, v_white_min);
        cv::Scalar white_upper(h_white_max, s_white_max, v_white_max);
        cv::inRange(hsv_image, white_lower, white_upper, white_mask);

        // Create mask for yellow lane
        cv::Mat yellow_mask;
        cv::Scalar yellow_lower(h_yellow_min, s_yellow_min, v_yellow_min);
        cv::Scalar yellow_upper(h_yellow_max, s_yellow_max, v_yellow_max);
        cv::inRange(hsv_image, yellow_lower, yellow_upper, yellow_mask);

        // Show images
        cv::imshow(WINDOW_ORIGINAL, current_image);
        cv::imshow(WINDOW_WHITE_MASK, white_mask);
        cv::imshow(WINDOW_YELLOW_MASK, yellow_mask);

        // Display current white values
        cv::Mat white_control_display = cv::Mat::zeros(100, 400, CV_8UC3);
        std::string white_hsv_text = "H: [" + std::to_string(h_white_min) + "," + std::to_string(h_white_max) + "] ";
        white_hsv_text += "S: [" + std::to_string(s_white_min) + "," + std::to_string(s_white_max) + "] ";
        white_hsv_text += "V: [" + std::to_string(v_white_min) + "," + std::to_string(v_white_max) + "]";
        cv::putText(white_control_display, white_hsv_text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::imshow(WINDOW_WHITE_CONTROLS, white_control_display);

        // Display current yellow values
        cv::Mat yellow_control_display = cv::Mat::zeros(100, 400, CV_8UC3);
        std::string yellow_hsv_text = "H: [" + std::to_string(h_yellow_min) + "," + std::to_string(h_yellow_max) + "] ";
        yellow_hsv_text += "S: [" + std::to_string(s_yellow_min) + "," + std::to_string(s_yellow_max) + "] ";
        yellow_hsv_text += "V: [" + std::to_string(v_yellow_min) + "," + std::to_string(v_yellow_max) + "]";
        cv::putText(yellow_control_display, yellow_hsv_text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::imshow(WINDOW_YELLOW_CONTROLS, yellow_control_display);

        // Check for save request
        if (save_requested)
        {
            saveParameters();
            save_requested = false;
            // Reset the trackbars
            cv::setTrackbarPos("Save White Params", WINDOW_WHITE_CONTROLS, 0);
            cv::setTrackbarPos("Save Yellow Params", WINDOW_YELLOW_CONTROLS, 0);
        }

        // Check for 's' key press
        int key = cv::waitKey(1);
        if (key == 's' || key == 'S')
        {
            saveParameters();
        }
    }

    void saveParameters()
    {
        ROS_INFO("Saving HSV parameters to %s", output_file.c_str());

        // Make sure directory exists
        std::string directory = output_file.substr(0, output_file.find_last_of("/\\"));
        std::string command = "mkdir -p " + directory;
        system(command.c_str());

        // Save parameters to YAML file
        std::ofstream file(output_file);
        if (file.is_open())
        {
            file << "# HSV Parameters saved by hsv_tuner node\n";

            // White lane parameters
            file << "white_lane:\n";
            file << "  hue:\n";
            file << "    low: " << h_white_min << "\n";
            file << "    high: " << h_white_max << "\n";
            file << "  saturation:\n";
            file << "    low: " << s_white_min << "\n";
            file << "    high: " << s_white_max << "\n";
            file << "  value:\n";
            file << "    low: " << v_white_min << "\n";
            file << "    high: " << v_white_max << "\n";

            // Yellow lane parameters
            file << "yellow_lane:\n";
            file << "  hue:\n";
            file << "    low: " << h_yellow_min << "\n";
            file << "    high: " << h_yellow_max << "\n";
            file << "  saturation:\n";
            file << "    low: " << s_yellow_min << "\n";
            file << "    high: " << s_yellow_max << "\n";
            file << "  value:\n";
            file << "    low: " << v_yellow_min << "\n";
            file << "    high: " << v_yellow_max << "\n";

            file << "\n# ROI settings (from original file)\n";
            file << "roi:\n";
            file << "  height_factor: 0.5 # Portion of the image height to use for ROI (from bottom)\n";

            file.close();
            ROS_INFO("Parameters saved successfully!");

            // Also print parameters in C++ format for easy copying
            ROS_INFO("C++ code for White lane parameters:");
            ROS_INFO("int hue_white_l = %d, hue_white_h = %d;", h_white_min, h_white_max);
            ROS_INFO("int saturation_white_l = %d, saturation_white_h = %d;", s_white_min, s_white_max);
            ROS_INFO("int value_white_l = %d, value_white_h = %d;", v_white_min, v_white_max);

            ROS_INFO("C++ code for Yellow lane parameters:");
            ROS_INFO("int hue_yellow_l = %d, hue_yellow_h = %d;", h_yellow_min, h_yellow_max);
            ROS_INFO("int saturation_yellow_l = %d, saturation_yellow_h = %d;", s_yellow_min, s_yellow_max);
            ROS_INFO("int value_yellow_l = %d, value_yellow_h = %d;", v_yellow_min, v_yellow_max);
        }
        else
        {
            ROS_ERROR("Failed to open file for writing: %s", output_file.c_str());
        }
    }

    void run()
    {
        ros::Rate loop_rate(30); // 30 Hz

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hsv_tuner_node");
    HSVTuner tuner;
    tuner.run();
    return 0;
}