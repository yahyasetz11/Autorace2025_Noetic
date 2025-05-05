#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

class HSVTuner
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // OpenCV windows
    const std::string WINDOW_ORIGINAL = "Original Image";
    const std::string WINDOW_MASKED = "Masked Image";
    const std::string WINDOW_CONTROLS = "HSV Controls";

    // HSV values
    int h_min, h_max;
    int s_min, s_max;
    int v_min, v_max;

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
        nh_.param<std::string>("output_file", output_file, "hsv_params.yaml");

        // Get initial HSV values
        nh_.param("h_min", h_min, 0);
        nh_.param("h_max", h_max, 179);
        nh_.param("s_min", s_min, 0);
        nh_.param("s_max", s_max, 255);
        nh_.param("v_min", v_min, 0);
        nh_.param("v_max", v_max, 255);

        // Subscribe to the camera topic
        image_sub_ = it_.subscribe(camera_topic, 1, &HSVTuner::imageCallback, this);

        // Create OpenCV windows
        cv::namedWindow(WINDOW_ORIGINAL, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_MASKED, cv::WINDOW_NORMAL);
        cv::namedWindow(WINDOW_CONTROLS, cv::WINDOW_NORMAL);

        // Create trackbars
        cv::createTrackbar("H min", WINDOW_CONTROLS, &h_min, 179);
        cv::createTrackbar("H max", WINDOW_CONTROLS, &h_max, 179);
        cv::createTrackbar("S min", WINDOW_CONTROLS, &s_min, 255);
        cv::createTrackbar("S max", WINDOW_CONTROLS, &s_max, 255);
        cv::createTrackbar("V min", WINDOW_CONTROLS, &v_min, 255);
        cv::createTrackbar("V max", WINDOW_CONTROLS, &v_max, 255);

        // Create save button using a dummy trackbar
        int save_button = 0;
        cv::createTrackbar("Save Parameters", WINDOW_CONTROLS, &save_button, 1, onSaveTrackbar, this);

        ROS_INFO("HSV Tuner initialized. Subscribed to %s", camera_topic.c_str());
        ROS_INFO("Initial HSV values: H=[%d,%d], S=[%d,%d], V=[%d,%d]", h_min, h_max, s_min, s_max, v_min, v_max);
        ROS_INFO("Press 's' to save parameters to %s", output_file.c_str());
    }

    ~HSVTuner()
    {
        cv::destroyAllWindows();
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

        // Create mask using HSV thresholds
        cv::Mat mask;
        cv::Scalar lower_bound(h_min, s_min, v_min);
        cv::Scalar upper_bound(h_max, s_max, v_max);
        cv::inRange(hsv_image, lower_bound, upper_bound, mask);

        // Create two different result images:
        // 1. Binary mask (true black and white)
        cv::Mat binary_result = mask.clone();

        // 2. Masked original (for reference)
        cv::Mat color_result;
        cv::bitwise_and(current_image, current_image, color_result, mask);

        // Show images
        cv::imshow(WINDOW_ORIGINAL, current_image);
        cv::imshow(WINDOW_MASKED, binary_result); // Show binary mask instead

        // Add a third window for the color-masked result
        cv::imshow("Color Masked", color_result);

        // Display current values
        cv::Mat control_display = cv::Mat::zeros(100, 400, CV_8UC3);
        std::string hsv_text = "H: [" + std::to_string(h_min) + "," + std::to_string(h_max) + "] ";
        hsv_text += "S: [" + std::to_string(s_min) + "," + std::to_string(s_max) + "] ";
        hsv_text += "V: [" + std::to_string(v_min) + "," + std::to_string(v_max) + "]";
        cv::putText(control_display, hsv_text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        cv::imshow(WINDOW_CONTROLS, control_display);

        // Check for save request
        if (save_requested)
        {
            saveParameters();
            save_requested = false;
            // Reset the trackbar
            int value = 0;
            cv::setTrackbarPos("Save Parameters", WINDOW_CONTROLS, value);
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
            file << "hsv:\n";
            file << "  white:\n";
            file << "    h_min: " << h_min << "\n";
            file << "    h_max: " << h_max << "\n";
            file << "    s_min: " << s_min << "\n";
            file << "    s_max: " << s_max << "\n";
            file << "    v_min: " << v_min << "\n";
            file << "    v_max: " << v_max << "\n";
            file.close();
            ROS_INFO("Parameters saved successfully!");

            // Also print parameters in C++ format for easy copying
            ROS_INFO("C++ code for these parameters:");
            ROS_INFO("int hue_min = %d, hue_max = %d;", h_min, h_max);
            ROS_INFO("int saturation_min = %d, saturation_max = %d;", s_min, s_max);
            ROS_INFO("int value_min = %d, value_max = %d;", v_min, v_max);
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