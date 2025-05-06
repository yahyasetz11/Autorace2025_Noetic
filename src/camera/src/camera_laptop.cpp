#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "camera_laptop");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Set up parameters with default values
    int camera_id;
    int frame_width;
    int frame_height;
    int fps;

    private_nh.param("camera_id", camera_id, 0);         // Default camera (usually built-in webcam)
    private_nh.param("frame_width", frame_width, 320);   // Default width
    private_nh.param("frame_height", frame_height, 240); // Default height
    private_nh.param("fps", fps, 30);                    // Default FPS

    // Create image transport and publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera_laptop", 1);

    // Set up OpenCV capture
    cv::VideoCapture cap(camera_id);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera with ID %d", camera_id);
        return -1;
    }

    // Try to set camera properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height);
    cap.set(cv::CAP_PROP_FPS, fps);

    // Get actual camera properties (may be different from requested)
    int actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    int actual_fps = cap.get(cv::CAP_PROP_FPS);

    ROS_INFO("Camera initialized: %dx%d at %d FPS", actual_width, actual_height, actual_fps);

    // Set up loop rate
    ros::Rate loop_rate(fps);

    // Main loop
    while (ros::ok())
    {
        cv::Mat frame;

        // Capture frame
        if (!cap.read(frame))
        {
            ROS_ERROR("Failed to capture frame");
            continue;
        }

        // Convert to ROS image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = ros::Time::now();

        // Publish image
        pub.publish(msg);

        // Process callbacks and sleep
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Release camera
    cap.release();

    return 0;
}