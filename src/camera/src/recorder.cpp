#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <string>

class VideoRecorder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    cv::VideoWriter video_writer_;
    std::string output_path_;
    bool is_writer_initialized_ = false;
    int fps_ = 30;

public:
    VideoRecorder()
    {
        std::string image_topic;
        nh_.param<std::string>("image_topic", image_topic, "/camera_laptop");

        std::time_t now = std::time(0);
        std::tm *local_time = std::localtime(&now);
        char filename[100];
        std::strftime(filename, sizeof(filename), "recorded_%Y%m%d_%H%M%S.mp4", local_time);
        output_path_ = std::string(std::getenv("HOME")) + "/Documents/Autorace2025_Noetic/video/" + filename;

        image_sub_ = nh_.subscribe(image_topic, 10, &VideoRecorder::imageCallback, this);

        ROS_INFO("Recording from topic: %s", image_topic.c_str());
        ROS_INFO("Saving video to: %s", output_path_.c_str());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (!is_writer_initialized_)
        {
            int width = cv_ptr->image.cols;
            int height = cv_ptr->image.rows;
            video_writer_.open(output_path_, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps_, cv::Size(width, height));
            if (!video_writer_.isOpened())
            {
                ROS_ERROR("Failed to open video writer");
                return;
            }
            is_writer_initialized_ = true;
            ROS_INFO("VideoWriter initialized: %dx%d", width, height);
        }

        video_writer_.write(cv_ptr->image);
    }

    ~VideoRecorder()
    {
        if (video_writer_.isOpened())
        {
            video_writer_.release();
            ROS_INFO("Recording finished.");
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_recorder_cpp");
    VideoRecorder recorder;
    ros::spin();
    return 0;
}
