#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <boost/filesystem.hpp>

class VideoRecorder
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber image_sub_;
    ros::Timer timer_; // Timer to periodically check shutdown

    // Parameters
    std::string camera_topic_;
    std::string output_dir_;
    std::string filename_prefix_;
    int fps_;
    double max_duration_; // in seconds

    // OpenCV video writer
    cv::VideoWriter video_writer_;

    // State variables
    bool recording_;
    ros::Time start_time_;
    int frame_count_;
    std::string current_file_path_;

    // Video dimensions
    int width_;
    int height_;

public:
    VideoRecorder() : private_nh_("~"),
                      recording_(false),
                      frame_count_(0),
                      width_(640),
                      height_(480)
    {
        // Load parameters from the parameter server
        private_nh_.param<std::string>("camera_topic", camera_topic_, "/camera/image/compressed");
        private_nh_.param<std::string>("output_dir", output_dir_, "~/videos");
        private_nh_.param<std::string>("filename_prefix", filename_prefix_, "ros_video");
        private_nh_.param<int>("fps", fps_, 30);
        private_nh_.param<double>("max_duration", max_duration_, 300.0); // 5 minutes default

        // Expand the output directory path if it contains ~ for home directory
        if (output_dir_[0] == '~')
        {
            const char *home_dir = getenv("HOME");
            if (home_dir)
            {
                output_dir_.replace(0, 1, home_dir);
            }
        }

        // Ensure output directory exists
        boost::filesystem::path dir_path(output_dir_);
        try
        {
            if (!boost::filesystem::exists(dir_path))
            {
                boost::filesystem::create_directories(dir_path);
                ROS_INFO("Created output directory: %s", output_dir_.c_str());
            }
        }
        catch (const boost::filesystem::filesystem_error &e)
        {
            ROS_ERROR("Failed to create output directory: %s", e.what());
            return;
        }

        // Create subscriber for compressed image topic
        image_sub_ = nh_.subscribe(camera_topic_, 10, &VideoRecorder::imageCallback, this);

        // Create a timer that checks if we should shutdown
        timer_ = nh_.createTimer(ros::Duration(0.5), &VideoRecorder::timerCallback, this);

        ROS_INFO("Video recorder initialized");
        ROS_INFO("Listening to topic: %s", camera_topic_.c_str());
        ROS_INFO("Recording to directory: %s", output_dir_.c_str());
        ROS_INFO("Frame rate: %d fps", fps_);
        ROS_INFO("Maximum duration per video: %.1f seconds", max_duration_);
        ROS_INFO("Press Ctrl+C to stop recording and save the video");
    }

    ~VideoRecorder()
    {
        stopRecording();
        ROS_INFO("Video recorder shut down");
    }

    // Timer callback to check for shutdown
    void timerCallback(const ros::TimerEvent &event)
    {
        // Check if ROS is still ok
        if (!ros::ok() && recording_)
        {
            ROS_INFO("ROS shutdown detected, stopping recording...");
            stopRecording();
        }
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        try
        {
            // Skip if ROS is shutting down
            if (!ros::ok())
            {
                return;
            }

            // Convert compressed image to OpenCV format
            cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

            // Initialize video writer if not recording
            if (!recording_)
            {
                startNewRecording(image.cols, image.rows);
            }

            // Check if we need to start a new recording due to time limit
            if (recording_ && (ros::Time::now() - start_time_).toSec() > max_duration_)
            {
                ROS_INFO("Maximum recording duration reached, starting new video file");
                stopRecording();
                startNewRecording(image.cols, image.rows);
            }

            // Write frame to video
            if (recording_ && video_writer_.isOpened())
            {
                video_writer_.write(image);
                frame_count_++;

                // Log progress occasionally
                if (frame_count_ % 30 == 0)
                {
                    double elapsed = (ros::Time::now() - start_time_).toSec();
                    ROS_INFO("Recording in progress: %.1f seconds, %d frames (%.2f fps)",
                             elapsed, frame_count_, frame_count_ / elapsed);
                }
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
        catch (std::exception &e)
        {
            ROS_ERROR("Exception in image callback: %s", e.what());
        }
    }

    void startNewRecording(int width, int height)
    {
        // Generate timestamped filename
        std::time_t now = std::time(nullptr);
        char timestamp[20];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));

        std::string filename = filename_prefix_ + "_" + timestamp + ".avi";
        current_file_path_ = output_dir_ + "/" + filename;

        // Store dimensions
        width_ = width;
        height_ = height;

        // Initialize video writer
        video_writer_.open(current_file_path_,
                           cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                           fps_,
                           cv::Size(width_, height_));

        if (!video_writer_.isOpened())
        {
            ROS_ERROR("Failed to open video file for writing: %s", current_file_path_.c_str());
            return;
        }

        // Reset recording state
        recording_ = true;
        start_time_ = ros::Time::now();
        frame_count_ = 0;

        ROS_INFO("Started recording to: %s", current_file_path_.c_str());
        ROS_INFO("Video dimensions: %dx%d", width_, height_);
    }

    void stopRecording()
    {
        if (recording_ && video_writer_.isOpened())
        {
            // Make sure to flush buffers and properly close the file
            video_writer_.release();

            if (frame_count_ > 0)
            {
                double duration = (ros::Time::now() - start_time_).toSec();
                ROS_INFO("Finished recording: %s", current_file_path_.c_str());
                ROS_INFO("Recorded %d frames in %.1f seconds (%.2f fps)",
                         frame_count_, duration, frame_count_ / duration);
                ROS_INFO("Video saved successfully!");
            }
            recording_ = false;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_recorder");

    ROS_INFO("Starting video recorder...");

    {
        // Create the recorder in a block so destructor is called before ros::shutdown()
        VideoRecorder recorder;

        // Keep the node alive until shutdown
        ros::spin();

        // stopRecording will be called in the destructor
    }

    ROS_INFO("Video recorder shut down properly");
    return 0;
}