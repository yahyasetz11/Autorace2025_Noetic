#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

class ImageProjector
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Homography parameters
    int top_x_, top_y_, bottom_x_, bottom_y_;

    // Latest image
    cv::Mat latest_image_;
    bool image_received_;

public:
    ImageProjector() : private_nh_("~"), it_(nh_), image_received_(false)
    {
        // Load parameters with defaults from the provided code
        private_nh_.param("top_x", top_x_, 72);
        private_nh_.param("top_y", top_y_, 4);
        private_nh_.param("bottom_x", bottom_x_, 115);
        private_nh_.param("bottom_y", bottom_y_, 120);

        // Get camera topic from parameter
        std::string camera_topic;
        private_nh_.param<std::string>("camera_topic", camera_topic, "/camera/rgb/image_raw");

        // Set up subscriber and publisher
        image_sub_ = it_.subscribe(camera_topic, 1, &ImageProjector::imageCallback, this, image_transport::TransportHints("compressed"));
        image_pub_ = it_.advertise("/camera/image_projected_compensated", 1);

        ROS_INFO("ImageProjector initialized with parameters:");
        ROS_INFO("  top_x: %d, top_y: %d", top_x_, top_y_);
        ROS_INFO("  bottom_x: %d, bottom_y: %d", bottom_x_, bottom_y_);
        ROS_INFO("  camera_topic: %s", camera_topic.c_str());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            latest_image_ = cv_ptr->image;
            image_received_ = true;

            // Process and publish immediately
            processAndPublish(cv_ptr->header.stamp);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void processAndPublish(const ros::Time &stamp)
    {
        if (!image_received_)
            return;

        cv::Mat image_original = latest_image_.clone();

        // Apply Gaussian blur
        cv::GaussianBlur(image_original, image_original, cv::Size(5, 5), 0);

        // Get image dimensions
        int height = image_original.rows;
        int width = image_original.cols;
        int center_x = width / 2;

        // Setting homography variables
        int half_x = width / 2;
        int half_y = height / 2;
        double ratio_y = (216.0 / 288.0) * height;

        // Calculate source points for homography
        std::vector<cv::Point2f> src_points = {
            cv::Point2f(center_x - top_x_, 216 - top_y_),
            cv::Point2f(center_x + top_x_, 216 - top_y_),
            cv::Point2f(center_x + bottom_x_, 176 + bottom_y_),
            cv::Point2f(center_x - bottom_x_, 176 + bottom_y_)};

        // Calculate destination points
        std::vector<cv::Point2f> dst_points = {
            cv::Point2f(100, 0),
            cv::Point2f(400, 0),
            cv::Point2f(400, 300),
            cv::Point2f(100, 300)};

        // Calculate homography matrix
        cv::Mat h = cv::findHomography(src_points, dst_points);

        // Apply homography transformation
        cv::Mat image_projected;
        cv::warpPerspective(image_original, image_projected, h, cv::Size(500, 300));

        // Fill the empty space with black triangles
        std::vector<cv::Point> triangle1 = {
            cv::Point(0, 299),
            cv::Point(0, 120),
            cv::Point(100, 299)};

        std::vector<cv::Point> triangle2 = {
            cv::Point(499, 299),
            cv::Point(499, 120),
            cv::Point(399, 299)};

        cv::fillPoly(image_projected, std::vector<std::vector<cv::Point>>{triangle1, triangle2}, cv::Scalar(0, 0, 0));

        // Convert back to ROS message and publish
        sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_projected).toImageMsg();
        output_msg->header.stamp = stamp;
        image_pub_.publish(output_msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_projector");
    ImageProjector projector;

    // Use a higher rate than the camera to ensure we process all frames
    ros::Rate rate(24);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}