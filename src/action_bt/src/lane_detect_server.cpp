#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/LaneDetectAction.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cmath> // Untuk fungsi matematika

class LaneDetectServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::LaneDetectAction> as_;
    std::string action_name_;
    msg_file::LaneDetectFeedback feedback_;
    msg_file::LaneDetectResult result_;

    // ROS komunikasi
    ros::Publisher cmd_vel_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // Variabel untuk lane detection
    cv::Mat img_bgr_;
    cv::Mat img_hsv_;
    cv::Mat img_masked_;
    bool image_received_;
    ros::Time start_time_;
    std::string camera_topic_;

    // Parameter lane detection
    int hue_white_l = 0, hue_white_h = 179;
    int saturation_white_l = 0, saturation_white_h = 70;
    int value_white_l = 165, value_white_h = 255;
    int hue_yellow_l = 10, hue_yellow_h = 50;
    int saturation_yellow_l = 100, saturation_yellow_h = 255;
    int value_yellow_l = 100, value_yellow_h = 255;

    // Parameter untuk navigasi - NILAI DITINGKATKAN
    double max_linear_speed = 0.08;    // m/s (sedikit diturunkan)
    double max_angular_speed = 1.5;    // rad/s (ditingkatkan)
    double steering_sensitivity = 5.0; // faktor penguatan steering (ditingkatkan)
    double non_linear_factor = 1.5;    // faktor non-linear (lebih dari 1.0 untuk respons lebih agresif)

public:
    LaneDetectServer(std::string name) : as_(nh_, name, boost::bind(&LaneDetectServer::executeCB, this, _1), false),
                                         action_name_(name),
                                         it_(nh_),
                                         image_received_(false)
    {
        // Dapatkan parameter dari ROS parameter server jika tersedia
        nh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_projected_compensated");
        nh_.param<double>("max_linear_speed", max_linear_speed, 0.08);        // Nilai default diturunkan
        nh_.param<double>("max_angular_speed", max_angular_speed, 1.5);       // Nilai default ditingkatkan
        nh_.param<double>("steering_sensitivity", steering_sensitivity, 5.0); // Nilai default ditingkatkan
        nh_.param<double>("non_linear_factor", non_linear_factor, 1.5);       // Nilai default ditambahkan

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers - gunakan topic kamera yang benar
        ROS_INFO("Subscribing to camera topic: %s", camera_topic_.c_str());
        image_sub_ = it_.subscribe(camera_topic_, 1, &LaneDetectServer::imageCallback, this);

        as_.start();
        ROS_INFO("Lane Detection Action Server Started");

        // Tampilkan parameter navigasi
        ROS_INFO("Navigation parameters:");
        ROS_INFO("  max_linear_speed: %.2f m/s", max_linear_speed);
        ROS_INFO("  max_angular_speed: %.2f rad/s", max_angular_speed);
        ROS_INFO("  steering_sensitivity: %.2f", steering_sensitivity);
        ROS_INFO("  non_linear_factor: %.2f", non_linear_factor);
    }

    ~LaneDetectServer()
    {
        // Pastikan robot berhenti saat server dimatikan
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
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

    void detectLane()
    {
        if (!image_received_)
        {
            ROS_WARN_THROTTLE(1, "No camera image received yet on topic: %s", camera_topic_.c_str());
            return;
        }

        // 1. Buat mask untuk white dan yellow lanes
        cv::Mat img_white_mask, img_yellow_mask;
        cv::Scalar lower_white = cv::Scalar(hue_white_l, saturation_white_l, value_white_l);
        cv::Scalar upper_white = cv::Scalar(hue_white_h, saturation_white_h, value_white_h);
        cv::Scalar lower_yellow = cv::Scalar(hue_yellow_l, saturation_yellow_l, value_yellow_l);
        cv::Scalar upper_yellow = cv::Scalar(hue_yellow_h, saturation_yellow_h, value_yellow_h);

        cv::inRange(img_hsv_, lower_white, upper_white, img_white_mask);
        cv::inRange(img_hsv_, lower_yellow, upper_yellow, img_yellow_mask);

        // 2. Kombinasikan masks
        img_masked_ = cv::Mat::zeros(img_bgr_.rows, img_bgr_.cols, CV_8UC1);
        cv::bitwise_or(img_white_mask, img_yellow_mask, img_masked_);

        // 3. Aplikasikan ROI mask - fokus hanya pada bagian bawah gambar
        int height = img_masked_.rows;
        int width = img_masked_.cols;

        // Debug informasi
        ROS_INFO_ONCE("Image dimensions: %d x %d", width, height);

        cv::Mat roi_mask = cv::Mat::zeros(height, width, CV_8UC1);

        std::vector<cv::Point> roi_points;
        roi_points.push_back(cv::Point(0, height));
        roi_points.push_back(cv::Point(width, height));
        roi_points.push_back(cv::Point(width, height / 2));
        roi_points.push_back(cv::Point(0, height / 2));

        cv::fillConvexPoly(roi_mask, roi_points, cv::Scalar(255, 0, 0));
        cv::bitwise_and(img_masked_, roi_mask, img_masked_);

        // 4. Aplikasikan processing sederhana
        cv::GaussianBlur(img_masked_, img_masked_, cv::Size(5, 5), 0);

        // 5. Visualisasi untuk debugging
        cv::Mat debug_img;
        cv::cvtColor(img_masked_, debug_img, cv::COLOR_GRAY2BGR);

        // Gambar garis tengah sebagai referensi
        cv::line(debug_img, cv::Point(width / 2, 0), cv::Point(width / 2, height),
                 cv::Scalar(0, 0, 255), 1);

        // Tampilkan hasil
        cv::imshow("Lane Detection", debug_img);
        cv::waitKey(1);

        // 6. Kontrol pergerakan robot
        moveRobot(img_masked_);
    }

    // Fungsi penguatan non-linear
    double applyNonLinearGain(double value, double factor)
    {
        // Aplikasikan fungsi eksponensial untuk membuat respons lebih agresif pada offset besar
        return copysign(pow(fabs(value), factor), value);
    }

    void moveRobot(const cv::Mat &lane_img)
    {
        int height = lane_img.rows;
        int width = lane_img.cols;

        // Buat beberapa region of interest (ROI) untuk analisis yang lebih baik
        int numRegions = 5;
        std::vector<int> region_centers(numRegions, -1);

        int step = height / (numRegions + 1);
        int center_x = width / 2;

        // Analisis setiap region untuk mendapatkan posisi lane
        cv::Mat debug_img = cv::Mat::zeros(height, width, CV_8UC3);
        cv::cvtColor(lane_img, debug_img, cv::COLOR_GRAY2BGR);

        for (int r = 0; r < numRegions; r++)
        {
            int y = height - (r + 1) * step; // Mulai dari bawah ke atas
            int left_sum = 0, right_sum = 0;
            int left_weighted_sum = 0, right_weighted_sum = 0;

            // Scan area yang lebih luas pada region
            int scan_width = 20; // Lebar scan di setiap sisi dari y

            // Hitung titik-titik lane di region ini
            for (int offset = -scan_width; offset <= scan_width; offset++)
            {
                int scan_y = y + offset;
                if (scan_y < 0 || scan_y >= height)
                    continue;

                for (int x = 0; x < width; x++)
                {
                    if (lane_img.at<uchar>(scan_y, x) > 0)
                    { // Piksel lane (putih)
                        if (x < center_x)
                        {
                            left_sum++;
                            left_weighted_sum += x;
                        }
                        else
                        {
                            right_sum++;
                            right_weighted_sum += x;
                        }
                    }
                }
            }

            // Hitung center lane di region ini
            if (left_sum > 0 && right_sum > 0)
            {
                // Jika ada lane di kedua sisi, ambil titik tengah
                int left_avg = left_weighted_sum / left_sum;
                int right_avg = right_weighted_sum / right_sum;
                region_centers[r] = (left_avg + right_avg) / 2;
            }
            else if (left_sum > 0)
            {
                // Jika hanya ada lane di kiri
                region_centers[r] = left_weighted_sum / left_sum;
            }
            else if (right_sum > 0)
            {
                // Jika hanya ada lane di kanan
                region_centers[r] = right_weighted_sum / right_sum;
            }

            // Debug: Gambar titik center lane di setiap region
            if (region_centers[r] != -1)
            {
                cv::circle(debug_img, cv::Point(region_centers[r], y), 5, cv::Scalar(0, 0, 255), -1);
                cv::line(debug_img, cv::Point(center_x, y), cv::Point(region_centers[r], y),
                         cv::Scalar(0, 255, 0), 2);
            }
        }

        cv::imshow("Lane Centers", debug_img);
        cv::waitKey(1);

        // Tentukan strategi belok berdasarkan analisis region
        double angular_z = 0.0;
        int valid_regions = 0;
        double total_offset = 0.0;

        // Beri bobot lebih tinggi pada region yang lebih dekat dengan robot
        double region_weights[5] = {10.0, 5.0, 3.0, 2.0, 1.0}; // Bobot sangat ditingkatkan untuk region bawah

        for (int r = 0; r < numRegions; r++)
        {
            if (region_centers[r] != -1)
            {
                // Offset dari tengah gambar
                double offset = (region_centers[r] - center_x) / (double)(width / 2); // Normalisasi ke [-1, 1]

                // Aplikasikan faktor non-linear untuk meningkatkan respons pada offset besar
                offset = applyNonLinearGain(offset, non_linear_factor);

                // Tambahkan ke total dengan bobot
                total_offset += offset * region_weights[r];
                valid_regions += region_weights[r];
            }
        }

        if (valid_regions > 0)
        {
            // Hitung rata-rata offset berbobot
            double avg_offset = total_offset / valid_regions;

            // Konversi offset menjadi nilai steering dengan sensitivitas yang ditingkatkan
            angular_z = -1 * avg_offset * max_angular_speed * steering_sensitivity;

            // Batasi nilai maksimum steering
            if (angular_z > max_angular_speed)
                angular_z = max_angular_speed;
            if (angular_z < -max_angular_speed)
                angular_z = -max_angular_speed;

            ROS_INFO("Norm Offset: %.2f, Angular: %.2f", avg_offset, angular_z);
        }
        else
        {
            ROS_WARN_THROTTLE(1, "No lane detected in any region!");
            angular_z = 0.0;
        }

        // Perhitungan adaptive untuk kecepatan linear berdasarkan kemudi
        // Kurangi kecepatan saat belok tajam
        double linear_x = max_linear_speed * (1.0 - 0.7 * fabs(angular_z / max_angular_speed));

        // Buat dan kirim perintah pergerakan
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_x;
        cmd.angular.z = angular_z;

        // Debug info
        ROS_INFO_THROTTLE(0.5, "CMD: linear.x=%.2f, angular.z=%.2f", cmd.linear.x, cmd.angular.z);

        // Publish perintah
        cmd_vel_pub_.publish(cmd);
    }

    void executeCB(const msg_file::LaneDetectGoalConstPtr &goal)
    {
        bool success = true;
        start_time_ = ros::Time::now();
        ros::Rate r(10); // 10 Hz

        ROS_INFO("LaneDetect Server: Executing dengan durasi %f detik", goal->duration);

        // Loop hingga mencapai durasi yang ditentukan
        while (ros::Time::now() - start_time_ < ros::Duration(goal->duration))
        {
            // Cek jika preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            // Lakukan deteksi lane dan kontrol robot
            detectLane();

            // Hitung waktu yang telah berlalu untuk feedback
            feedback_.time_elapsed = (ros::Time::now() - start_time_).toSec();

            // Publish feedback
            as_.publishFeedback(feedback_);

            r.sleep();
        }

        // Hentikan robot ketika selesai
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Stopping robot");

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
    ros::init(argc, argv, "lane_detect_server");
    LaneDetectServer laneDetectServer("lane_detect");
    ros::spin();
    return 0;
}