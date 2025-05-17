#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <msg_file/EulerAngles.h>

class YawTracker
{
public:
    YawTracker() : initial_yaw_set_(false), current_yaw_(0.0)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters if specified
        std::string imu_topic;
        private_nh.param<std::string>("imu_topic", imu_topic, "/imu");

        // Publishers
        euler_pub_ = nh.advertise<msg_file::EulerAngles>("euler_angles", 10);

        // Subscriber
        imu_sub_ = nh.subscribe(imu_topic, 10, &YawTracker::imuCallback, this);

        // Initialize start time
        start_time_ = ros::Time::now();

        ROS_INFO("Yaw Tracker initialized. Waiting for initial IMU reading...");
        ROS_INFO("Using IMU topic: %s", imu_topic.c_str());
    }

    ~YawTracker()
    {
        saveHistory("yaw_history.csv");
    }

    void run()
    {
        ros::spin();
    }

private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
    {
        // Extract quaternion from IMU message
        tf2::Quaternion q;
        tf2::fromMsg(imu_msg->orientation, q);

        // Convert quaternion to Euler angles (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // Convert to degrees
        double roll_degrees = roll * 180.0 / M_PI;
        double pitch_degrees = pitch * 180.0 / M_PI;
        double yaw_degrees = yaw * 180.0 / M_PI;

        // Set initial yaw if not already set
        if (!initial_yaw_set_)
        {
            initial_yaw_ = yaw_degrees;
            initial_roll_ = roll_degrees;
            initial_pitch_ = pitch_degrees;
            initial_yaw_set_ = true;
            ROS_INFO_STREAM("Initial orientation set to: roll=" << initial_roll_
                                                                << ", pitch=" << initial_pitch_ << ", yaw=" << initial_yaw_ << " degrees");
        }

        // Calculate relative angles
        double rel_roll = roll_degrees - initial_roll_;
        double rel_pitch = pitch_degrees - initial_pitch_;
        double rel_yaw = yaw_degrees - initial_yaw_;

        // Normalize to [-180, 180] range
        if (rel_yaw > 180.0)
        {
            rel_yaw -= 360.0;
        }
        else if (rel_yaw < -180.0)
        {
            rel_yaw += 360.0;
        }

        // Same for roll and pitch
        if (rel_roll > 180.0)
        {
            rel_roll -= 360.0;
        }
        else if (rel_roll < -180.0)
        {
            rel_roll += 360.0;
        }

        if (rel_pitch > 180.0)
        {
            rel_pitch -= 360.0;
        }
        else if (rel_pitch < -180.0)
        {
            rel_pitch += 360.0;
        }

        current_yaw_ = rel_yaw;

        // Record yaw with timestamp
        ros::Time current_time = ros::Time::now();
        double elapsed_time = (current_time - start_time_).toSec();
        yaw_history_.push_back(std::make_pair(elapsed_time, rel_yaw));

        // Publish all Euler angles with our custom message
        msg_file::EulerAngles euler_msg;
        euler_msg.roll = rel_roll;
        euler_msg.pitch = rel_pitch;
        euler_msg.yaw = rel_yaw;

        // Also include angular velocity if available in the IMU message
        if (!imu_msg->angular_velocity.z == 0)
        {
            euler_msg.angular_velocity_z = imu_msg->angular_velocity.z * 180.0 / M_PI; // Convert to deg/sec
        }

        euler_pub_.publish(euler_msg);

        // Log every 5 seconds (to avoid flooding the console)
        int elapsed_time_int = static_cast<int>(elapsed_time);
        if (elapsed_time_int % 5 == 0 && std::abs(elapsed_time - elapsed_time_int) < 0.1)
        {
            ROS_INFO_STREAM("Current orientation: roll=" << std::fixed << std::setprecision(2) << rel_roll
                                                         << ", pitch=" << std::fixed << std::setprecision(2) << rel_pitch
                                                         << ", yaw=" << std::fixed << std::setprecision(2) << rel_yaw
                                                         << " degrees (elapsed time: " << std::fixed << std::setprecision(2)
                                                         << elapsed_time << "s)");
        }
    }

    void saveHistory(const std::string &filename)
    {
        try
        {
            std::ofstream file(filename);
            if (file.is_open())
            {
                file << "time,yaw_degrees" << std::endl;
                for (const auto &entry : yaw_history_)
                {
                    file << entry.first << "," << entry.second << std::endl;
                }
                file.close();
                ROS_INFO_STREAM("Yaw history saved to " << filename);
            }
            else
            {
                ROS_ERROR_STREAM("Failed to open file: " << filename);
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Failed to save yaw history: " << e.what());
        }
    }

    // ROS handles
    ros::Subscriber imu_sub_;
    ros::Publisher euler_pub_; // Our new custom message

    // State variables
    bool initial_yaw_set_;
    double initial_roll_;
    double initial_pitch_;
    double initial_yaw_;
    double current_yaw_;
    ros::Time start_time_;
    std::vector<std::pair<double, double>> yaw_history_; // (time, yaw) pairs
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yaw_tracker");

    YawTracker tracker;
    tracker.run();

    return 0;
}