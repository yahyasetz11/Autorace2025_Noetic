#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/AlignAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>

// Define constants
#define ALIGN_TOLERANCE 0.025 // ~3 degrees difference
#define ALIGN_TIMEOUT 30.0    // 30 seconds default
#define MAX_ANGULAR_SPEED 0.8
#define MIN_ANGULAR_SPEED 0.1
#define ANGULAR_P_GAIN 2.0
#define VERIFICATION_TIME 2.0 // 2 seconds verification time

class AlignServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::AlignAction> as_;
    std::string action_name_;
    msg_file::AlignFeedback feedback_;
    msg_file::AlignResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber scan_sub_;

    // Parameters
    std::string align_direction_;
    ros::Time start_time_;
    bool scan_received_;
    sensor_msgs::LaserScan latest_scan_;

    // Verification variables
    ros::Time aligned_since_;
    bool in_verification_;

public:
    AlignServer(std::string name) : as_(nh_, name, boost::bind(&AlignServer::executeCB, this, _1), false),
                                    action_name_(name),
                                    scan_received_(false),
                                    in_verification_(false)
    {
        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        scan_sub_ = nh_.subscribe("/scan", 1, &AlignServer::scanCallback, this);

        as_.start();
        ROS_INFO("Align Action Server Started");
    }

    ~AlignServer()
    {
        // Stop the robot when the server shuts down
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        latest_scan_ = *scan;
        scan_received_ = true;
    }

    // Calculate alignment error based on laser scan
    double calculateAlignmentError()
    {
        if (!scan_received_)
        {
            ROS_WARN_THROTTLE(1, "No laser scan data received yet!");
            return 0.0;
        }

        double error = 0.0;
        const unsigned int scan_size = latest_scan_.ranges.size();
        const double angle_increment = latest_scan_.angle_increment;

        // Calculate indices for the corresponding angles
        // For a typical 360° lidar with 0° at the front:
        // right = 270° (3π/2), left = 90° (π/2), front = 0°, back = 180° (π)
        unsigned int front_idx = 0;
        unsigned int right_idx = static_cast<unsigned int>(3 * M_PI / 2 / angle_increment);
        unsigned int back_idx = static_cast<unsigned int>(M_PI / angle_increment);
        unsigned int left_idx = static_cast<unsigned int>(M_PI / 2 / angle_increment);

        // Ensure indices are within bounds
        right_idx = std::min(right_idx, scan_size - 1);
        left_idx = std::min(left_idx, scan_size - 1);
        back_idx = std::min(back_idx, scan_size - 1);

        // Check scan indices
        ROS_INFO_ONCE("Scan indices - Front: %u, Right: %u, Back: %u, Left: %u",
                      front_idx, right_idx, back_idx, left_idx);

        // Determine offset to check (10 degrees = 10 * π/180 radians)
        int offset = static_cast<int>(10 * M_PI / 180 / angle_increment);

        if (align_direction_ == "right")
        {
            // Get ranges at right side +/- offset
            unsigned int right_plus_idx = (right_idx + offset) % scan_size;
            unsigned int right_minus_idx = (right_idx - offset + scan_size) % scan_size;

            double range_plus = latest_scan_.ranges[right_plus_idx];
            double range_minus = latest_scan_.ranges[right_minus_idx];

            // For parallel alignment, we want ranges at +offset and -offset to be equal
            error = range_plus - range_minus;

            ROS_INFO_THROTTLE(0.5, "Right alignment - Range+: %.3f, Range-: %.3f, Error: %.3f",
                              range_plus, range_minus, error);
        }
        else if (align_direction_ == "left")
        {
            // Get ranges at left side +/- offset
            unsigned int left_plus_idx = (left_idx + offset) % scan_size;
            unsigned int left_minus_idx = (left_idx - offset + scan_size) % scan_size;

            double range_plus = latest_scan_.ranges[left_plus_idx];
            double range_minus = latest_scan_.ranges[left_minus_idx];

            // For parallel alignment, we want ranges at +offset and -offset to be equal
            error = range_plus - range_minus;

            ROS_INFO_THROTTLE(0.5, "Left alignment - Range+: %.3f, Range-: %.3f, Error: %.3f",
                              range_plus, range_minus, error);
        }
        else if (align_direction_ == "front")
        {
            // For front perpendicular alignment, compare left and right sides
            // We want (front-left - front) = (front - front-right)
            unsigned int front_left_idx = (front_idx + offset) % scan_size;
            unsigned int front_right_idx = (front_idx - offset + scan_size) % scan_size;

            double range_front = latest_scan_.ranges[front_idx];
            double range_left = latest_scan_.ranges[front_left_idx];
            double range_right = latest_scan_.ranges[front_right_idx];

            double diff_left = range_left - range_front;
            double diff_right = range_front - range_right;

            error = diff_left - diff_right;

            ROS_INFO_THROTTLE(0.5, "Front alignment - Range front: %.3f, Left: %.3f, Right: %.3f, Error: %.3f",
                              range_front, range_left, range_right, error);
        }
        else if (align_direction_ == "back")
        {
            // For back perpendicular alignment, compare left and right sides
            unsigned int back_left_idx = (back_idx + offset) % scan_size;
            unsigned int back_right_idx = (back_idx - offset + scan_size) % scan_size;

            double range_back = latest_scan_.ranges[back_idx];
            double range_left = latest_scan_.ranges[back_left_idx];
            double range_right = latest_scan_.ranges[back_right_idx];

            double diff_left = range_left - range_back;
            double diff_right = range_back - range_right;

            error = diff_left - diff_right;

            ROS_INFO_THROTTLE(0.5, "Back alignment - Range back: %.3f, Left: %.3f, Right: %.3f, Error: %.3f",
                              range_back, range_left, range_right, error);
        }
        else
        {
            ROS_WARN("Unknown alignment direction: %s", align_direction_.c_str());
        }

        return error;
    }

    void executeCB(const msg_file::AlignGoalConstPtr &goal)
    {
        bool success = false;
        align_direction_ = goal->by;
        start_time_ = ros::Time::now();
        in_verification_ = false;
        ros::Rate r(10); // 10 Hz

        ROS_INFO("Align Server: Direction=%s, Tolerance=%.3f, Timeout=%.1f",
                 align_direction_.c_str(), ALIGN_TOLERANCE, ALIGN_TIMEOUT);

        // Wait for first scan data if not already received
        ros::Time wait_start = ros::Time::now();
        while (!scan_received_ && ros::ok() && (ros::Time::now() - wait_start).toSec() < 5.0)
        {
            ROS_INFO_THROTTLE(1.0, "Waiting for laser scan data...");
            ros::Duration(0.1).sleep();
        }

        if (!scan_received_)
        {
            ROS_ERROR("No laser scan data received after waiting. Cannot perform alignment.");
            result_.success = false;
            as_.setAborted(result_);
            return;
        }

        // Main execution loop
        while (ros::ok())
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            // Calculate time elapsed
            double time_elapsed = (ros::Time::now() - start_time_).toSec();

            // Check timeout
            if (time_elapsed > ALIGN_TIMEOUT)
            {
                ROS_INFO("%s: Timed out after %.1f seconds", action_name_.c_str(), time_elapsed);
                success = false;
                break;
            }

            // Calculate current alignment error
            double error = calculateAlignmentError();

            // Check if aligned within tolerance
            if (fabs(error) < ALIGN_TOLERANCE)
            {
                if (!in_verification_)
                {
                    // Just entered verification state
                    in_verification_ = true;
                    aligned_since_ = ros::Time::now();
                    ROS_INFO("Started alignment verification - Error: %.3f", error);
                }
                else
                {
                    // Already in verification state, check if enough time has passed
                    double aligned_time = (ros::Time::now() - aligned_since_).toSec();

                    if (aligned_time >= VERIFICATION_TIME)
                    {
                        ROS_INFO("%s: Alignment verified for %.1f seconds! Error: %.3f",
                                 action_name_.c_str(), aligned_time, error);
                        success = true;
                        break;
                    }

                    ROS_INFO_THROTTLE(0.5, "Verifying alignment: %.1f/%.1f seconds, Error: %.3f",
                                      aligned_time, VERIFICATION_TIME, error);
                }
            }
            else if (in_verification_)
            {
                // Was in verification but error is now too high - reset
                in_verification_ = false;
                ROS_INFO("Alignment lost during verification! Error: %.3f", error);
            }

            // Calculate control command (skip if in verification)
            double angular_z = 0.0;
            if (!in_verification_)
            {
                angular_z = -error * ANGULAR_P_GAIN;

                // Apply min/max limits to angular velocity
                if (fabs(angular_z) < MIN_ANGULAR_SPEED && fabs(angular_z) > 0.01)
                {
                    angular_z = (angular_z > 0) ? MIN_ANGULAR_SPEED : -MIN_ANGULAR_SPEED;
                }

                if (fabs(angular_z) > MAX_ANGULAR_SPEED)
                {
                    angular_z = (angular_z > 0) ? MAX_ANGULAR_SPEED : -MAX_ANGULAR_SPEED;
                }
            }

            // Create and send movement command
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0; // No forward movement during alignment
            cmd.angular.z = angular_z;
            cmd_vel_pub_.publish(cmd);

            // Update feedback
            feedback_.time_elapsed = time_elapsed;
            feedback_.current_error = error;
            as_.publishFeedback(feedback_);

            if (!in_verification_)
            {
                ROS_INFO_THROTTLE(0.5, "Aligning %s - Error: %.3f, Angular vel: %.2f, Time: %.1f",
                                  align_direction_.c_str(), error, angular_z, time_elapsed);
            }

            r.sleep();
        }

        // Stop the robot when done
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);

        // Set result
        result_.success = success;

        if (success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
        else
        {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "align_server");
    AlignServer alignServer("align");
    ros::spin();
    return 0;
}