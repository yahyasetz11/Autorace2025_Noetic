#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <scan_debug/ScanDebugConfig.h>
#include <cmath>
#include <vector>
#include <iomanip>

class ScanDebugNode
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<scan_debug::ScanDebugConfig> server_;
    dynamic_reconfigure::Server<scan_debug::ScanDebugConfig>::CallbackType f_;

    // Parameters
    double center_angle_;
    double angle_range_;
    bool show_details_;

    // Scan data
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_;
    bool info_printed_;

public:
    ScanDebugNode() : scan_received_(false), info_printed_(false)
    {
        // Subscribe to scan topic
        scan_sub_ = nh_.subscribe("/scan", 1, &ScanDebugNode::scanCallback, this);

        // Setup dynamic reconfigure
        f_ = boost::bind(&ScanDebugNode::configCallback, this, _1, _2);
        server_.setCallback(f_);

        ROS_INFO("Scan Debug Node started");
    }

    void configCallback(scan_debug::ScanDebugConfig &config, uint32_t level)
    {
        center_angle_ = config.center_angle;
        angle_range_ = config.angle_range;
        show_details_ = config.show_details;

        ROS_INFO("Updated parameters: center=%.1f°, range=±%.1f°",
                 center_angle_, angle_range_);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        latest_scan_ = *scan;
        scan_received_ = true;

        // Print scan info once
        if (!info_printed_)
        {
            printScanInfo();
            info_printed_ = true;
        }

        // Process the scan based on current parameters
        processSelectedRange();
    }

    void printScanInfo()
    {
        ROS_INFO("=== LIDAR Configuration ===");
        ROS_INFO("Array size: %lu", latest_scan_.ranges.size());
        ROS_INFO("Angle min: %.2f° (%.3f rad)",
                 latest_scan_.angle_min * 180.0 / M_PI, latest_scan_.angle_min);
        ROS_INFO("Angle max: %.2f° (%.3f rad)",
                 latest_scan_.angle_max * 180.0 / M_PI, latest_scan_.angle_max);
        ROS_INFO("Angle increment: %.4f° (%.6f rad)",
                 latest_scan_.angle_increment * 180.0 / M_PI, latest_scan_.angle_increment);
        ROS_INFO("Time increment: %.6f s", latest_scan_.time_increment);
        ROS_INFO("Scan time: %.6f s", latest_scan_.scan_time);
        ROS_INFO("Range min: %.3f m", latest_scan_.range_min);
        ROS_INFO("Range max: %.3f m", latest_scan_.range_max);

        // Determine scan direction
        if (latest_scan_.angle_increment > 0)
        {
            ROS_INFO("Scan direction: Counter-clockwise");
        }
        else
        {
            ROS_INFO("Scan direction: Clockwise");
        }

        // Print some key angles and their indices
        ROS_INFO("\n=== Key Angles ===");
        double angles[] = {0.0, 90.0, 180.0, -90.0, -180.0};
        const char *labels[] = {"Front", "Left", "Back", "Right", "Back"};

        for (int i = 0; i < 5; i++)
        {
            int index = angleToIndex(angles[i]);
            if (index >= 0 && index < latest_scan_.ranges.size())
            {
                double actual_angle = indexToAngle(index);
                ROS_INFO("%s (%.1f°): index=%d, actual_angle=%.2f°",
                         labels[i], angles[i], index, actual_angle);
            }
        }
        ROS_INFO("===========================\n");
    }

    int angleToIndex(double angle_deg)
    {
        // Convert to radians
        double angle_rad = angle_deg * M_PI / 180.0;

        // Normalize angle to be within the scan range
        while (angle_rad < latest_scan_.angle_min)
            angle_rad += 2 * M_PI;
        while (angle_rad > latest_scan_.angle_max)
            angle_rad -= 2 * M_PI;

        // Calculate index
        int index = round((angle_rad - latest_scan_.angle_min) / latest_scan_.angle_increment);

        // Ensure index is within bounds
        if (index < 0)
            index = 0;
        if (index >= latest_scan_.ranges.size())
            index = latest_scan_.ranges.size() - 1;

        return index;
    }

    double indexToAngle(int index)
    {
        double angle_rad = latest_scan_.angle_min + index * latest_scan_.angle_increment;
        return angle_rad * 180.0 / M_PI;
    }

    void processSelectedRange()
    {
        if (!scan_received_)
            return;

        // Calculate the range of angles to examine
        double start_angle = center_angle_ - angle_range_;
        double end_angle = center_angle_ + angle_range_;

        // Get indices for the range
        int start_idx = angleToIndex(start_angle);
        int end_idx = angleToIndex(end_angle);

        // Collect valid ranges in this area
        std::vector<double> valid_ranges;
        std::vector<int> indices;

        // Handle wraparound if necessary
        if (start_idx <= end_idx)
        {
            for (int i = start_idx; i <= end_idx; i++)
            {
                if (!std::isnan(latest_scan_.ranges[i]) &&
                    !std::isinf(latest_scan_.ranges[i]) &&
                    latest_scan_.ranges[i] > latest_scan_.range_min &&
                    latest_scan_.ranges[i] < latest_scan_.range_max)
                {
                    valid_ranges.push_back(latest_scan_.ranges[i]);
                    indices.push_back(i);
                }
            }
        }
        else
        {
            // Handle wraparound case
            for (int i = start_idx; i < latest_scan_.ranges.size(); i++)
            {
                if (!std::isnan(latest_scan_.ranges[i]) &&
                    !std::isinf(latest_scan_.ranges[i]) &&
                    latest_scan_.ranges[i] > latest_scan_.range_min &&
                    latest_scan_.ranges[i] < latest_scan_.range_max)
                {
                    valid_ranges.push_back(latest_scan_.ranges[i]);
                    indices.push_back(i);
                }
            }
            for (int i = 0; i <= end_idx; i++)
            {
                if (!std::isnan(latest_scan_.ranges[i]) &&
                    !std::isinf(latest_scan_.ranges[i]) &&
                    latest_scan_.ranges[i] > latest_scan_.range_min &&
                    latest_scan_.ranges[i] < latest_scan_.range_max)
                {
                    valid_ranges.push_back(latest_scan_.ranges[i]);
                    indices.push_back(i);
                }
            }
        }

        // Calculate statistics
        if (!valid_ranges.empty())
        {
            double sum = 0.0;
            double min_range = valid_ranges[0];
            double max_range = valid_ranges[0];

            for (double range : valid_ranges)
            {
                sum += range;
                if (range < min_range)
                    min_range = range;
                if (range > max_range)
                    max_range = range;
            }

            double average = sum / valid_ranges.size();

            // Print results
            std::cout << "\n=== Scan Analysis ===" << std::endl;
            std::cout << "Center angle: " << std::fixed << std::setprecision(1)
                      << center_angle_ << "°" << std::endl;
            std::cout << "Range: ±" << angle_range_ << "° ("
                      << start_angle << "° to " << end_angle << "°)" << std::endl;
            std::cout << "Indices: " << start_idx << " to " << end_idx << std::endl;
            std::cout << "Valid readings: " << valid_ranges.size()
                      << "/" << (abs(end_idx - start_idx) + 1) << std::endl;
            std::cout << "Average distance: " << std::setprecision(3)
                      << average << " m" << std::endl;
            std::cout << "Min distance: " << min_range << " m" << std::endl;
            std::cout << "Max distance: " << max_range << " m" << std::endl;

            if (show_details_)
            {
                std::cout << "\nDetailed readings:" << std::endl;
                for (size_t i = 0; i < valid_ranges.size(); i++)
                {
                    double angle = indexToAngle(indices[i]);
                    std::cout << "  Index " << indices[i]
                              << " (angle " << std::setprecision(1) << angle
                              << "°): " << std::setprecision(3)
                              << valid_ranges[i] << " m" << std::endl;
                }
            }
            std::cout << "==================\n"
                      << std::endl;
        }
        else
        {
            std::cout << "\nNo valid readings in the selected range!" << std::endl;
            std::cout << "Center: " << center_angle_ << "°, Range: ±"
                      << angle_range_ << "°\n"
                      << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_debug_node");
    ScanDebugNode node;
    ros::spin();
    return 0;
}