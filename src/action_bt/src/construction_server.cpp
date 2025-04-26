#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_file/ConstructionAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>
#include <cmath>

class ConstructionServer
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<msg_file::ConstructionAction> as_;
    std::string action_name_;
    msg_file::ConstructionFeedback feedback_;
    msg_file::ConstructionResult result_;

    // ROS communication
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber laser_sub_;

    // LIDAR data
    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_;

    // Algorithm phases
    enum Phase
    {
        MOVE_TO_WALL,
        TURN_LEFT_90,
        TURN_LEFT_LAST,
        ALIGN_WITH_LEFT_WALL,
        ALIGN_WITH_RIGHT_WALL,
        MOVE_UNTIL_RIGHT_FRONT_CLEAR,
        TURN_RIGHT_180,
        MOVE_UNTIL_LEFT_FRONT_CLEAR,
        TURN_LEFT_180,
        COMPLETED
    };
    Phase current_phase_;

    // Movement parameters
    double linear_speed_;
    double angular_speed_;

    // Distance and angle parameters
    double wall_distance_threshold_;
    double wall_side_distance_threshold_;
    double front_scan_angle_;
    double side_scan_angle_;
    double turn_precision_threshold_;
    double turn_linear_speed_;
    double desired_radius_;

    // Turn tracking
    double turn_target_angle_;
    double current_angle_;
    ros::Time turn_start_time_;
    ros::Time phase_start_time_;

    // Config file paths
    std::string speed_config_path_;
    std::string construction_config_path_;

public:
    ConstructionServer(std::string name) : as_(nh_, name, boost::bind(&ConstructionServer::executeCB, this, _1), false),
                                           action_name_(name),
                                           scan_received_(false),
                                           current_phase_(MOVE_TO_WALL),
                                           current_angle_(0.0)
    {
        // Define config file paths (using the src/config directory)
        speed_config_path_ = ros::package::getPath("action_bt") + "/../config/speed_conf.yaml";
        construction_config_path_ = ros::package::getPath("action_bt") + "/../config/construction_params.yaml";

        // Load parameters from YAML file
        loadSpeedParameters();
        loadConstructionParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        laser_sub_ = nh_.subscribe("/scan", 1, &ConstructionServer::laserCallback, this);

        as_.start();
        ROS_INFO("Construction Action Server Started");
        ROS_INFO("Speed parameters from %s:", speed_config_path_.c_str());
        ROS_INFO("  linear_speed: %.2f m/s", linear_speed_);
        ROS_INFO("  angular_speed: %.2f rad/s", angular_speed_);

        ROS_INFO("Construction parameters from %s:", construction_config_path_.c_str());
        ROS_INFO("  wall_distance_threshold: %.2f m", wall_distance_threshold_);
        ROS_INFO("  wall_side_distance_threshold: %.2f m", wall_side_distance_threshold_);
        ROS_INFO("  front_scan_angle: %.2f degrees", front_scan_angle_);
        ROS_INFO("  side_scan_angle: %.2f degrees", side_scan_angle_);
        ROS_INFO("  turn_precision_threshold: %.2f rad", turn_precision_threshold_);
    }

    ~ConstructionServer()
    {
        // Stop robot when server shuts down
        stopRobot();
    }

    void loadSpeedParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(speed_config_path_);

            // Extract parameters with defaults if not found
            // For consistency, we use max_linear_speed and max_angular_speed from speed_conf.yaml
            linear_speed_ = config["max_linear_speed"] ? config["max_linear_speed"].as<double>() * 0.5 : 0.08;
            angular_speed_ = config["max_angular_speed"] ? config["max_angular_speed"].as<double>() * 0.7 : 0.5;

            ROS_INFO("Successfully loaded speed parameters from: %s", speed_config_path_.c_str());
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading speed configuration YAML file: %s", e.what());
            ROS_WARN("Using default speed parameters");
            // Default values
            linear_speed_ = 0.08;
            angular_speed_ = 0.5;
        }
    }

    void loadConstructionParameters()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(construction_config_path_);

            // Extract parameters from construction section
            if (config["construction"])
            {
                YAML::Node construction = config["construction"];

                // Only override linear_speed and angular_speed if they're explicitly specified in construction params
                if (construction["linear_speed"])
                    linear_speed_ = construction["linear_speed"].as<double>();

                if (construction["angular_speed"])
                    angular_speed_ = construction["angular_speed"].as<double>();

                wall_distance_threshold_ = construction["wall_distance_threshold"] ? construction["wall_distance_threshold"].as<double>() : 0.3;

                wall_side_distance_threshold_ = construction["wall_side_distance_threshold"] ? construction["wall_side_distance_threshold"].as<double>() : 4;

                front_scan_angle_ = construction["front_scan_angle"] ? construction["front_scan_angle"].as<double>() : 10.0;

                side_scan_angle_ = construction["side_scan_angle"] ? construction["side_scan_angle"].as<double>() : 60.0;

                turn_precision_threshold_ = construction["turn_precision_threshold"] ? construction["turn_precision_threshold"].as<double>() : 0.1;

                turn_linear_speed_ = construction["turn_linear_speed"] ? construction["turn_linear_speed"].as<double>() : 0.08;

                desired_radius_ = construction["desired_radius"] ? construction["desired_radius"].as<double>() : 0.5;
            }
            else
            {
                ROS_WARN("Construction section not found in config file. Using defaults.");
                wall_distance_threshold_ = 0.3;
                wall_side_distance_threshold_ = 4;
                front_scan_angle_ = 10.0;
                side_scan_angle_ = 60.0;
                turn_precision_threshold_ = 0.1;
                turn_linear_speed_ = 0.08;
                desired_radius_ = 0.5;

                ROS_INFO("Successfully loaded construction parameters from: %s", construction_config_path_.c_str());
            }
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading construction configuration YAML file: %s", e.what());
            ROS_WARN("Using default construction parameters");
            wall_distance_threshold_ = 0.3;
            wall_side_distance_threshold_ = 4;
            front_scan_angle_ = 10.0;
            side_scan_angle_ = 60.0;
            turn_precision_threshold_ = 0.1;
            turn_linear_speed_ = 0.08;
            desired_radius_ = 0.5;
        }
    }

    void
    laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        latest_scan_ = *scan;
        scan_received_ = true;
    }

    double getDistanceInRange(double angle_degrees)
    {
        if (!scan_received_)
        {
            ROS_WARN_THROTTLE(1, "No laser scan data received yet");
            return -1.0;
        }

        // Convert degrees to radians
        double angle_rad = angle_degrees * M_PI / 180.0;

        // Convert angle to array index
        int index = (angle_rad - latest_scan_.angle_min) / latest_scan_.angle_increment;

        // Check index bounds
        if (index < 0 || index >= static_cast<int>(latest_scan_.ranges.size()))
        {
            ROS_WARN("Angle %.2f degrees out of scan range", angle_degrees);
            return -1.0;
        }

        double range = latest_scan_.ranges[index];

        // Check if range is valid
        if (std::isnan(range) || std::isinf(range))
        {
            return latest_scan_.range_max;
        }

        return range;
    }

    double getFrontDistance()
    {
        double min_dist = latest_scan_.range_max;

        // Check ranges in front of the robot within front_scan_angle_
        for (double angle = -front_scan_angle_; angle <= front_scan_angle_; angle += 1.0)
        {
            double dist = getDistanceInRange(angle);
            if (dist > 0 && dist < min_dist)
            {
                min_dist = dist;
            }
        }

        return min_dist;
    }

    double getLeftSideDistance(double angle_offset = 0.0)
    {
        // Get distance at 90 degrees to the left + any offset
        double angle = 90.0 + angle_offset;
        return getDistanceInRange(angle);
    }

    double getRightSideDistance(double angle_offset = 0.0)
    {
        // Get distance at 90 degrees to the right + any offset
        double angle = 270.0 + angle_offset;
        return getDistanceInRange(angle);
    }

    bool isParallelToLeftWall()
    {
        // Compare distances at two points along the wall
        double front_left = getLeftSideDistance(-15.0); // 75 degrees (forward-left)
        double back_left = getLeftSideDistance(15.0);   // 105 degrees (backward-left)

        // If distances are similar, robot is parallel
        double diff = std::abs(front_left - back_left);

        ROS_INFO("Wall alignment check - Front: %.2f, Back: %.2f, Diff: %.2f",
                 front_left, back_left, diff);

        // Consider parallel if difference is less than 5cm
        return diff < 0.05;
    }

    bool isParallelToRightWall()
    {
        // Compare distances at two points along the wall
        double front_right = getRightSideDistance(-15.0); // 75 degrees (forward-right)
        double back_right = getRightSideDistance(15.0);   // 105 degrees (backward-right)

        // If distances are similar, robot is parallel
        double diff = std::abs(front_right - back_right);

        ROS_INFO("Wall alignment check - Front: %.2f, Back: %.2f, Diff: %.2f",
                 front_right, back_right, diff);

        // Consider parallel if difference is less than 5cm
        return diff < 0.05;
    }

    double getRightFrontDistance()
    {
        double min_dist = latest_scan_.range_max;

        // Check ranges in right front area
        ROS_INFO("Checking angles from %.1f to %.1f degrees", 360.0 - (side_scan_angle_ - 3.0), 360.0 - side_scan_angle_);
        for (double angle = 360.0 - (side_scan_angle_ - 10.0); angle <= 360 - side_scan_angle_; angle += 1.0)
        {
            double dist = getDistanceInRange(angle);
            ROS_INFO("Angle: %.1f, Distance: %.2f", angle, dist);
            if (dist > 0 && dist < min_dist)
            {
                min_dist = dist;
            }
        }

        return min_dist;
    }

    double getLeftFrontDistance()
    {
        double min_dist = latest_scan_.range_max;

        // Check ranges in left front area
        ROS_INFO("Checking angles from %.1f to %.1f degrees", 360.0 - (side_scan_angle_ - 3.0), 360.0 - side_scan_angle_);
        for (double angle = side_scan_angle_; angle <= side_scan_angle_ + 10.0; angle += 1.0)
        {
            double dist = getDistanceInRange(angle);
            ROS_INFO("Angle: %.1f, Distance: %.2f", angle, dist);
            if (dist > 0 && dist < min_dist)
            {
                min_dist = dist;
            }
        }

        return min_dist;
    }

    void stopRobot()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
        ROS_INFO("Robot stopped");
    }

    void moveForward()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = linear_speed_;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    void rotateLeft(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd);
    }

    void rotateRight(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = -angular_speed;
        cmd_vel_pub_.publish(cmd);
    }

    void turnLeft(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.08;
        cmd.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd);
    }

    void turnRight(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.08;
        cmd.angular.z = -angular_speed;
        cmd_vel_pub_.publish(cmd);
    }

    void turnWithRadius(bool turnRight, double desired_radius, double linear_speed)
    {
        geometry_msgs::Twist cmd;

        // Ensure minimum radius to prevent division by zero
        double radius = std::max(desired_radius, 0.1);

        // Calculate the required angular velocity based on the desired radius
        // radius = linear_speed / angular_speed
        // so: angular_speed = linear_speed / radius
        double angular_speed = linear_speed / radius;

        // Set linear velocity
        cmd.linear.x = linear_speed;

        // Set angular velocity with direction
        if (turnRight)
        {
            cmd.angular.z = -angular_speed;
        }
        else
        {
            cmd.angular.z = angular_speed;
        }

        cmd_vel_pub_.publish(cmd);

        // Debug output
        ROS_INFO_THROTTLE(0.5, "Radius turn: linear=%.2f, angular=%.2f, target radius=%.2f",
                          cmd.linear.x, cmd.angular.z, radius);
    }

    void executeCB(const msg_file::ConstructionGoalConstPtr &goal)
    {
        bool success = true;
        phase_start_time_ = ros::Time::now();
        ros::Rate r(10); // 10Hz

        // Reset phase to beginning
        current_phase_ = MOVE_TO_WALL;

        ROS_INFO("Starting Construction Mission");

        // Wait for first laser scan to arrive
        ros::Time start_wait = ros::Time::now();
        while (!scan_received_ && ros::ok())
        {
            if ((ros::Time::now() - start_wait).toSec() > 5.0)
            {
                ROS_ERROR("No laser scan received after 5 seconds. Aborting mission.");
                as_.setAborted();
                return;
            }
            ROS_INFO_THROTTLE(1, "Waiting for laser scan data...");
            ros::spinOnce();
            r.sleep();
        }

        while (ros::ok() && current_phase_ != COMPLETED)
        {
            // Check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                stopRobot();
                break;
            }

            // Process current phase
            switch (current_phase_)
            {
            case MOVE_TO_WALL:
            {
                feedback_.current_phase = "Moving toward wall";
                double front_dist = getFrontDistance();

                ROS_INFO_THROTTLE(0.5, "Phase: Move to wall, distance: %.2f (threshold: %.2f)",
                                  front_dist, wall_distance_threshold_);

                if (front_dist <= wall_distance_threshold_)
                {
                    // Close enough to the wall, stop and transition to next phase
                    stopRobot();
                    current_phase_ = TURN_LEFT_90;
                    turn_start_time_ = ros::Time::now();
                    current_angle_ = 0;
                    ROS_INFO("Reached wall. Starting left turn.");
                }
                else
                {
                    // Move forward toward wall
                    moveForward();
                }
                break;
            }

            case TURN_LEFT_90:
            {
                feedback_.current_phase = "Turning left 90 degrees";

                // Simple timed turn - 90 degrees
                double turn_duration = 97.0 / (angular_speed_ * (180.0 / M_PI));
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Phase: Turn left 90, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = ALIGN_WITH_RIGHT_WALL;
                    ROS_INFO("Left turn complete. Aligning with the Right Wall.");
                }
                else
                {
                    // Continue turning
                    turnLeft(angular_speed_);
                }
                break;
            }

            case ALIGN_WITH_LEFT_WALL:
            {
                feedback_.current_phase = "Aligning with wall";

                if (isParallelToLeftWall())
                {
                    // We're parallel, move to next phase
                    stopRobot();
                    current_phase_ = MOVE_UNTIL_LEFT_FRONT_CLEAR;
                    ROS_INFO("Aligned with wall");
                }
                else
                {
                    // Not parallel yet, make small adjustments
                    double front_left = getLeftSideDistance(-15.0);
                    double back_left = getLeftSideDistance(15.0);

                    if (front_left > back_left)
                    {
                        // Front is farther from wall than back, need to turn right slightly
                        rotateRight(angular_speed_ * 0.3); // Slower rotation for fine adjustment
                        ROS_INFO("Aligning: turning right slightly");
                    }
                    else
                    {
                        // Back is farther from wall than front, need to turn left slightly
                        rotateLeft(angular_speed_ * 0.3); // Slower rotation for fine adjustment
                        ROS_INFO("Aligning: turning left slightly");
                    }
                }
                break;
            }

            case ALIGN_WITH_RIGHT_WALL:
            {
                feedback_.current_phase = "Aligning with right wall";

                if (isParallelToRightWall())
                {
                    // We're parallel, move to next phase
                    stopRobot();
                    current_phase_ = MOVE_UNTIL_RIGHT_FRONT_CLEAR;
                    ROS_INFO("Aligned with right wall");
                }
                else
                {
                    // Not parallel yet, make small adjustments
                    double front_right = getRightSideDistance(-15.0);
                    double back_right = getRightSideDistance(15.0);

                    if (front_right > back_right)
                    {
                        // Front is farther from wall than back, need to turn left slightly
                        rotateLeft(angular_speed_ * 0.3); // Slower rotation for fine adjustment
                        ROS_INFO("Aligning: turning left slightly");
                    }
                    else
                    {
                        // Back is farther from wall than front, need to turn right slightly
                        rotateRight(angular_speed_ * 0.3); // Slower rotation for fine adjustment
                        ROS_INFO("Aligning: turning right slightly");
                    }
                }
                break;
            }

            case MOVE_UNTIL_RIGHT_FRONT_CLEAR:
            {
                feedback_.current_phase = "Moving until right front is clear";
                double right_front_dist = getRightFrontDistance();

                ROS_INFO_THROTTLE(0.5, "Phase: Move until right front clear, distance: %.2f (threshold: %.2f)",
                                  right_front_dist, wall_side_distance_threshold_);

                if (right_front_dist > wall_side_distance_threshold_)
                {
                    // Right front is clear, stop and transition to next phase
                    stopRobot();
                    current_phase_ = TURN_RIGHT_180;
                    turn_start_time_ = ros::Time::now();
                    current_angle_ = 0;
                    ROS_INFO("Right front is clear. Starting right turn 180.");
                }
                else
                {
                    // Move forward until right front is clear
                    moveForward();
                }
                break;
            }

            case TURN_RIGHT_180:
            {
                feedback_.current_phase = "Turning right 180 degrees with radius";

                // Set your desired radius directly in meters
                double desired_radius = desired_radius_; // 0.5 meters radius
                double linear_speed = linear_speed_;     // Lower for more control

                // Calculate path length for a 180° turn with this radius
                double arc_length = M_PI * desired_radius;

                // Time to travel this arc = distance / speed
                double expected_turn_time = arc_length / linear_speed;

                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Radius turn: elapsed=%.2f/%.2f, radius=%.2f m, arc=%.2f m",
                                  elapsed, expected_turn_time, desired_radius, arc_length);

                if (elapsed >= expected_turn_time)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = MOVE_UNTIL_LEFT_FRONT_CLEAR;
                    ROS_INFO("Radius right turn complete. Moving until left front is clear.");
                }
                else
                {
                    // Continue radius turn - true = turn right, with specific radius
                    turnWithRadius(true, desired_radius, linear_speed);
                }
                break;
            }

            case MOVE_UNTIL_LEFT_FRONT_CLEAR:
            {
                feedback_.current_phase = "Moving until left front is clear";
                double left_front_dist = getLeftFrontDistance();

                ROS_INFO_THROTTLE(0.5, "Phase: Move until left front clear, distance: %.2f (threshold: %.2f)",
                                  left_front_dist, wall_side_distance_threshold_);

                if (left_front_dist > wall_side_distance_threshold_)
                {
                    // Left front is clear, stop and transition to next phase
                    stopRobot();
                    current_phase_ = TURN_LEFT_LAST;
                    turn_start_time_ = ros::Time::now();
                    current_angle_ = 0;
                    ROS_INFO("Left front is clear. Starting left turn 180.");
                }
                else
                {
                    // Move forward until left front is clear
                    moveForward();
                }
                break;
            }

            case TURN_LEFT_LAST:
            {
                feedback_.current_phase = "Turning left 90 degrees";

                // Simple timed turn - 90 degrees
                double turn_duration = 97.0 / (angular_speed_ * (180.0 / M_PI));
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Phase: Turn left 90, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = COMPLETED;
                    ROS_INFO("Left turn complete. Starting left turn 180.");
                }
                else
                {
                    // Continue turning
                    rotateLeft(angular_speed_);
                }
                break;
            }

            case TURN_LEFT_180:
            {
                feedback_.current_phase = "Turning left 180 degrees with radius";

                // Set your desired radius directly in meters
                double desired_radius = desired_radius_; // 0.5 meters radius
                double linear_speed = linear_speed_;     // Lower for more control

                // Calculate path length for a 180° turn with this radius
                double arc_length = M_PI * desired_radius;

                // Time to travel this arc = distance / speed
                double expected_turn_time = arc_length / linear_speed;

                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Radius turn: elapsed=%.2f/%.2f, radius=%.2f m, arc=%.2f m",
                                  elapsed, expected_turn_time, desired_radius, arc_length);

                if (elapsed >= expected_turn_time)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = COMPLETED;
                    ROS_INFO("Radius right turn complete. Mission completed.");
                }
                else
                {
                    // Continue radius turn - false = turn left, with specific radius
                    turnWithRadius(false, desired_radius, linear_speed);
                }
                break;
            }

            case COMPLETED:
                // This should never execute, but just in case
                break;
            }

            // Calculate time elapsed for feedback
            feedback_.time_elapsed = (ros::Time::now() - phase_start_time_).toSec();

            // Publish feedback
            as_.publishFeedback(feedback_);

            r.sleep();
        }

        if (success)
        {
            result_.success = true;
            ROS_INFO("%s: Construction mission succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "construction_server");
    ConstructionServer constructionServer("construction");
    ros::spin();
    return 0;
}