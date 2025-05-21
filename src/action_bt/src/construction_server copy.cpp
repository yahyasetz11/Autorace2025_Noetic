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

    // Algorithm phases for rounded mode
    enum RoundedPhase
    {
        MOVE_TO_WALL,
        TURN_LEFT_90,
        TURN_LEFT_45,
        MOVE_FORWARD_1,
        TURN_RIGHT_90,
        MOVE_FORWARD_2,
        TURN_LEFT_90_2,
        TURN_LEFT_LAST,
        TURN_LEFT_LAST_2,
        ALIGN_WITH_LEFT_WALL,
        ALIGN_WITH_RIGHT_WALL,
        MOVE_UNTIL_RIGHT_FRONT_CLEAR,
        TURN_RIGHT_180,
        MOVE_UNTIL_LEFT_FRONT_CLEAR,
        TURN_LEFT_180,
        COMPLETED
    };

    // Algorithm phases for ramp mode
    enum RampPhase
    {
        RAMP_MOVE_TO_WALL,
        RAMP_TURN_LEFT_PIVOT,
        RAMP_MOVE_UNTIL_300_CLEAR,
        RAMP_TURN_RIGHT_PIVOT_1,
        RAMP_MOVE_UNTIL_270_CLEAR,
        RAMP_TURN_RIGHT_PIVOT_2,
        RAMP_MOVE_UNTIL_60_CLEAR,
        RAMP_TURN_LEFT_PIVOT_FINAL,
        RAMP_COMPLETED
    };

    double wheel_base_;
    bool flag = false;

    RoundedPhase current_rounded_phase_;
    RampPhase current_ramp_phase_;

    // Mode selection
    std::string mode_; // "rounded" or "ramp"

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

    // Ramp mode parameters
    double ramp_rotation_angle_;
    double ramp_front_threshold_;
    double ramp_side_threshold_;
    double ramp_final_threshold_;
    double pivot_angular_speed_;

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
                                           current_rounded_phase_(MOVE_TO_WALL),
                                           current_ramp_phase_(RAMP_MOVE_TO_WALL),
                                           current_angle_(0.0),
                                           mode_("rounded") // Default mode
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

        ROS_INFO("Ramp mode parameters:");
        ROS_INFO("  ramp_rotation_angle: %.2f degrees", ramp_rotation_angle_);
        ROS_INFO("  ramp_front_threshold: %.2f m", ramp_front_threshold_);
        ROS_INFO("  ramp_side_threshold: %.2f m", ramp_side_threshold_);
        ROS_INFO("  ramp_final_threshold: %.2f m", ramp_final_threshold_);
        ROS_INFO("  pivot_angular_speed: %.2f rad/s", pivot_angular_speed_);
    }

    ~ConstructionServer()
    {
        // Stop robot when server shuts down
        stopRobot();
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

                // Load ramp mode parameters
                ramp_rotation_angle_ = construction["ramp_rotation_angle"] ? construction["ramp_rotation_angle"].as<double>() : 60.0;
                ramp_front_threshold_ = construction["ramp_front_threshold"] ? construction["ramp_front_threshold"].as<double>() : 0.3;
                ramp_side_threshold_ = construction["ramp_side_threshold"] ? construction["ramp_side_threshold"].as<double>() : 0.35;
                ramp_final_threshold_ = construction["ramp_final_threshold"] ? construction["ramp_final_threshold"].as<double>() : 0.30;
                pivot_angular_speed_ = construction["pivot_angular_speed"] ? construction["pivot_angular_speed"].as<double>() : 0.5;

                // Load robot parameters
                wheel_base_ = construction["wheel_base"] ? construction["wheel_base"].as<double>() : 0.160;
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

                // Default ramp parameters
                ramp_rotation_angle_ = 60.0;
                ramp_front_threshold_ = 0.3;
                ramp_side_threshold_ = 0.35;
                ramp_final_threshold_ = 0.30;
                pivot_angular_speed_ = 0.5;

                // Default robot parameters
                wheel_base_ = 0.160;
            }

            ROS_INFO("Successfully loaded construction parameters from: %s", construction_config_path_.c_str());
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

            // Default ramp parameters
            ramp_rotation_angle_ = 60.0;
            ramp_front_threshold_ = 0.3;
            ramp_side_threshold_ = 0.35;
            ramp_final_threshold_ = 0.30;
            pivot_angular_speed_ = 0.5;

            // Default robot parameters
            wheel_base_ = 0.160;
        }
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

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
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
        for (double angle = -front_scan_angle_ - 5; angle <= front_scan_angle_ - 5; angle += 1.0)
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

    // New function for pivot turning (one wheel stationary)
    // New function for pivot turning (one wheel stationary)
    void pivotTurn(bool pivotLeft, double angle_degrees)
    {
        geometry_msgs::Twist cmd;

        // Convert angle to radians
        double angle_rad = angle_degrees * M_PI / 180.0;

        // For a true pivot turn, we need both linear and angular velocity
        // The relationship is: v = ω * r, where r is the wheel base for pivot turns

        if (pivotLeft)
        {
            // Left pivot: left wheel stationary, right wheel moves forward
            // Robot pivots around the left wheel
            // Linear velocity = angular velocity * (wheel_base/2)
            cmd.angular.z = pivot_angular_speed_;
            cmd.linear.x = pivot_angular_speed_ * (wheel_base_ / 2.0);
        }
        else
        {
            // Right pivot: right wheel stationary, left wheel moves forward
            // Robot pivots around the right wheel
            // Linear velocity = angular velocity * (wheel_base/2)
            cmd.angular.z = -pivot_angular_speed_;
            cmd.linear.x = pivot_angular_speed_ * (wheel_base_ / 2.0);
        }

        // Calculate time needed for the turn
        double turn_time = fabs(angle_rad / pivot_angular_speed_);

        cmd_vel_pub_.publish(cmd);

        ROS_INFO("Pivot turn: %s pivot, %.1f degrees, linear=%.3f, angular=%.2f, time=%.2f",
                 pivotLeft ? "left" : "right", angle_degrees, cmd.linear.x, cmd.angular.z, turn_time);
    }

    void executeRoundedMode()
    {
        ros::Rate r(10);
        bool success = true;

        while (ros::ok() && current_rounded_phase_ != COMPLETED)
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
            switch (current_rounded_phase_)
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
                    current_rounded_phase_ = TURN_LEFT_45;
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

            case TURN_LEFT_45:
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
                    current_rounded_phase_ = MOVE_FORWARD_1;
                    ROS_INFO("Left turn complete. Aligning with the Right Wall.");
                }
                else
                {
                    // Continue turning
                    rotateLeft(angular_speed_);
                }
                break;
            }

            case MOVE_FORWARD_1:
            {
                feedback_.current_phase = "Ramp: Moving until 300° clear";
                // Check at 300° (360-60)
                double dist_300 = getDistanceInRange(360.0 - ramp_rotation_angle_);

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move until 300° clear, distance: %.2f (threshold: %.2f)",
                                  dist_300, ramp_side_threshold_);

                if (dist_300 > ramp_side_threshold_)
                {
                    stopRobot();
                    current_rounded_phase_ = TURN_RIGHT_90;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("300° clear. Starting first right pivot turn.");
                }
                else
                {
                    moveForward();
                }
                break;
            }

            case TURN_RIGHT_90:
            {
                feedback_.current_phase = "Turning right 180 degrees with radius";

                // Set your desired radius directly in meters
                double desired_radius = desired_radius_; // 0.5 meters radius
                double linear_speed = linear_speed_;     // Lower for more control

                // Calculate path length for a 180° turn with this radius
                double arc_length = M_PI / 2 * desired_radius;

                // Time to travel this arc = distance / speed
                double expected_turn_time = arc_length / linear_speed;

                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Radius turn: elapsed=%.2f/%.2f, radius=%.2f m, arc=%.2f m",
                                  elapsed, expected_turn_time, desired_radius, arc_length);

                if (elapsed >= expected_turn_time)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_rounded_phase_ = MOVE_FORWARD_2;
                    ROS_INFO("Radius right turn complete. Moving until left front is clear.");
                }
                else
                {
                    // Continue radius turn - true = turn right, with specific radius
                    turnWithRadius(true, desired_radius, linear_speed);
                }
                break;
            }

            case MOVE_FORWARD_2:
            {
                feedback_.current_phase = "Ramp: Moving until 300° clear";
                // Check at 300° (360-60)
                double dist_300 = getDistanceInRange(360.0 - ramp_rotation_angle_);

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move until 300° clear, distance: %.2f (threshold: %.2f)",
                                  dist_300, ramp_side_threshold_);

                if (dist_300 > ramp_side_threshold_)
                {
                    stopRobot();
                    current_rounded_phase_ = TURN_RIGHT_90;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("300° clear. Starting first right pivot turn.");
                }
                else
                {
                    moveForward();
                }
                break;
            }

            case TURN_LEFT_90_2:
            {
                feedback_.current_phase = "Turning left 180 degrees with radius";

                // Set your desired radius directly in meters
                double desired_radius = 0.2; // 0.5 meters radius
                double linear_speed = 0.08;  // Lower for more control

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
                    current_rounded_phase_ = COMPLETED;
                    ROS_INFO("Radius right turn complete. Mission completed.");
                }
                else
                {
                    // Continue radius turn - false = turn left, with specific radius
                    turnWithRadius(false, desired_radius, linear_speed);
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
                    current_rounded_phase_ = ALIGN_WITH_RIGHT_WALL;
                    ROS_INFO("Left turn complete. Aligning with the Right Wall.");
                }
                else
                {
                    // Continue turning
                    rotateLeft(angular_speed_);
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
                    current_rounded_phase_ = MOVE_UNTIL_LEFT_FRONT_CLEAR;
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
                    current_rounded_phase_ = MOVE_UNTIL_RIGHT_FRONT_CLEAR;
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
                    current_rounded_phase_ = TURN_RIGHT_180;
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
                double arc_length = M_PI / 2 * desired_radius;

                // Time to travel this arc = distance / speed
                double expected_turn_time = arc_length / linear_speed;

                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Radius turn: elapsed=%.2f/%.2f, radius=%.2f m, arc=%.2f m",
                                  elapsed, expected_turn_time, desired_radius, arc_length);

                if (elapsed >= expected_turn_time)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_rounded_phase_ = MOVE_UNTIL_LEFT_FRONT_CLEAR;
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
                    // Left front is80 clear, stop and transition to next phase
                    stopRobot();
                    current_rounded_phase_ = TURN_LEFT_LAST;
                    // current_rounded_phase_ = TURN_LEFT_LAST;
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
                    current_rounded_phase_ = TURN_LEFT_LAST_2;
                    ROS_INFO("Left turn complete. Starting left turn 180.");
                }
                else
                {
                    // Continue turning
                    rotateLeft(angular_speed_);
                }
                break;
            }

            case TURN_LEFT_LAST_2:
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
                    current_rounded_phase_ = COMPLETED;
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
                double desired_radius = 0.2; // 0.5 meters radius
                double linear_speed = 0.08;  // Lower for more control

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
                    current_rounded_phase_ = COMPLETED;
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
    }

    void executeRampMode()
    {
        ros::Rate r(10);
        bool success = true;

        while (ros::ok() && current_ramp_phase_ != RAMP_COMPLETED)
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
            switch (current_ramp_phase_)
            {
            case RAMP_MOVE_TO_WALL:
            {
                feedback_.current_phase = "Ramp: Moving to wall";
                double front_dist = getDistanceInRange(0.0); // 0 degree

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move to wall, front distance: %.2f (threshold: %.2f)",
                                  front_dist, ramp_front_threshold_);

                if (front_dist <= ramp_front_threshold_)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_TURN_LEFT_PIVOT;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("Reached wall. Starting left pivot turn.");
                }
                else
                {
                    moveForward();
                }
                break;
            }

            case RAMP_TURN_LEFT_PIVOT:
            {
                feedback_.current_phase = "Ramp: Left pivot turn 60 degrees";

                double angle_rad = ramp_rotation_angle_ * M_PI / 180.0;
                double turn_duration = fabs(angle_rad / pivot_angular_speed_);
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Left pivot turn, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_MOVE_UNTIL_300_CLEAR;
                    ROS_INFO("Left pivot complete. Moving until 300° clear.");
                }
                else
                {
                    pivotTurn(true, ramp_rotation_angle_);
                }
                break;
            }

            case RAMP_MOVE_UNTIL_300_CLEAR:
            {
                feedback_.current_phase = "Ramp: Moving until 300° clear";
                // Check at 300° (360-60)
                double dist_300 = getDistanceInRange(360.0 - ramp_rotation_angle_);

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move until 300° clear, distance: %.2f (threshold: %.2f)",
                                  dist_300, ramp_side_threshold_);

                if (dist_300 > ramp_side_threshold_)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_TURN_RIGHT_PIVOT_1;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("300° clear. Starting first right pivot turn.");
                }
                else
                {
                    moveForward();
                }
                break;
            }

            case RAMP_TURN_RIGHT_PIVOT_1:
            {
                feedback_.current_phase = "Ramp: First right pivot turn 60 degrees";

                double angle_rad = ramp_rotation_angle_ * M_PI / 180.0;
                double turn_duration = fabs(angle_rad / pivot_angular_speed_);
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: First right pivot turn, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_MOVE_UNTIL_270_CLEAR;
                    ROS_INFO("First right pivot complete. Moving until 270° clear.");
                }
                else
                {
                    pivotTurn(false, ramp_rotation_angle_);
                }
                break;
            }

            case RAMP_MOVE_UNTIL_270_CLEAR:
            {
                feedback_.current_phase = "Ramp: Moving until 270° clear";
                // Check at 270° (360-90)
                double dist_270 = getDistanceInRange(270.0);

                if (flag && dist_270 > ramp_side_threshold_)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_TURN_RIGHT_PIVOT_2;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("270° clear. Starting second right pivot turn.");
                }
                else
                {
                    moveForward();
                    if (!flag && dist_270 > ramp_side_threshold_)
                    {
                        flag = false;
                        ROS_INFO("Flag = false");
                    }
                    else
                    {
                        flag = true;
                        ROS_INFO("Flag obstacle detected");
                    }
                }

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move until 270° clear, distance: %.2f (threshold: %.2f)",
                                  dist_270, ramp_side_threshold_);
                break;
            }

            case RAMP_TURN_RIGHT_PIVOT_2:
            {
                feedback_.current_phase = "Ramp: Second right pivot turn 60 degrees";

                double angle_rad = ramp_rotation_angle_ * M_PI / 180.0;
                double turn_duration = fabs(angle_rad / pivot_angular_speed_);
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Second right pivot turn, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_MOVE_UNTIL_60_CLEAR;
                    ROS_INFO("Second right pivot complete. Moving until 60° clear.");
                }
                else
                {
                    pivotTurn(false, ramp_rotation_angle_);
                }
                break;
            }

            case RAMP_MOVE_UNTIL_60_CLEAR:
            {
                feedback_.current_phase = "Ramp: Moving until 60° clear";
                // Check at 60°
                double dist_60 = getDistanceInRange(ramp_rotation_angle_);

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Move until 60° clear, distance: %.2f (threshold: %.2f)",
                                  dist_60, ramp_side_threshold_);

                if (dist_60 > ramp_side_threshold_)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_TURN_LEFT_PIVOT_FINAL;
                    turn_start_time_ = ros::Time::now();
                    ROS_INFO("60° clear. Starting final left pivot turn.");
                }
                else
                {
                    moveForward();
                }
                break;
            }

            case RAMP_TURN_LEFT_PIVOT_FINAL:
            {
                feedback_.current_phase = "Ramp: Final left pivot turn 60 degrees";

                double angle_rad = ramp_rotation_angle_ * M_PI / 180.0;
                double turn_duration = fabs(angle_rad / pivot_angular_speed_);
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Ramp Phase: Final left pivot turn, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    stopRobot();
                    current_ramp_phase_ = RAMP_COMPLETED;
                    ROS_INFO("Final pivot complete. Ramp mission completed.");
                }
                else
                {
                    pivotTurn(true, ramp_rotation_angle_);
                }
                break;
            }

            case RAMP_COMPLETED:
                // This should never execute, but just in case
                break;
            }

            // Calculate time elapsed for feedback
            feedback_.time_elapsed = (ros::Time::now() - phase_start_time_).toSec();

            // Publish feedback
            as_.publishFeedback(feedback_);

            r.sleep();
        }
    }

    void executeCB(const msg_file::ConstructionGoalConstPtr &goal)
    {
        bool success = true;
        phase_start_time_ = ros::Time::now();

        // Get mode from goal (default to "rounded" if not specified)
        mode_ = goal->mode;
        if (mode_.empty())
        {
            mode_ = "rounded";
        }

        // Reset phase counters
        current_rounded_phase_ = MOVE_TO_WALL;
        current_ramp_phase_ = RAMP_MOVE_TO_WALL;

        ROS_INFO("Starting Construction Mission with mode: %s", mode_.c_str());

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
            ros::Duration(0.1).sleep();
        }

        // Execute the appropriate mode
        if (mode_ == "rounded")
        {
            executeRoundedMode();
        }
        else if (mode_ == "ramp")
        {
            executeRampMode();
        }
        else
        {
            ROS_ERROR("Unknown mode: %s. Aborting mission.", mode_.c_str());
            as_.setAborted();
            return;
        }

        // Check if mission completed successfully
        if ((mode_ == "rounded" && current_rounded_phase_ == COMPLETED) ||
            (mode_ == "ramp" && current_ramp_phase_ == RAMP_COMPLETED))
        {
            result_.success = true;
            ROS_INFO("%s: Construction mission succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        }
        else
        {
            result_.success = false;
            ROS_INFO("%s: Construction mission failed", action_name_.c_str());
            as_.setAborted(result_);
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