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
    double front_scan_angle_;
    double side_scan_angle_;
    double turn_precision_threshold_;

    // Turn tracking
    double turn_target_angle_;
    double current_angle_;
    ros::Time turn_start_time_;
    ros::Time phase_start_time_;

    // Config file path
    std::string config_path_;

public:
    ConstructionServer(std::string name) : as_(nh_, name, boost::bind(&ConstructionServer::executeCB, this, _1), false),
                                           action_name_(name),
                                           scan_received_(false),
                                           current_phase_(MOVE_TO_WALL),
                                           current_angle_(0.0)
    {
        // Get config file path from parameter server or use default
        nh_.param<std::string>("construction_config_path", config_path_,
                               ros::package::getPath("action_bt") + "/config/construction_params.yaml");

        // Load parameters directly from YAML file
        loadParameters();

        // Publishers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Subscribers
        laser_sub_ = nh_.subscribe("/scan", 1, &ConstructionServer::laserCallback, this);

        as_.start();
        ROS_INFO("Construction Action Server Started");
        ROS_INFO("Parameters: linear_speed=%.2f, angular_speed=%.2f, wall_dist=%.2f",
                 linear_speed_, angular_speed_, wall_distance_threshold_);
    }

    ~ConstructionServer()
    {
        // Stop robot when server shuts down
        stopRobot();
    }

    void loadParameters()
    {
        try
        {
            ROS_INFO("Loading parameters from: %s", config_path_.c_str());
            YAML::Node config = YAML::LoadFile(config_path_);

            // Extract parameters with defaults if not found
            if (config["construction"])
            {
                YAML::Node construction = config["construction"];

                linear_speed_ = construction["linear_speed"] ? construction["linear_speed"].as<double>() : 0.1;

                angular_speed_ = construction["angular_speed"] ? construction["angular_speed"].as<double>() : 0.5;

                wall_distance_threshold_ = construction["wall_distance_threshold"] ? construction["wall_distance_threshold"].as<double>() : 0.3;

                front_scan_angle_ = construction["front_scan_angle"] ? construction["front_scan_angle"].as<double>() : 10.0;

                side_scan_angle_ = construction["side_scan_angle"] ? construction["side_scan_angle"].as<double>() : 30.0;

                turn_precision_threshold_ = construction["turn_precision_threshold"] ? construction["turn_precision_threshold"].as<double>() : 0.1;
            }
            else
            {
                ROS_WARN("Construction section not found in config file. Using defaults.");
                linear_speed_ = 0.1;
                angular_speed_ = 0.5;
                wall_distance_threshold_ = 0.3;
                front_scan_angle_ = 10.0;
                side_scan_angle_ = 30.0;
                turn_precision_threshold_ = 0.1;
            }

            ROS_INFO("Parameters loaded: linear_speed=%.2f, angular_speed=%.2f, wall_dist=%.2f",
                     linear_speed_, angular_speed_, wall_distance_threshold_);
        }
        catch (const YAML::Exception &e)
        {
            ROS_ERROR("Error loading YAML file: %s", e.what());
            ROS_WARN("Using default parameters");
            linear_speed_ = 0.1;
            angular_speed_ = 0.5;
            wall_distance_threshold_ = 0.3;
            front_scan_angle_ = 10.0;
            side_scan_angle_ = 30.0;
            turn_precision_threshold_ = 0.1;
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

    double getRightFrontDistance()
    {
        double min_dist = latest_scan_.range_max;

        // Check ranges in right front area
        for (double angle = -side_scan_angle_; angle <= -10.0; angle += 1.0)
        {
            double dist = getDistanceInRange(angle);
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
        for (double angle = 10.0; angle <= side_scan_angle_; angle += 1.0)
        {
            double dist = getDistanceInRange(angle);
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

    void turnLeft(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed;
        cmd_vel_pub_.publish(cmd);
    }

    void turnRight(double angular_speed)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = -angular_speed;
        cmd_vel_pub_.publish(cmd);
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
                double turn_duration = 90.0 / (angular_speed_ * (180.0 / M_PI));
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Phase: Turn left 90, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = MOVE_UNTIL_RIGHT_FRONT_CLEAR;
                    ROS_INFO("Left turn complete. Moving until right front is clear.");
                }
                else
                {
                    // Continue turning
                    turnLeft(angular_speed_);
                }
                break;
            }

            case MOVE_UNTIL_RIGHT_FRONT_CLEAR:
            {
                feedback_.current_phase = "Moving until right front is clear";
                double right_front_dist = getRightFrontDistance();

                ROS_INFO_THROTTLE(0.5, "Phase: Move until right front clear, distance: %.2f (threshold: %.2f)",
                                  right_front_dist, wall_distance_threshold_ * 2);

                if (right_front_dist > wall_distance_threshold_ * 2)
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
                feedback_.current_phase = "Turning right 180 degrees";

                // Simple timed turn - 180 degrees
                double turn_duration = 180.0 / (angular_speed_ * (180.0 / M_PI));
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Phase: Turn right 180, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    // Turn complete, stop and transition to next phase
                    stopRobot();
                    current_phase_ = MOVE_UNTIL_LEFT_FRONT_CLEAR;
                    ROS_INFO("Right turn complete. Moving until left front is clear.");
                }
                else
                {
                    // Continue turning
                    turnRight(angular_speed_);
                }
                break;
            }

            case MOVE_UNTIL_LEFT_FRONT_CLEAR:
            {
                feedback_.current_phase = "Moving until left front is clear";
                double left_front_dist = getLeftFrontDistance();

                ROS_INFO_THROTTLE(0.5, "Phase: Move until left front clear, distance: %.2f (threshold: %.2f)",
                                  left_front_dist, wall_distance_threshold_ * 2);

                if (left_front_dist > wall_distance_threshold_ * 2)
                {
                    // Left front is clear, stop and transition to next phase
                    stopRobot();
                    current_phase_ = TURN_LEFT_180;
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

            case TURN_LEFT_180:
            {
                feedback_.current_phase = "Turning left 180 degrees";

                // Simple timed turn - 180 degrees
                double turn_duration = 180.0 / (angular_speed_ * (180.0 / M_PI));
                double elapsed = (ros::Time::now() - turn_start_time_).toSec();

                ROS_INFO_THROTTLE(0.5, "Phase: Turn left 180, elapsed: %.2f, target: %.2f",
                                  elapsed, turn_duration);

                if (elapsed >= turn_duration)
                {
                    // Turn complete, mission completed
                    stopRobot();
                    current_phase_ = COMPLETED;
                    ROS_INFO("Left turn complete. Construction mission completed.");
                }
                else
                {
                    // Continue turning
                    turnLeft(angular_speed_);
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