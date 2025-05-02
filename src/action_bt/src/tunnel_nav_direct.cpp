#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class TunnelNavDirect
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_; // For parameters

    // ROS communication
    ros::Subscriber laser_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher initial_pose_pub_;
    ros::Publisher goal_pub_; // Publisher for move_base_simple/goal
    tf::TransformListener tf_listener_;

    // State management
    enum State
    {
        MOVING_TO_WALL,
        STOPPED_AT_WALL,
        WAITING_FOR_MAP,
        SENT_INITIAL_POSE,
        SENT_GOAL,
        NAVIGATING,
        REACHED_GOAL,
        FINISHED
    };
    State current_state_;

    // Configuration
    double wall_distance_threshold_;
    int wall_detection_angle_;
    double initial_linear_speed_;
    double goal_x_, goal_y_, goal_yaw_;

    // State variables
    bool wall_detected_;
    bool map_available_;
    bool goal_reached_;
    ros::Time state_start_time_;
    geometry_msgs::PoseStamped goal_pose_;

public:
    TunnelNavDirect() : private_nh_("~"), current_state_(MOVING_TO_WALL),
                        wall_detected_(false), map_available_(false), goal_reached_(false)
    {
        // Load parameters
        private_nh_.param<double>("wall_distance_threshold", wall_distance_threshold_, 0.5);
        private_nh_.param<int>("wall_detection_angle", wall_detection_angle_, 270);
        private_nh_.param<double>("initial_linear_speed", initial_linear_speed_, 0.2);
        private_nh_.param<double>("goal_x", goal_x_, 2.0);
        private_nh_.param<double>("goal_y", goal_y_, 0.0);
        private_nh_.param<double>("goal_yaw", goal_yaw_, 0.0);

        // Initialize publishers and subscribers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        laser_sub_ = nh_.subscribe("/scan", 1, &TunnelNavDirect::laserCallback, this);

        state_start_time_ = ros::Time::now();

        ROS_INFO("Tunnel Navigation Direct started");
        ROS_INFO("Configured parameters:");
        ROS_INFO("  wall_distance_threshold: %.2f", wall_distance_threshold_);
        ROS_INFO("  wall_detection_angle: %d", wall_detection_angle_);
        ROS_INFO("  initial_linear_speed: %.2f", initial_linear_speed_);
        ROS_INFO("  goal position: (%.2f, %.2f, %.2f)", goal_x_, goal_y_, goal_yaw_);
    }

    ~TunnelNavDirect()
    {
        // Stop robot on shutdown
        stopRobot();
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        // Only process if in the correct state
        if (current_state_ != MOVING_TO_WALL)
            return;

        // Calculate the index for the specified angle
        double angle_rad = wall_detection_angle_ * M_PI / 180.0;
        int detection_idx = (angle_rad - scan->angle_min) / scan->angle_increment;

        // Ensure the index is valid
        if (detection_idx >= 0 && detection_idx < scan->ranges.size())
        {
            float distance = scan->ranges[detection_idx];

            // Check if the distance is valid and less than threshold
            if (!std::isnan(distance) && !std::isinf(distance) && distance <= wall_distance_threshold_)
            {
                wall_detected_ = true;
                ROS_INFO("Wall detected at angle %d degrees at distance: %.2f m",
                         wall_detection_angle_, distance);
            }
        }
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
        cmd.linear.x = initial_linear_speed_;
        cmd.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd);
    }

    bool isMapAvailable()
    {
        // Check if map->odom transform is available (indicating SLAM is running)
        try
        {
            tf::StampedTransform transform;
            if (tf_listener_.waitForTransform("map", "odom", ros::Time(0), ros::Duration(1.0)))
            {
                return true;
            }
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5.0, "TF exception: %s", ex.what());
        }
        return false;
    }

    void setInitialPose()
    {
        ROS_INFO("Setting initial pose");

        // Create initial pose message
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.header.frame_id = "map";

        // Set position to origin
        initial_pose.pose.pose.position.x = 0.0;
        initial_pose.pose.pose.position.y = 0.0;
        initial_pose.pose.pose.position.z = 0.0;

        // Set orientation to identity (forward)
        initial_pose.pose.pose.orientation.w = 1.0;
        initial_pose.pose.pose.orientation.x = 0.0;
        initial_pose.pose.pose.orientation.y = 0.0;
        initial_pose.pose.pose.orientation.z = 0.0;

        // Set covariance (low uncertainty)
        for (int i = 0; i < 36; i++)
        {
            initial_pose.pose.covariance[i] = 0.0;
        }
        initial_pose.pose.covariance[0] = 0.25;  // x variance
        initial_pose.pose.covariance[7] = 0.25;  // y variance
        initial_pose.pose.covariance[35] = 0.06; // yaw variance

        // Publish initial pose
        initial_pose_pub_.publish(initial_pose);

        ROS_INFO("Initial pose published");
    }

    void sendGoalPose()
    {
        // Create goal pose message
        goal_pose_.header.frame_id = "map";
        goal_pose_.header.stamp = ros::Time::now();
        goal_pose_.pose.position.x = goal_x_;
        goal_pose_.pose.position.y = goal_y_;
        goal_pose_.pose.position.z = 0.0;

        // Convert yaw to quaternion
        tf::Quaternion q;
        q.setRPY(0, 0, goal_yaw_);
        goal_pose_.pose.orientation.x = q.x();
        goal_pose_.pose.orientation.y = q.y();
        goal_pose_.pose.orientation.z = q.z();
        goal_pose_.pose.orientation.w = q.w();

        // Publish goal pose
        goal_pub_.publish(goal_pose_);

        ROS_INFO("Navigation goal sent: x=%.2f, y=%.2f, yaw=%.2f",
                 goal_x_, goal_y_, goal_yaw_);
    }

    double getDistanceToGoal()
    {
        try
        {
            // Get the current robot position
            tf::StampedTransform transform;
            tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            // Calculate distance to goal
            double dx = goal_pose_.pose.position.x - transform.getOrigin().x();
            double dy = goal_pose_.pose.position.y - transform.getOrigin().y();
            double distance = std::sqrt(dx * dx + dy * dy);

            // Check if goal is reached (within 0.2m)
            if (distance < 0.2)
            {
                goal_reached_ = true;
                ROS_INFO("Goal reached! Distance: %.2f m", distance);
            }

            return distance;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(5.0, "Failed to get robot position: %s", ex.what());
            return -1.0;
        }
    }

    void executeStateMachine()
    {
        ros::Rate rate(10); // 10 Hz

        while (ros::ok() && current_state_ != FINISHED)
        {
            double time_in_state = (ros::Time::now() - state_start_time_).toSec();

            // State machine
            switch (current_state_)
            {
            case MOVING_TO_WALL:
                // Moving forward until wall detected
                if (wall_detected_)
                {
                    stopRobot();
                    current_state_ = STOPPED_AT_WALL;
                    state_start_time_ = ros::Time::now();
                    ROS_INFO("State transition: MOVING_TO_WALL -> STOPPED_AT_WALL");
                }
                else
                {
                    moveForward();
                }
                break;

            case STOPPED_AT_WALL:
                // Wait a moment for SLAM to initialize
                if (time_in_state > 2.0)
                {
                    current_state_ = WAITING_FOR_MAP;
                    state_start_time_ = ros::Time::now();
                    ROS_INFO("State transition: STOPPED_AT_WALL -> WAITING_FOR_MAP");
                }
                break;

            case WAITING_FOR_MAP:
                // Check if map is available
                if (isMapAvailable())
                {
                    map_available_ = true;
                    current_state_ = SENT_INITIAL_POSE;
                    state_start_time_ = ros::Time::now();
                    setInitialPose();
                    ROS_INFO("State transition: WAITING_FOR_MAP -> SENT_INITIAL_POSE");
                }
                else
                {
                    if (time_in_state > 30.0)
                    {
                        ROS_ERROR("Timed out waiting for map after 30 seconds");
                        current_state_ = FINISHED;
                    }
                    else
                    {
                        ROS_INFO_THROTTLE(5.0, "Waiting for map... (%.1f seconds)", time_in_state);
                    }
                }
                break;

            case SENT_INITIAL_POSE:
                // Wait for initial pose to propagate
                if (time_in_state > 2.0)
                {
                    current_state_ = SENT_GOAL;
                    state_start_time_ = ros::Time::now();
                    sendGoalPose();
                    ROS_INFO("State transition: SENT_INITIAL_POSE -> SENT_GOAL");
                }
                break;

            case SENT_GOAL:
                // Wait for move_base to process the goal
                if (time_in_state > 2.0)
                {
                    current_state_ = NAVIGATING;
                    state_start_time_ = ros::Time::now();
                    ROS_INFO("State transition: SENT_GOAL -> NAVIGATING");
                }
                break;

            case NAVIGATING:
                // Monitor progress
                {
                    double distance = getDistanceToGoal();
                    if (distance >= 0)
                    {
                        ROS_INFO_THROTTLE(2.0, "Distance to goal: %.2f m", distance);
                    }

                    if (goal_reached_)
                    {
                        current_state_ = REACHED_GOAL;
                        state_start_time_ = ros::Time::now();
                        ROS_INFO("State transition: NAVIGATING -> REACHED_GOAL");
                    }
                    else if (time_in_state > 300.0) // 5 minute timeout
                    {
                        ROS_ERROR("Navigation timed out after 300 seconds");
                        current_state_ = FINISHED;
                    }
                }
                break;

            case REACHED_GOAL:
                // Wait a moment at the goal
                if (time_in_state > 5.0)
                {
                    current_state_ = FINISHED;
                    ROS_INFO("State transition: REACHED_GOAL -> FINISHED");
                    ROS_INFO("Navigation completed successfully!");
                }
                break;

            case FINISHED:
                // Nothing to do
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tunnel_nav_direct");

    TunnelNavDirect tunnelNav;
    tunnelNav.executeStateMachine();

    return 0;
}