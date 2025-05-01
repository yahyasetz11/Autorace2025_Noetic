#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/TunnelNavAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::TunnelNavResultConstPtr &result)
{
    ROS_INFO("Action completed with status: %s", state.toString().c_str());
    ROS_INFO("Result: %s", result->success ? "success" : "failure");

    if (result->success && !result->map_path.empty())
    {
        ROS_INFO("Map saved to: %s", result->map_path.c_str());
    }

    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal activated");
}

void feedbackCb(const msg_file::TunnelNavFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: state=%s, distance to goal=%.2f",
             feedback->current_state.c_str(),
             feedback->distance_to_goal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tunnel_nav_client_test");
    ros::NodeHandle nh;

    // Default values
    std::string mode = "online";
    std::string map_file = "";

    // Process command line arguments
    for (int i = 1; i < argc; i++)
    {
        std::string arg(argv[i]);

        if (arg == "--mode" && i + 1 < argc)
        {
            mode = argv[++i];
        }
        else if (arg == "--map" && i + 1 < argc)
        {
            map_file = argv[++i];
        }
        else if (arg == "--help")
        {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --mode <mode>    SLAM mode: 'online' or 'offline'" << std::endl;
            std::cout << "  --map <file>     Map file path (for offline mode)" << std::endl;
            std::cout << "  --help           Show this help message" << std::endl;
            return 0;
        }
    }

    // Check mode validity
    if (mode != "online" && mode != "offline")
    {
        ROS_ERROR("Invalid mode: %s. Must be 'online' or 'offline'", mode.c_str());
        return 1;
    }

    // For offline mode, verify map file
    if (mode == "offline" && map_file.empty())
    {
        ROS_ERROR("For offline mode, a map file must be specified");
        return 1;
    }

    // Create action client
    actionlib::SimpleActionClient<msg_file::TunnelNavAction> client("tunnel_nav", true);

    ROS_INFO("Waiting for server...");
    client.waitForServer();
    ROS_INFO("Server connected!");

    // Set goal
    msg_file::TunnelNavGoal goal;
    goal.mode = mode;
    goal.map_file = map_file;

    ROS_INFO("Sending goal...");
    ROS_INFO("  Mode: %s", mode.c_str());
    if (mode == "offline")
    {
        ROS_INFO("  Map file: %s", map_file.c_str());
    }

    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}