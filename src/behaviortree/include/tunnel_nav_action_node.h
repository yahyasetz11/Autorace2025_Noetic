#ifndef TUNNEL_NAV_ACTION_NODE_H
#define TUNNEL_NAV_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/TunnelNavAction.h>

class TunnelNavActionNode : public BT::SyncActionNode
{
public:
    TunnelNavActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("tunnel_nav", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for tunnel_nav action server...");
        client_.waitForServer();
        ROS_INFO("Tunnel navigation action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("mode", "online", "SLAM mode: online or offline"),
            BT::InputPort<std::string>("map_file", "", "Map file path (for offline mode, optional)"),
            BT::OutputPort<std::string>("map_path", "Path to the saved map (only for online mode)")};
    }

    BT::NodeStatus tick() override
    {
        // Get input parameters (only mode and map_file from behavior tree)
        std::string mode = "online";
        std::string map_file = "";

        getInput("mode", mode);
        getInput("map_file", map_file);

        // Create and send the goal
        msg_file::TunnelNavGoal goal;
        goal.mode = mode;
        goal.map_file = map_file;

        ROS_INFO("Sending tunnel navigation goal - Mode: %s", mode.c_str());
        if (mode == "offline")
        {
            ROS_INFO("Map file: %s", map_file.empty() ? "using default" : map_file.c_str());
        }

        client_.sendGoal(goal);

        // Wait for action to complete with a long timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(600.0)); // 10 minute timeout

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                // If successful, set the output port with the map path if available
                auto result = client_.getResult();
                if (!result->map_path.empty())
                {
                    setOutput("map_path", result->map_path);
                    ROS_INFO("Tunnel navigation succeeded. Map saved to: %s", result->map_path.c_str());
                }
                else
                {
                    ROS_INFO("Tunnel navigation succeeded");
                }
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Tunnel navigation failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Tunnel navigation timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::TunnelNavAction> client_;
};

#endif // TUNNEL_NAV_ACTION_NODE_H