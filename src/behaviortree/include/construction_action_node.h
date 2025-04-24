#ifndef CONSTRUCTION_ACTION_NODE_H
#define CONSTRUCTION_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/ConstructionAction.h>

class ConstructionActionNode : public BT::SyncActionNode
{
public:
    ConstructionActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("construction", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for construction action server...");
        client_.waitForServer();
        ROS_INFO("Construction action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        // No ports needed for now as per requirements
        return {};
    }

    BT::NodeStatus tick() override
    {
        // Create and send the goal (empty as per requirements)
        msg_file::ConstructionGoal goal;

        ROS_INFO("Sending construction mission goal");
        client_.sendGoal(goal);

        // Wait for action to complete with a reasonable timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(300.0)); // 5 min timeout

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Construction mission succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Construction mission failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Construction mission timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::ConstructionAction> client_;
};

#endif // CONSTRUCTION_ACTION_NODE_H