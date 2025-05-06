#ifndef CROSS_WALKING_ACTION_NODE_H
#define CROSS_WALKING_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/CrossWalkAction.h>

class CrossWalkActionNode : public BT::SyncActionNode
{
public:
    CrossWalkActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("cross_walk", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for cross_walk action server...");
        client_.waitForServer();
        ROS_INFO("CrossWalk action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        // No ports needed as per requirements
        return {};
    }

    BT::NodeStatus tick() override
    {
        // Create and send the goal (empty as no parameters needed)
        msg_file::CrossWalkGoal goal;

        ROS_INFO("Sending CrossWalk goal");
        client_.sendGoal(goal);

        // Wait for action to complete with a reasonable timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(300.0)); // 5 min timeout

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("CrossWalk mission succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("CrossWalk mission failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("CrossWalk mission timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::CrossWalkAction> client_;
};

#endif // CROSS_WALKING_ACTION_NODE_H