#ifndef ALIGN_ACTION_NODE_H
#define ALIGN_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/AlignAction.h>

class AlignActionNode : public BT::SyncActionNode
{
public:
    AlignActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("align", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for align action server...");
        client_.waitForServer();
        ROS_INFO("Align action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("by", "right", "Direction to align: right, left, front, back")};
    }

    BT::NodeStatus tick() override
    {
        // Get input parameter
        std::string direction = "right";
        getInput("by", direction);

        // Create and send the goal
        msg_file::AlignGoal goal;
        goal.by = direction;

        ROS_INFO("Sending align goal - Direction: %s", direction.c_str());

        client_.sendGoal(goal);

        // Wait for action to complete (using the timeout defined in the server)
        bool finished_before_timeout = client_.waitForResult(ros::Duration(35.0)); // Slightly longer than server's timeout

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Alignment succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Alignment failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Alignment timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::AlignAction> client_;
};

#endif // ALIGN_ACTION_NODE_H