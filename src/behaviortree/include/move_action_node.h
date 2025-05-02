// Create a new file at src/behaviortree/include/move_action_node.h with this content:
#ifndef MOVE_ACTION_NODE_H
#define MOVE_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/MoveAction.h>

class MoveActionNode : public BT::SyncActionNode
{
public:
    MoveActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("move", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for move action server...");
        client_.waitForServer();
        ROS_INFO("Move action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("mode", "forward", "Movement mode: forward or backward"),
            BT::InputPort<std::string>("until", "front", "Direction to check distance: front, back, left, or right"),
            BT::InputPort<float>("distance", 0.3, "Distance threshold in meters")};
    }

    BT::NodeStatus tick() override
    {
        // Get input parameters
        std::string mode = "forward";
        std::string until = "front";
        float distance = 0.3;

        getInput("mode", mode);
        getInput("until", until);
        getInput("distance", distance);

        // Create and send the goal
        msg_file::MoveGoal goal;
        goal.mode = mode;
        goal.until = until;
        goal.distance = distance;

        ROS_INFO("Sending goal - Mode: %s, Until: %s, Distance: %.2f m",
                 mode.c_str(), until.c_str(), distance);

        client_.sendGoal(goal);

        // Wait for action to complete with a reasonable timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(60.0));

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Move action succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Move action failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Move action timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::MoveAction> client_;
};

#endif // MOVE_ACTION_NODE_H