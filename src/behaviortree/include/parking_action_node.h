#ifndef PARKING_ACTION_NODE_H
#define PARKING_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/ParkingAction.h>

class ParkingActionNode : public BT::SyncActionNode
{
public:
    ParkingActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("parking", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for parking action server...");
        client_.waitForServer();
        ROS_INFO("Parking action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("mode", "dynamic", "Parking mode: left, right, or dynamic")};
    }

    BT::NodeStatus tick() override
    {
        // Get input parameters
        std::string mode = "dynamic";
        getInput("mode", mode);

        // Validate mode
        if (mode != "left" && mode != "right" && mode != "dynamic")
        {
            ROS_WARN("Invalid parking mode: %s. Using default 'dynamic'", mode.c_str());
            mode = "dynamic";
        }

        // Create and send the goal
        msg_file::ParkingGoal goal;
        goal.mode = mode;

        ROS_INFO("Sending parking goal - Mode: %s", mode.c_str());

        client_.sendGoal(goal);

        // Wait for action to complete with timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(300.0)); // 5 minutes timeout

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Parking mission succeeded");
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Parking mission failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Parking mission timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::ParkingAction> client_;
};

#endif // PARKING_ACTION_NODE_H