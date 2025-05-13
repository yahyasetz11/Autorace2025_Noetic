#ifndef TRAFFIC_LIGHT_ACTION_NODE_H
#define TRAFFIC_LIGHT_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/TrafficLightAction.h>

class TrafficLightActionNode : public BT::SyncActionNode
{
public:
    TrafficLightActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("traffic_light", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for traffic_light action server...");
        client_.waitForServer();
        ROS_INFO("Traffic Light action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<float>("timeout", 30.0f, "Maximum time to wait if no traffic light detected (seconds)")};
    }

    BT::NodeStatus tick() override
    {
        // Get timeout parameter
        float timeout = 30.0f;
        getInput("timeout", timeout);

        // Create and send the goal
        msg_file::TrafficLightGoal goal;
        goal.timeout = timeout;

        ROS_INFO("Sending traffic light goal - Timeout: %.2f seconds", timeout);

        client_.sendGoal(goal);

        // Wait for action to complete with a reasonable timeout
        bool finished_before_timeout = client_.waitForResult(ros::Duration(timeout + 10.0)); // Add buffer to timeout

        if (finished_before_timeout)
        {
            auto result = client_.getResult();
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Traffic light detection succeeded with final state: %s",
                         result->final_state.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Traffic light detection failed with state: %s",
                         client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Traffic light detection timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::TrafficLightAction> client_;
};

#endif // TRAFFIC_LIGHT_ACTION_NODE_H