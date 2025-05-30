#ifndef ROTATE_ACTION_NODE_H
#define ROTATE_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/RotateAction.h>

class RotateActionNode : public BT::SyncActionNode
{
public:
    RotateActionNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config),
          client_("rotate", true)
    {
        // Wait for the action server to start
        ROS_INFO("Waiting for rotate action server...");
        client_.waitForServer();
        ROS_INFO("Rotate action server connected!");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("direction", "clockwise", "Rotation direction: clockwise, counterclockwise, or align"),
            BT::InputPort<float>("angle", 90.0, "Angle to rotate in degrees (for clockwise/counterclockwise) or target yaw angle (for align mode)")};
    }

    BT::NodeStatus tick() override
    {
        // Get input parameters
        std::string direction = "clockwise";
        float angle = 90.0;

        getInput("direction", direction);
        getInput("angle", angle);

        // Create and send the goal
        msg_file::RotateGoal goal;
        goal.direction = direction;
        goal.angle = angle;

        if (direction == "align")
        {
            ROS_INFO("Sending goal - Direction: %s, Target Yaw: %.2f degrees",
                     direction.c_str(), angle);
        }
        else
        {
            ROS_INFO("Sending goal - Direction: %s, Angle: %.2f degrees",
                     direction.c_str(), angle);
        }

        client_.sendGoal(goal);

        // Calculate timeout based on mode and parameters
        double timeout;
        if (direction == "align")
        {
            timeout = 30.0; // Fixed timeout for align mode
        }
        else
        {
            // Calculate timeout based on angle with some buffer
            timeout = (fabs(angle) / 45.0) * 5.0; // rough estimate: 5 seconds per 45 degrees
            if (timeout < 10.0)
                timeout = 10.0; // minimum timeout
        }

        bool finished_before_timeout = client_.waitForResult(ros::Duration(timeout));

        if (finished_before_timeout)
        {
            if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Rotate action succeeded. Final angle: %.2f degrees",
                         client_.getResult()->final_angle);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Rotate action failed with state: %s", client_.getState().toString().c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
        {
            ROS_INFO("Rotate action timeout");
            client_.cancelGoal();
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    actionlib::SimpleActionClient<msg_file::RotateAction> client_;
};

#endif // ROTATE_ACTION_NODE_H