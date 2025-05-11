#ifndef LANE_DETECT_ACTION_NODE_H
#define LANE_DETECT_ACTION_NODE_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/LaneDetectAction.h>

class LaneDetectActionNode : public BT::SyncActionNode
{
public:
  LaneDetectActionNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config),
        client_("lane_detect", true)
  {
    // Wait for the action server to start
    ROS_INFO("Waiting for lane_detect action server...");
    client_.waitForServer();
    ROS_INFO("Lane detect action server connected!");
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<float>("duration", 0.0, "Duration in seconds (0 for infinite)"),
        BT::InputPort<std::string>("mode", "center", "Lane following mode: center, left, or right"),
        BT::InputPort<std::string>("sign", "", "Sign to wait for (empty if using duration)"),
        BT::InputPort<float>("speed", 0.0, "Linear speed override (0 to use YAML value)")};
  }

  BT::NodeStatus tick() override
  {
    // Get input parameters
    float duration = 0.0;
    std::string mode = "center";
    std::string sign = "";
    float speed = 0.0;

    getInput("duration", duration);
    getInput("mode", mode);
    getInput("sign", sign);
    getInput("speed", speed);

    // Create and send the goal
    msg_file::LaneDetectGoal goal;
    goal.duration = duration;
    goal.mode = mode;
    goal.sign = sign;
    goal.speed = speed;

    ROS_INFO("Sending goal - Mode: %s, Duration: %.1f, Sign: %s, Speed: %.2f",
             mode.c_str(), duration, sign.empty() ? "none" : sign.c_str(), speed);

    client_.sendGoal(goal);

    // Wait for action to complete
    // If sign-based or both-lanes based, we need a longer timeout
    float timeout = duration;
    if (duration <= 0 || !sign.empty())
    {
      timeout = 300.0; // 5 minutes timeout for sign-based detection
    }
    else
    {
      timeout += 1.0; // Add buffer for duration-based detection
    }

    bool finished_before_timeout = client_.waitForResult(ros::Duration(timeout));

    if (finished_before_timeout)
    {
      if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Lane detection succeeded");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("Lane detection failed with state: %s", client_.getState().toString().c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      ROS_INFO("Lane detection timeout");
      client_.cancelGoal();
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  actionlib::SimpleActionClient<msg_file::LaneDetectAction> client_;
};

#endif // LANE_DETECT_ACTION_NODE_H