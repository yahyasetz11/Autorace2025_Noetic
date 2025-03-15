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
    // Tunggu action server mulai
    ROS_INFO("Menunggu lane_detect action server...");
    client_.waitForServer();
    ROS_INFO("Lane detect action server terhubung!");
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("duration")};
  }

  BT::NodeStatus tick() override
  {
    float duration;
    if (!getInput("duration", duration))
    {
      ROS_ERROR("Missing required input [duration]");
      return BT::NodeStatus::FAILURE;
    }

    msg_file::LaneDetectGoal goal;
    goal.duration = duration;

    ROS_INFO("Mengirim goal dengan durasi: %f", duration);
    client_.sendGoal(goal);

    // Tunggu action selesai
    bool finished_before_timeout = client_.waitForResult(ros::Duration(duration + 1.0));

    if (finished_before_timeout)
    {
      if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Lane detection berhasil");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        ROS_INFO("Lane detection gagal dengan state: %s", client_.getState().toString().c_str());
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      ROS_INFO("Lane detection timeout");
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  actionlib::SimpleActionClient<msg_file::LaneDetectAction> client_;
};

#endif // LANE_DETECT_ACTION_NODE_H