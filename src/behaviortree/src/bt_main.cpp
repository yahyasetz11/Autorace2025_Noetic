#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ros/package.h>
#include "lane_detect_action_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect_bt");
    ros::NodeHandle nh;

    // Create BehaviorTree factory
    BT::BehaviorTreeFactory factory;

    // Register custom action node
    factory.registerNodeType<LaneDetectActionNode>("LaneDetect");

    // Load XML dari file
    std::string xml_filename = ros::package::getPath("behaviortree") + "/xml/bt.xml";
    auto tree = factory.createTreeFromFile(xml_filename);

    // Create logger
    BT::StdCoutLogger logger_cout(tree);

    // Execute the tree
    tree.tickRootWhileRunning();

    return 0;
}