#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/ConstructionAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::ConstructionResultConstPtr &result)
{
    ROS_INFO("Action completed with status: %s", state.toString().c_str());
    ROS_INFO("Result: %s", result->success ? "success" : "failure");
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal activated");
}

void feedbackCb(const msg_file::ConstructionFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: time elapsed = %.2f seconds, phase = %s",
             feedback->time_elapsed, feedback->current_phase.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "construction_client_test");

    // Create action client
    actionlib::SimpleActionClient<msg_file::ConstructionAction> client("construction", true);

    ROS_INFO("Waiting for construction server...");
    client.waitForServer();
    ROS_INFO("Server available!");

    // Set goal (empty for now as per requirements)
    msg_file::ConstructionGoal goal;

    ROS_INFO("Sending goal...");
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}