#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/CrossWalkAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::CrossWalkResultConstPtr &result)
{
    ROS_INFO("Action completed with status: %s", state.toString().c_str());
    ROS_INFO("Result: %s", result->success ? "success" : "failure");
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal activated");
}

void feedbackCb(const msg_file::CrossWalkFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: time elapsed = %.2f seconds, state = %d, pixel count = %d",
             feedback->time_elapsed, feedback->current_state, feedback->pixel_count);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cross_walk_client_test");

    // Create action client
    actionlib::SimpleActionClient<msg_file::CrossWalkAction> client("cross_walk", true);

    ROS_INFO("Waiting for server...");
    client.waitForServer();
    ROS_INFO("Server available!");

    // Set goal (empty as per requirements)
    msg_file::CrossWalkGoal goal;

    ROS_INFO("Sending goal...");
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}