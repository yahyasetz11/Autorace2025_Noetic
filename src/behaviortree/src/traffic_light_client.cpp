#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/TrafficLightAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::TrafficLightResultConstPtr &result)
{
    ROS_INFO("Action completed with status: %s", state.toString().c_str());
    ROS_INFO("Result: %s, Final state: %s",
             result->success ? "success" : "failure",
             result->final_state.c_str());
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal activated");
}

void feedbackCb(const msg_file::TrafficLightFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: time elapsed = %.2f seconds, state = %s, red_pixels = %d, green_pixels = %d",
             feedback->time_elapsed, feedback->current_state.c_str(),
             feedback->red_pixels, feedback->green_pixels);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_light_client_test");

    // Create action client
    actionlib::SimpleActionClient<msg_file::TrafficLightAction> client("traffic_light", true);

    ROS_INFO("Waiting for server...");
    client.waitForServer();
    ROS_INFO("Server available!");

    // Set goal
    msg_file::TrafficLightGoal goal;
    goal.timeout = 30.0; // 30 second timeout

    ROS_INFO("Sending goal...");
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}