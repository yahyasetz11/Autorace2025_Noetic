#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/ParkingAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::ParkingResultConstPtr &result)
{
    ROS_INFO("Action completed with status: %s", state.toString().c_str());
    ROS_INFO("Result: %s", result->success ? "success" : "failure");
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal activated");
}

void feedbackCb(const msg_file::ParkingFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: time elapsed = %.2f seconds", feedback->time_elapsed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "parking_client_test");
    ros::NodeHandle nh;

    // Get parameters
    std::string mode;
    nh.param<std::string>("mode", mode, "dynamic");

    // Create action client
    actionlib::SimpleActionClient<msg_file::ParkingAction> client("parking", true);

    ROS_INFO("Waiting for server...");
    client.waitForServer();
    ROS_INFO("Server available!");

    // Set goal
    msg_file::ParkingGoal goal;
    goal.mode = mode;

    ROS_INFO("Sending goal with mode = %s", mode.c_str());
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}