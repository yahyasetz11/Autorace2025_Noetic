#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <msg_file/LaneDetectAction.h>

void doneCb(const actionlib::SimpleClientGoalState &state,
            const msg_file::LaneDetectResultConstPtr &result)
{
    ROS_INFO("Action selesai dengan status: %s", state.toString().c_str());
    ROS_INFO("Hasil: %s", result->success ? "sukses" : "gagal");
    ros::shutdown();
}

void activeCb()
{
    ROS_INFO("Goal diaktifkan");
}

void feedbackCb(const msg_file::LaneDetectFeedbackConstPtr &feedback)
{
    ROS_INFO("Feedback: waktu berlalu = %.2f detik", feedback->time_elapsed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_detect_client_test");

    // Buat action client
    actionlib::SimpleActionClient<msg_file::LaneDetectAction> client("lane_detect", true);

    ROS_INFO("Menunggu server...");
    client.waitForServer();
    ROS_INFO("Server tersedia!");

    // Set goal
    msg_file::LaneDetectGoal goal;
    goal.duration = 20.0; // 20 detik

    ROS_INFO("Mengirim goal...");
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}