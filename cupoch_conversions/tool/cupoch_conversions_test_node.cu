// prj hdrs
#include "cupoch_conversions/cupoch_conversions.hpp"

// ros hdrs
#include <ros/ros.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;
using namespace cupoch;

std::string camera_point_topic;

int main(int argc, char** argv)
{
    // 初始化ROS环境,一个进程
    ros::init(argc, argv, "cupoch_conversions_test_node");
    ros::NodeHandle private_nh("~");

    utility::InitializeAllocator();
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    private_nh.param("camera3d_point_topic", camera_point_topic, std::string("/points_cloud"));

    ROS_INFO("waiting for msg in topic(%s)", camera_point_topic.c_str());
    sensor_msgs::msg::PointCloud2::SharedPtr msg =
        ros::topic::waitForMessage<sensor_msgs::msg::PointCloud2>(camera_point_topic, private_nh);
    ROS_INFO("topic(%s) received...", camera_point_topic.c_str());

    auto cloud = std::make_shared<geometry::PointCloud>();
    cupoch_conversions::rosToCupoch(msg, cloud);

    if (cloud->HasPoints())
    {
        ROS_INFO("has points");
    }



    ROS_INFO("topic(%s) processed done...", camera_point_topic.c_str());
    return 0;
}