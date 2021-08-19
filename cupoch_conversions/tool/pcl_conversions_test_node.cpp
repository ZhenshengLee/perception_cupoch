// prj hdrs
#include "pcl_conversions/pcl_conversions.h"

// ros hdrs
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

using namespace std;
using namespace pcl;

using PointT = pcl::PointXYZ;

std::string camera_point_topic;
pcl::PointCloud<PointT>::Ptr cloud{new pcl::PointCloud<PointT>};
sensor_msgs::PointCloud2 m_pub_pcl_pc;

ros::Publisher time_pub;

void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO_STREAM_ONCE("points_callback");

    auto t1 = ros::WallTime::now();
    auto t2 = ros::WallTime::now();

    t1 = ros::WallTime::now();
    pcl::fromROSMsg(*points_msg, *cloud);
    t2 = ros::WallTime::now();
    std_msgs::Int32 time;
    time.data = (t2 - t1).toSec() * 1000.0;
    time_pub.publish(time);
    ROS_INFO_STREAM("rosToPcl processing_time: " << (t2 - t1).toSec() * 1000.0 << "[ms]");

    ROS_INFO("this msg has %d points", cloud->points.size());

    ROS_INFO_STREAM_ONCE("points_callback end");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_conversions_test_node");
    ros::NodeHandle private_nh("~");

    private_nh.param("camera3d_point_topic", camera_point_topic, std::string("/points_cloud"));

    auto points_sub = private_nh.subscribe(camera_point_topic, 10, points_callback);
    time_pub = private_nh.advertise<std_msgs::Int32>("/pcl/ros2pcl", 1);

    ros::spin();

    return 0;
}