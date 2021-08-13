// std hdrs
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

// prj hdrs
#include "cupoch_conversions/cupoch_conversions.hpp"

// ros hdrs
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;
using namespace std::chrono_literals;
using namespace cupoch;
using namespace cupoch_conversions;

class Talker: public rclcpp::Node {
private:
  using Topic = sensor_msgs::msg::PointCloud2;

public:
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("shm_demo_pc_talker", options) {

    input_cloud = io::CreatePointCloudFromFile("./res/pcd/twoobstacle.pcd");
    if (input_cloud->HasPoints()) {
      RCLCPP_INFO(this->get_logger(), "has points");
    }

    auto publishMessage = [this]()->void {

      Topic msg;
      cupochToRos(input_cloud, msg);
      RCLCPP_INFO(this->get_logger(), "Publishing pointcloud by cupoch");
      m_publisher->publish(std::move(msg));
    };

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    m_publisher = this->create_publisher < Topic > ("cupoch_pc", qos);

    // Use a timer to schedule periodic message publishing.
    m_timer = this->create_wall_timer(1s, publishMessage);
  }

private:
  rclcpp::Publisher < Topic > ::SharedPtr m_publisher;
  rclcpp::TimerBase::SharedPtr m_timer;
  std::shared_ptr < cupoch::geometry::PointCloud >
  input_cloud {std::make_shared < cupoch::geometry::PointCloud > ()};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  utility::InitializeAllocator(cupoch::utility::rmmAllocationMode_t::PoolAllocation, 1000000000);
  utility::SetVerbosityLevel(cupoch::utility::VerbosityLevel::Debug);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared < Talker > (options));
  rclcpp::shutdown();

  return 0;
}