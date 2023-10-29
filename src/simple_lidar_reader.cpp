#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

class ScanSubscriber : public rclcpp::Node
{
public:
  ScanSubscriber()
    : Node("scan_subscriber")
  {
    // Create a subscriber to the scan topic
    auto callback = std::bind(&ScanSubscriber::laserScanCallback, this, std::placeholders::_1);
    auto subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, callback);

    auto publisher = create_publisher<sensor_msgs::msg::PointCloud>("/laser2pointcloud",10);

    // Print a message when the subscriber is initialized
    RCLCPP_INFO(this->get_logger(), "Scan subscriber initialized");

    subscriber_ = subscriber;
    publisher_ = publisher;

  }

private:

  void PublishLaserAsPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr msg){

    // Create a new point cloud message
    sensor_msgs::msg::PointCloud cloud;

    // Calculate the 3D coordinates of each point in the point cloud
    int NumberOfMeasurements = msg->ranges.size();
    for (int i = 0; i < NumberOfMeasurements; i++)
    {
      float range = msg->ranges[i];
      float angle = msg->angle_min + i * msg->angle_increment;

      geometry_msgs::msg::Point32 point;

      point.x = range * cos(angle);
      point.y = range * sin(angle);
      point.z = 0.1f; // Assuming the laser scanner sensor is a bit higher than ground level

      // Add the point to the point cloud message
      cloud.points.push_back(point);
    }

    // Set the header of the point cloud message
    cloud.header.frame_id = msg->header.frame_id;
    cloud.header.stamp = msg->header.stamp;

    // Publish the point cloud message
    publisher_->publish(cloud);
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    PublishLaserAsPointCloud(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a ScanSubscriber node
  auto node = std::make_shared<ScanSubscriber>();

  // Spin the node until it is shutdown
  rclcpp::spin(node);

  // Destroy the node
  rclcpp::shutdown();

  return 0;
}