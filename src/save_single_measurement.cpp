#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <fstream>
#include <string>

class PointCloudToCSVSaver : public rclcpp::Node
{
public:
  PointCloudToCSVSaver()
    : Node("point_cloud_to_csv_saver")
  {
    // Create a subscriber to the point cloud topic
    auto subscriber = create_subscription<sensor_msgs::msg::PointCloud>(
      "/laser2pointcloud", 10, std::bind(&PointCloudToCSVSaver::pointCloudCallback, this, std::placeholders::_1));
    // Create a CSV file writer object
    
    csv_writer.open("point_cloud.csv");
    csv_writer << "x" << "," << "y" << "," <<"z" << "\n";
    
    exitAfterOneIteration = true;
    subscriber_ = subscriber;
  }

  

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
  {
    // Iterate over the point cloud and write each point to the CSV file
    int numberOfMeasurements = msg->points.size();
    RCLCPP_INFO(this->get_logger(), "%s", std::to_string(numberOfMeasurements).c_str());
    for (int i = 0; i < numberOfMeasurements; i++)
    {
      csv_writer << msg->points[i].x << "," << msg->points[i].y << "," << msg->points[i].z << "\n";
      
    }
    csv_writer.close();
    RCLCPP_INFO(this->get_logger(), "Created the file!\n");
    rclcpp::shutdown();
  }

  
  bool exitAfterOneIteration;
  std::ofstream csv_writer;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subscriber_;
  std::ofstream csv_file;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create a PointCloudToCSVSaver node
  auto node = std::make_shared<PointCloudToCSVSaver>();

  // Spin the node until it is shutdown
  rclcpp::spin(node);

  // Destroy the node
  rclcpp::shutdown();

  return 0;
}