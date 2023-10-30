#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>

#include "Lines.h"
#include "line_functions.h"

using namespace std::chrono_literals;

class LinePublisher : public rclcpp::Node
{
public:
  LinePublisher(): Node("scan_subscriber")
  {
    auto callback = std::bind(&LinePublisher::laserScanCallback, this, std::placeholders::_1);
    auto subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, callback);

    auto publisher = create_publisher<visualization_msgs::msg::MarkerArray>("/allTheLines",10);

    // Publish lines in 5Hz
    timer_ = this->create_wall_timer(200ms, std::bind(&LinePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Scan subscriber initialized");

    subscriber_ = subscriber;
    publisher_ = publisher;
  }

private:

    void rangesToCartesianCoordinates(std::vector<point2D>& Measurements_ , const int Np)
    {
        for (int i = 0; i < Np; i++)
        {
            point2D point;
            
            float range = laser_msg->ranges[i];
            
            float angle = laser_msg->angle_min + i * laser_msg->angle_increment;

            point.x = range * cos(angle);
            point.y = range * sin(angle);
            
            Measurements_.push_back(point);
        }
        
    }

    void visualize(const std::vector<straightLine> finishedLines)
    {

        visualization_msgs::msg::MarkerArray multiLines;
        builtin_interfaces::msg::Duration lifeTimeDuration;
        lifeTimeDuration.nanosec = 200e6;
        // Arbitrary height.
        geometry_msgs::msg::Point startingPoint , EndingPoint;
        int numberOfFinishedLines = finishedLines.size();
        startingPoint.z = 0.1;
        EndingPoint.z = 0.1;
        for (int i = 0 ; i < numberOfFinishedLines ; i++)
        {
            visualization_msgs::msg::Marker singleLine , singlePoint;
            singleLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
            singleLine.header.frame_id = "base_scan";
            singleLine.lifetime = lifeTimeDuration;
            singleLine.ns = std::to_string(i);
            singleLine.scale.x = 0.1;
            
            // White lines.
            singleLine.color.a = 1;
            singleLine.color.r = 1;
            singleLine.color.g = 1;
            singleLine.color.b = 1;

            startingPoint.x = finishedLines[i].realStartPoint.x;
            startingPoint.y = finishedLines[i].realStartPoint.y;
            EndingPoint.x = finishedLines[i].realEndPoint.x;
            EndingPoint.y = finishedLines[i].realEndPoint.y;
            singleLine.points.push_back(startingPoint);
            singleLine.points.push_back(EndingPoint);

            

            multiLines.markers.push_back(singleLine);
            
            // ---------------------- Points Area -----------------------
            
            if (finishedLines[i].realEndPoint.intersect)
            {
                singlePoint.type = visualization_msgs::msg::Marker::SPHERE;
                singlePoint.header.frame_id = "base_scan";
                singlePoint.ns = std::to_string(i + 100);
                singlePoint.lifetime = lifeTimeDuration;
                singlePoint.scale.x = 0.1;
                singlePoint.scale.y = 0.1;
                singlePoint.scale.z = 0.1;
                // Red spheres.
                singlePoint.color.a = 1;
                singlePoint.color.r = 0;
                singlePoint.color.g = 1;
                singlePoint.color.b = 0;
                singlePoint.pose.position.x = finishedLines[i].realEndPoint.x;
                singlePoint.pose.position.y = finishedLines[i].realEndPoint.y;
                singlePoint.pose.position.z = 0.2;
                multiLines.markers.push_back(singlePoint);
            }
        }
        this->publisher_->publish(multiLines);
    }

    void countLines(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int Np_ = msg->ranges.size();
        Lines lines(epsilon_ , delta_, Snum_ , Pmin_ , Np_ , Lmin_);
        std::vector<point2D> Measurements_;
        
        rangesToCartesianCoordinates(Measurements_ , Np_);

        

        lines.uploadData(Measurements_);
        // lines.seedSegmentDetection();
        // lines.regionGrowing();
        // lines.cleanSameLines();
        // lines.overlapRegionProcessing();
        // lines.joinLines();
        lines.find();
        
        std::vector<straightLine> finishedLines = lines.getFinishedLines();
        
        
        visualize(finishedLines);

    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        flag = true;
        laser_msg = msg;
    }

    void timer_callback()
    {
        if (flag){countLines(laser_msg);}
        else {RCLCPP_INFO(this->get_logger(), "No scan data has been recieved.");}
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_msg;

    // Algorithm constants:
    bool flag = false;
    float epsilon_ = 0.06 , delta_ = 0.15;
    int Snum_ = 6 , Pmin_ = 10 , Lmin_ = 0.4;
    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    // Create a LinePublisher node
    auto node = std::make_shared<LinePublisher>();

    // Spin the node until it is shutdown
    rclcpp::spin(node);

    // Destroy the node
    rclcpp::shutdown();

    return 0;
}