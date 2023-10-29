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

    void rangesToCartesianCoordinates(std::vector<point2D>& Measurements , const int Np)
    {
        for (int i = 0; i < Np; i++)
        {
            point2D point;
            
            float range = laser_msg->ranges[i];
            
            float angle = laser_msg->angle_min + i * laser_msg->angle_increment;

            point.x = range * cos(angle);
            point.y = range * sin(angle);
            
            Measurements.push_back(point);
        }
        
    }

    void visualizeLinesInRviz(const std::vector<straightLine> finishedLines , const std::vector<point2D> Measurements)
    {
        visualization_msgs::msg::MarkerArray multiLines;
        geometry_msgs::msg::Point startingPoint , EndingPoint;
        point2D start , end;
        // Arbitrary height.
        startingPoint.z = 0.1;
        EndingPoint.z = 0.1;
        int numberOfFinishedLines = finishedLines.size();
        for (int i = 0 ; i < numberOfFinishedLines ; i++)
        {

            visualization_msgs::msg::Marker singleLine;
            singleLine.type = visualization_msgs::msg::Marker::LINE_STRIP;
            singleLine.header.frame_id = "base_scan";
            singleLine.ns = std::to_string(i);
            singleLine.scale.x = 0.1;
            // Starting point.
            start = createPredictedPosition(Measurements[finishedLines[i].start] , finishedLines[i].params);
            end = createPredictedPosition(Measurements[finishedLines[i].end] , finishedLines[i].params);
            
            startingPoint.x = start.x;
            startingPoint.y = start.y;
            EndingPoint.x = end.x;
            EndingPoint.y = end.y;
            singleLine.points.push_back(startingPoint);
            singleLine.points.push_back(EndingPoint);

            // White lines.
            singleLine.color.a = 1;
            singleLine.color.r = 1;
            singleLine.color.g = 1;
            singleLine.color.b = 1;

            multiLines.markers.push_back(singleLine);
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
        lines.seedSegmentDetection();
        lines.regionGrowing();
        lines.cleanSameLines();
        lines.overlapRegionProcessing();
        lines.joinLines();
        // std::vector<straightLine> finishedLines = lines.getSeeds();
        // std::vector<straightLine> finishedLines = lines.getRegions();
        
        std::vector<straightLine> finishedLines = lines.getFinishedLines();
        // int numberOfFinishedLines = finishedLines.size();
        // RCLCPP_INFO(this->get_logger(), "%s", std::to_string(numberOfFinishedLines).c_str());
        visualizeLinesInRviz(finishedLines , Measurements_);

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