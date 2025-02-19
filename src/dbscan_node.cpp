#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include "sllidar_ros2/dbscan.hpp"

class Dbscan : public rclcpp::Node
{
public:
    Dbscan() : Node("dbscan_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&Dbscan::filter_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dbscan_clusters", 10);
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<std::vector<double>> scan_points;
        scan_points.reserve(msg->ranges.size());

        double angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (std::isfinite(r) && (r >= msg->range_min && r <= msg->range_max))
            {
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                scan_points.push_back({x, y});
            }
            angle += msg->angle_increment;
        }

        //DBSCAN
        double eps = 0.2;    // 近傍閾値 (m)
        int min_samples = 10; // コア点判定に必要な最小点数
        DBSCAN dbscan(eps, min_samples);
        dbscan.fit(scan_points);

        auto marker_array = createMarkerArray(dbscan.getPoints());
        marker_pub_->publish(marker_array);

    }

    visualization_msgs::msg::MarkerArray createMarkerArray(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int max_label = -1;
        for (auto &p : points) {
            if (p.label > max_label) {
                max_label = p.label;
            }
        }

        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_frame"; 
            marker.header.stamp = this->now();
            marker.ns = "dbscan_clusters";
            marker.id = cluster_id;

            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.01;
            marker.scale.y = 0.01;

            std_msgs::msg::ColorRGBA color = idToColor(cluster_id);
            marker.color = color;

            for (size_t i = 0; i < points.size(); ++i) {
                if (points[i].label == cluster_id) {
                    geometry_msgs::msg::Point pt;

                    pt.x = points[i].coords[0];
                    pt.y = points[i].coords[1];
                    pt.z = 0.0;
                    marker.points.push_back(pt);
                }
            }

            marker_array.markers.push_back(marker);
        }

        //ノイズも可視化する場合: label = -1 を別 Marker にする
        {
            visualization_msgs::msg::Marker noise_marker;
            noise_marker.header.frame_id = "laser_frame";
            noise_marker.header.stamp = this->now();
            noise_marker.ns = "dbscan_clusters";

            noise_marker.id = max_label + 1;
            noise_marker.type = visualization_msgs::msg::Marker::POINTS;
            noise_marker.action = visualization_msgs::msg::Marker::ADD;
            noise_marker.pose.orientation.w = 1.0;
            noise_marker.scale.x = 0.01;
            noise_marker.scale.y = 0.01;
 
            noise_marker.color.r = 0.5f;
            noise_marker.color.g = 0.5f;
            noise_marker.color.b = 0.5f;
            noise_marker.color.a = 1.0f;

            for (auto &p : points) {
                if (p.label == -1) {
                    geometry_msgs::msg::Point pt;
                    pt.x = p.coords[0];
                    pt.y = p.coords[1];
                    pt.z = 0.0;
                    noise_marker.points.push_back(pt);
                }
            }
            marker_array.markers.push_back(noise_marker);
        }

        return marker_array;
    }

    std_msgs::msg::ColorRGBA idToColor(int id)
    {
        std_msgs::msg::ColorRGBA c;
        c.a = 1.0f;

        switch (id % 6) {
        case 0: c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; break; // 赤
        case 1: c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; break; // 緑
        case 2: c.r = 0.0f; c.g = 0.0f; c.b = 1.0f; break; // 青
        case 3: c.r = 1.0f; c.g = 1.0f; c.b = 0.0f; break; // 黄
        case 4: c.r = 1.0f; c.g = 0.0f; c.b = 1.0f; break; // マゼンタ
        case 5: c.r = 0.0f; c.g = 1.0f; c.b = 1.0f; break; // シアン
        default: c.r = 1.0f; c.g = 1.0f; c.b = 1.0f; break; // 白
        }

        return c;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dbscan>());
    rclcpp::shutdown();
    return 0;
}

