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

        // DBSCAN
        double eps = 0.1;     // 近傍閾値 (m)
        int min_samples = 10; // コア点判定に必要な最小点数
        DBSCAN dbscan(eps, min_samples);
        dbscan.fit(scan_points);

        // DBSCAN の結果 (ポイント群) を取得
        const auto & clustered_points = dbscan.getPoints();

        // MarkerArray を生成
        visualization_msgs::msg::MarkerArray marker_array;

        // 1. クラスター点群を可視化するMarkerを追加
        auto cluster_markers = createClusterMarkers(clustered_points);
        for (auto &mk : cluster_markers.markers) {
            marker_array.markers.push_back(mk);
        }

        // 2. 各クラスタの重心Markerを追加
        auto centroid_markers = createCentroidMarkers(clustered_points);
        for (auto &mk : centroid_markers.markers) {
            marker_array.markers.push_back(mk);
        }

        // Publish
        marker_pub_->publish(marker_array);
    }

    // ==========================
    //  クラスターのMarkerを作成
    // ==========================
    visualization_msgs::msg::MarkerArray createClusterMarkers(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // 最大クラスタIDを取得
        int max_label = -1;
        for (auto &p : points) {
            if (p.label > max_label) {
                max_label = p.label;
            }
        }

        // クラスタIDが有効なもの(0～max_label)をMarker (POINTS) で可視化
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_frame"; 
            marker.header.stamp = this->now();
            marker.ns = "dbscan_clusters";
            marker.id = cluster_id;      // クラスタごとにIDを割り当て
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;       // 点の大きさ
            marker.scale.y = 0.01;
            marker.color = idToColor(cluster_id);

            // クラスタIDに属する点を追加
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

        // ラベル -1 (ノイズ) を可視化する場合
        {
            visualization_msgs::msg::Marker noise_marker;
            noise_marker.header.frame_id = "laser_frame";
            noise_marker.header.stamp = this->now();
            noise_marker.ns = "dbscan_clusters";
            noise_marker.id = max_label + 1; // ノイズ用ID
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

    // ==========================
    //  各クラスタの重心を可視化
    // ==========================
    visualization_msgs::msg::MarkerArray createCentroidMarkers(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // 最大クラスタIDを取得
        int max_label = -1;
        for (auto &p : points) {
            if (p.label > max_label) {
                max_label = p.label;
            }
        }

        // 各クラスタの座標を集計 (label >= 0)
        std::vector<double> sum_x(max_label + 1, 0.0);
        std::vector<double> sum_y(max_label + 1, 0.0);
        std::vector<int> count(max_label + 1, 0);

        for (auto &p : points) {
            if (p.label >= 0) {
                sum_x[p.label] += p.coords[0];
                sum_y[p.label] += p.coords[1];
                count[p.label] += 1;
            }
        }

        // クラスタごとに重心Markerを作成
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id) {
            if (count[cluster_id] == 0) {
                continue;
            }

            double mean_x = sum_x[cluster_id] / count[cluster_id];
            double mean_y = sum_y[cluster_id] / count[cluster_id];

            // 1つの重心点を描画するMarkerを生成
            visualization_msgs::msg::Marker centroid_marker;
            centroid_marker.header.frame_id = "laser_frame";
            centroid_marker.header.stamp = this->now();
            centroid_marker.ns = "dbscan_centroids";
            centroid_marker.id = cluster_id;         // クラスタIDをそのまま採用
            centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;  // 形状は好みで
            centroid_marker.action = visualization_msgs::msg::Marker::ADD;

            // 重心位置を設定
            centroid_marker.pose.position.x = mean_x;
            centroid_marker.pose.position.y = mean_y;
            centroid_marker.pose.position.z = 0.0;
            centroid_marker.pose.orientation.w = 1.0;

            // 重心Markerの大きさ
            centroid_marker.scale.x = 0.05;  // 半径5cm程度で設定
            centroid_marker.scale.y = 0.05;
            centroid_marker.scale.z = 0.05;

            // 黒色(r=g=b=0.0)に設定
            centroid_marker.color.r = 0.0f;
            centroid_marker.color.g = 0.0f;
            centroid_marker.color.b = 0.0f;
            centroid_marker.color.a = 1.0f;

            marker_array.markers.push_back(centroid_marker);
        }

        return marker_array;
    }

    // クラスタIDに応じた色を返す
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
