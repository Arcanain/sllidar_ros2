#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>

class PointCloudFilter : public rclcpp::Node
{
public:
    PointCloudFilter() : Node("perception_obstacle")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PointCloudFilter::filter_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered/scan", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/closest_point_marker", 10);
        closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_point", 10);
        fov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fov_marker", 10);
        obstacle_detected_pub = this->create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);

        // パラメータ初期化
        threshold_ = 1.5;     // 外側円（上限半径）
        min_threshold_ = 0.5; // 内側円（下限半径）
        y_min_ = -0.45;
        y_max_ = 0.45;
        num_points_ = 30; // サンプリング数
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // フィルタリングなし
        auto filtered_msg = sensor_msgs::msg::LaserScan(*msg);
        bool obstacle_detected = false;

        // 緑の図形: 
        // 条件： 
        //   min_threshold_ <= distance <= threshold_ 
        //   y_min_ <= y <= y_max_
        //   x >= 0 (右半分)
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float distance = msg->ranges[i];
            if (std::isfinite(distance)) {
                float angle = msg->angle_min + i * msg->angle_increment;
                float x = distance * std::cos(angle);
                float y = distance * std::sin(angle);

                if (distance >= min_threshold_ && distance <= threshold_ &&
                    y >= y_min_ && y <= y_max_ &&
                    x >= 0.0)
                {
                    obstacle_detected = true;
                    break;
                }
            }
        }

        // obstacle_detectedをパブリッシュ
        std_msgs::msg::Bool obstacle_msg;
        obstacle_msg.data = obstacle_detected;
        obstacle_detected_pub->publish(obstacle_msg);

        // カラー設定
        std_msgs::msg::ColorRGBA green_color;
        green_color.a = 0.5;
        green_color.r = 0.0;
        green_color.g = 1.0;
        green_color.b = 0.0;

        std::string frame_id = "laser_frame";

        // 2つの円弧（min_threshold_, threshold_）とy=±0.4で囲われた領域を作成
        visualization_msgs::msg::Marker fov_marker;
        createRotatedSliceMarker(fov_marker, min_threshold_, threshold_, y_min_, y_max_, num_points_, frame_id, green_color);

        fov_marker.ns = "fov";
        fov_marker.id = 1;
        fov_marker_publisher_->publish(fov_marker);

        // スキャンをそのままパブリッシュ
        publisher_->publish(filtered_msg);
    }

    void createRotatedSliceMarker(visualization_msgs::msg::Marker &marker, float min_threshold, float threshold,
                                  float y_min, float y_max, size_t num_points, const std::string &frame_id,
                                  const std_msgs::msg::ColorRGBA &color)
    {
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color = color;

        std::vector<geometry_msgs::msg::Point> inner_points;
        std::vector<geometry_msgs::msg::Point> outer_points;

        float dy = (y_max - y_min) / static_cast<float>(num_points);

        for (size_t i = 0; i <= num_points; ++i)
        {
            float y = y_min + i * dy;

            // 外側円弧(右半分)
            float x_outer = std::sqrt(threshold * threshold - y * y);
            geometry_msgs::msg::Point p_outer;
            p_outer.x = x_outer; // 右方向(正x)
            p_outer.y = y;
            p_outer.z = 0.0;
            outer_points.push_back(p_outer);

            // 内側円弧(右半分)
            float x_inner = std::sqrt(min_threshold * min_threshold - y * y);
            geometry_msgs::msg::Point p_inner;
            p_inner.x = x_inner;
            p_inner.y = y;
            p_inner.z = 0.0;
            inner_points.push_back(p_inner);
        }

        // inner_pointsとouter_pointsで三角形を形成
        for (size_t i = 0; i < num_points; ++i)
        {
            marker.points.push_back(inner_points[i]);
            marker.points.push_back(outer_points[i]);
            marker.points.push_back(inner_points[i + 1]);

            marker.points.push_back(inner_points[i + 1]);
            marker.points.push_back(outer_points[i]);
            marker.points.push_back(outer_points[i + 1]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub;
    float threshold_;      // 円の半径の上限
    float min_threshold_;  // 円の半径の下限
    float y_min_;          // 下側ライン(-0.4)
    float y_max_;          // 上側ライン(+0.4)
    size_t num_points_;    // サンプリング数
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}