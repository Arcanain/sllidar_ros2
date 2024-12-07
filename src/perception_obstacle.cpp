#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>  // 追加
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
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/closest_point_marker", 10);  // 型をMarkerArrayに変更
        closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_point", 10);
        fov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fov_marker", 10);
        obstacle_detected_pub = this->create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);

        // 初期化
        threshold_ = 1.3;          // 円の半径の上限
        min_threshold_ = 0.45;      // 円の半径の下限
        forward_angle_min_ = -M_PI / 9; // 前方の最小角度 (-45度)
        forward_angle_max_ = M_PI / 9;  // 前方の最大角度 (+45度)
        num_points_ = 30;          // 扇形を構成する頂点の数
    }

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_msg = sensor_msgs::msg::LaserScan(*msg);
        float closest_distance = std::numeric_limits<float>::infinity();
        float closest_x = 0.0;
        float closest_y = 0.0;
        float closest_angle = 0.0;

        // 中心の座標 (円の中心)
        float center_x = 0.0;
        float center_y = 0.0;

        // 障害物検出フラグを初期化
        bool obstacle_detected = false;

        // フィルタリング処理と最も近い点の検索
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = msg->ranges[i] * cos(angle);
            float y = msg->ranges[i] * sin(angle);

            // 前方の角度範囲内かつ閾値以内の点群のみ残す
            if (angle >= -M_PI && angle <= M_PI)
            {
                float distance = std::hypot(x - center_x, y - center_y);
                if (distance < min_threshold_ || distance > threshold_)
                {
                    filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // フィルタリング
                }
                else
                {
                    if (angle >= forward_angle_min_ && angle <= forward_angle_max_)
                    {
                    	obstacle_detected = true;
                    }
                    
                    if (distance < closest_distance)
                    {
                    closest_distance = distance;
                    closest_x = x;
                    closest_y = y;
                    //closest_angle = angle;
                    }
                }
            }
            else
            {
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity(); // 範囲外の点群をフィルタリング
            }
        }

        // 最も近い点が見つかったかどうかをチェック
        if (closest_distance != std::numeric_limits<float>::infinity())
        {
            // 最も近い点が前方90度（±45度）内にあるかを確認
            //if (closest_angle >= forward_angle_min_ && closest_angle <= forward_angle_max_)
            //{
                //obstacle_detected = true;
            //}

            // 最も近い点のマーカーを生成
            visualization_msgs::msg::Marker closest_point_marker;
            closest_point_marker.header.frame_id = "base_link"; // フレームIDは適切なものを指定
            closest_point_marker.header.stamp = this->get_clock()->now();
            closest_point_marker.ns = "closest_point";
            closest_point_marker.id = 0;
            closest_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
            closest_point_marker.action = visualization_msgs::msg::Marker::ADD;
            closest_point_marker.pose.position.x = closest_x;
            closest_point_marker.pose.position.y = closest_y;
            closest_point_marker.pose.position.z = 0.0;
            closest_point_marker.pose.orientation.w = 1.0;
            closest_point_marker.scale.x = 0.1; // 球の大きさ
            closest_point_marker.scale.y = 0.1;
            closest_point_marker.scale.z = 0.1;
            closest_point_marker.color.a = 1.0;
            closest_point_marker.color.r = 0.0;
            closest_point_marker.color.g = 0.0;
            closest_point_marker.color.b = 1.0; // 青色

            // MarkerArrayに追加
            visualization_msgs::msg::MarkerArray marker_array;  // MarkerArrayを作成
            marker_array.markers.push_back(closest_point_marker);

            // マーカーをパブリッシュ
            marker_publisher_->publish(marker_array);

            // 最も近い点の座標をgeometry_msgs::msg::Pointとしてパブリッシュ
            geometry_msgs::msg::Point closest_point;
            closest_point.x = closest_x;
            closest_point.y = closest_y;
            closest_point.z = 0.0;
            closest_point_publisher_->publish(closest_point);
        }
        else
        {
            // マーカーを削除するために、アクションをDELETEに設定してパブリッシュ
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "base_link"; // フレームIDは適切なものを指定
            delete_marker.header.stamp = this->get_clock()->now();
            delete_marker.ns = "closest_point";
            delete_marker.id = 0;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;

            // MarkerArrayに追加
            visualization_msgs::msg::MarkerArray marker_array;  // MarkerArrayを作成
            marker_array.markers.push_back(delete_marker);

            // マーカーをパブリッシュ
            marker_publisher_->publish(marker_array);
        }

        // obstacle_detectedをパブリッシュ
        std_msgs::msg::Bool obstacle_msg;
        obstacle_msg.data = obstacle_detected;
        obstacle_detected_pub->publish(obstacle_msg);

        // 以下、他のコードはそのまま

        // カラー設定
        std_msgs::msg::ColorRGBA green_color;
        green_color.a = 0.5;
        green_color.r = 0.0;
        green_color.g = 1.0;
        green_color.b = 0.0;

        std_msgs::msg::ColorRGBA yellow_color;
        yellow_color.a = 0.5;
        yellow_color.r = 1.0;
        yellow_color.g = 1.0;
        yellow_color.b = 0.0;

        // フレームID
        std::string frame_id = "base_link";

        // 前方90度のマーカーを作成
        visualization_msgs::msg::Marker fov_marker_forward;
        createFOVMarker(fov_marker_forward, forward_angle_min_, forward_angle_max_, min_threshold_, threshold_, num_points_, frame_id, green_color);
        fov_marker_forward.ns = "fov";
        fov_marker_forward.id = 1;

        // 後方270度のマーカーを作成（-180度から-45度まで）
        visualization_msgs::msg::Marker fov_marker_backward1;
        createFOVMarker(fov_marker_backward1, -M_PI, forward_angle_min_, min_threshold_, threshold_, num_points_, frame_id, yellow_color);
        fov_marker_backward1.ns = "fov";
        fov_marker_backward1.id = 2;

        // 後方270度のマーカーを作成（+45度から+180度まで）
        visualization_msgs::msg::Marker fov_marker_backward2;
        createFOVMarker(fov_marker_backward2, forward_angle_max_, M_PI, min_threshold_, threshold_, num_points_, frame_id, yellow_color);
        fov_marker_backward2.ns = "fov";
        fov_marker_backward2.id = 3;

        // マーカーをパブリッシュ
        fov_marker_publisher_->publish(fov_marker_forward);
        fov_marker_publisher_->publish(fov_marker_backward1);
        fov_marker_publisher_->publish(fov_marker_backward2);

        // フィルタリング結果をパブリッシュ
        publisher_->publish(filtered_msg);
    }

    // マーカーを作成する関数を追加
    void createFOVMarker(visualization_msgs::msg::Marker &marker, float angle_min, float angle_max, float min_threshold, float threshold, size_t num_points, const std::string &frame_id, const std_msgs::msg::ColorRGBA &color)
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
        float angle_step = (angle_max - angle_min) / num_points;

        for (size_t i = 0; i <= num_points; ++i)
        {
            float angle = angle_min + i * angle_step;

            geometry_msgs::msg::Point p_inner;
            p_inner.x = min_threshold * cos(angle);
            p_inner.y = min_threshold * sin(angle);
            p_inner.z = 0.0;
            inner_points.push_back(p_inner);

            geometry_msgs::msg::Point p_outer;
            p_outer.x = threshold * cos(angle);
            p_outer.y = threshold * sin(angle);
            p_outer.z = 0.0;
            outer_points.push_back(p_outer);
        }

        // 内側と外側のポイント間に三角形を作成
        for (size_t i = 0; i < num_points; ++i)
        {
            // 第一の三角形
            marker.points.push_back(inner_points[i]);
            marker.points.push_back(outer_points[i]);
            marker.points.push_back(inner_points[i + 1]);

            // 第二の三角形
            marker.points.push_back(inner_points[i + 1]);
            marker.points.push_back(outer_points[i]);
            marker.points.push_back(outer_points[i + 1]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;  // 型をMarkerArrayに変更
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub;
    float threshold_;         // 円の半径の上限
    float min_threshold_;     // 円の半径の下限
    float forward_angle_min_; // 前方角度の最小値 (-45度)
    float forward_angle_max_; // 前方角度の最大値 (+45度)
    size_t num_points_;       // 扇形の頂点数
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilter>());
    rclcpp::shutdown();
    return 0;
}
