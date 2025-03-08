#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include "sllidar_ros2/dbscan.hpp"

class Dbscan : public rclcpp::Node
{
public:
    Dbscan() : Node("dbscan_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Dbscan::filter_callback, this, std::placeholders::_1)
        );

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "dbscan_clusters", 10
        );
    }

private:
    // 前フレームに可視化していたクラスタIDを保持する
    // （ここでは「0以上のクラスターID + -1(ノイズ)」を入れておく）
    std::unordered_set<int> prev_cluster_ids_;

    // フィルタコールバック
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // LaserScan -> (x, y)点列
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

        // DBSCAN の結果 (クラスタリングされた各点: coords, label 等が含まれる)
        const auto &clustered_points = dbscan.getPoints();

        // =========================
        // 今フレームのクラスタID一覧を抽出
        // =========================
        //   - label >= 0 は通常のクラスタ
        //   - label == -1 は “ノイズ” として扱う
        std::unordered_set<int> new_cluster_ids;
        bool has_noise = false;

        int max_label = getMaxLabel(clustered_points);
        for (auto &p : clustered_points)
        {
            // クラスタID（-1はノイズ）
            if (p.label >= 0)
            {
                new_cluster_ids.insert(p.label);
            }
            else
            {
                // ノイズがあるフレームでのみ “has_noise” フラグを立てる
                has_noise = true;
            }
        }

        // ----------------------------
        // 不要になったIDを削除する (DELETE)
        // ----------------------------
        //  前フレームで可視化していたIDの中で、今回いなくなったIDを探す
        visualization_msgs::msg::MarkerArray marker_array; // まずここに削除コマンドを貯める

        for (int old_id : prev_cluster_ids_)
        {
            // old_id == -1 はノイズ用ID扱いとする
            // （ノイズ用Markerは固定で「id=9999」として可視化する想定）
            // そのため “old_id=-1” だったら “今フレームもノイズがなければ削除” みたいにする
            if (old_id == -1)
            {
                if (!has_noise)
                {
                    // 前フレームはノイズがあったが今回ノイズが無いなら削除
                    deleteMarkersForID(marker_array, NOISE_MARKER_ID);
                }
            }
            else
            {
                // old_id >= 0 のクラスタで、今回いなくなったIDは削除
                if (new_cluster_ids.count(old_id) == 0)
                {
                    deleteMarkersForID(marker_array, old_id);
                }
            }
        }

        // ----------------------------
        // 今フレームに存在するIDを “追加 or 更新”(ADD)
        // ----------------------------
        // 1) クラスタ点群(POINTS)
        {
            // createClusterMarkers(...) では
            //   * 通常クラスタID (0..max_label) ごとにマーカーを生成
            //   * ノイズは別ID (NOISE_MARKER_ID) で一括して可視化
            auto clusters = createClusterMarkers(clustered_points);
            for (auto &m : clusters.markers)
            {
                marker_array.markers.push_back(m);
            }
        }

        // 2) 各クラスタの重心(SPHERE)
        {
            auto centroids = createCentroidMarkers(clustered_points);
            for (auto &m : centroids.markers)
            {
                marker_array.markers.push_back(m);
            }
        }

        // 3) 各クラスタの円(Line Strip)
        {
            auto circles = createCircleMarkers(clustered_points);
            for (auto &m : circles.markers)
            {
                marker_array.markers.push_back(m);
            }
        }

        // ----------------------------
        // Publish & prev_cluster_ids_ 更新
        // ----------------------------
        marker_pub_->publish(marker_array);

        // 今回表示したIDを保存する
        std::unordered_set<int> current_ids = new_cluster_ids;
        if (has_noise)
        {
            // ノイズ(-1) も今回表示したとみなす
            current_ids.insert(-1);
        }
        prev_cluster_ids_ = current_ids;
    }

    // =================================================
    // クラスタ点群(POINTS)を可視化するMarker
    // =================================================
    visualization_msgs::msg::MarkerArray createClusterMarkers(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // まずクラスタIDの最大値を取得 (-1はノイズなので除外する)
        int max_label = getMaxLabel(points);

        // クラスタ (0 ～ max_label) の POINTS を作成
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "laser_frame";
            marker.header.stamp = this->now();
            marker.ns = "dbscan_clusters";
            marker.id = cluster_id;  // ←クラスタIDをそのまま利用
            marker.type = visualization_msgs::msg::Marker::POINTS;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.color = idToColor(cluster_id);

            for (auto &p : points)
            {
                if (p.label == cluster_id)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = p.coords[0];
                    pt.y = p.coords[1];
                    pt.z = 0.0;
                    marker.points.push_back(pt);
                }
            }

            // クラスタに属する点が無ければスキップ(=空マーカーは送らない)
            if (!marker.points.empty()) {
                marker_array.markers.push_back(marker);
            }
        }

        // ノイズ(-1) を可視化（ID固定: NOISE_MARKER_ID）
        //   ※ max_label+1 等を使うとフレームごとにIDが変わってしまい、
        //     差分削除の管理がややこしくなるので固定値を使う。
        {
            // ノイズ点を集める
            std::vector<geometry_msgs::msg::Point> noise_pts;
            for (auto &p : points)
            {
                if (p.label == -1)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = p.coords[0];
                    pt.y = p.coords[1];
                    pt.z = 0.0;
                    noise_pts.push_back(pt);
                }
            }
            if (!noise_pts.empty())
            {
                visualization_msgs::msg::Marker noise_marker;
                noise_marker.header.frame_id = "laser_frame";
                noise_marker.header.stamp = this->now();
                noise_marker.ns = "dbscan_clusters";
                noise_marker.id = NOISE_MARKER_ID; // 固定ID
                noise_marker.type = visualization_msgs::msg::Marker::POINTS;
                noise_marker.action = visualization_msgs::msg::Marker::ADD;
                noise_marker.pose.orientation.w = 1.0;
                noise_marker.scale.x = 0.01;
                noise_marker.scale.y = 0.01;
                noise_marker.color.r = 0.5f;
                noise_marker.color.g = 0.5f;
                noise_marker.color.b = 0.5f;
                noise_marker.color.a = 1.0f;
                noise_marker.points = noise_pts;

                marker_array.markers.push_back(noise_marker);
            }
        }

        return marker_array;
    }

    // =================================================
    // 各クラスタ重心(SPHERE)を表示するMarker
    // =================================================
    visualization_msgs::msg::MarkerArray createCentroidMarkers(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int max_label = getMaxLabel(points);

        // クラスタごとの (x,y)合計 と 個数 を集計 (label>=0)
        std::vector<double> sum_x(max_label + 1, 0.0);
        std::vector<double> sum_y(max_label + 1, 0.0);
        std::vector<int> count(max_label + 1, 0);

        for (auto &p : points) {
            if (p.label >= 0)
            {
                sum_x[p.label] += p.coords[0];
                sum_y[p.label] += p.coords[1];
                count[p.label] += 1;
            }
        }

        // 重心をSPHEREで可視化
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            if (count[cluster_id] <= 0) {
                continue;
            }

            double cx = sum_x[cluster_id] / count[cluster_id];
            double cy = sum_y[cluster_id] / count[cluster_id];

            visualization_msgs::msg::Marker centroid_marker;
            centroid_marker.header.frame_id = "laser_frame";
            centroid_marker.header.stamp = this->now();
            centroid_marker.ns = "dbscan_centroids";
            centroid_marker.id = cluster_id;         
            centroid_marker.type = visualization_msgs::msg::Marker::SPHERE; 
            centroid_marker.action = visualization_msgs::msg::Marker::ADD;

            centroid_marker.pose.position.x = cx;
            centroid_marker.pose.position.y = cy;
            centroid_marker.pose.position.z = 0.0;
            centroid_marker.pose.orientation.w = 1.0;

            centroid_marker.scale.x = 0.03;  // 適度なサイズ
            centroid_marker.scale.y = 0.03;
            centroid_marker.scale.z = 0.03;

            centroid_marker.color.r = 0.0f;
            centroid_marker.color.g = 0.0f;
            centroid_marker.color.b = 0.0f;
            centroid_marker.color.a = 1.0f;

            marker_array.markers.push_back(centroid_marker);
        }

        return marker_array;
    }

    // =====================================================
    // 重心から最も遠い点の距離を半径とする円(Line Strip)を可視化
    // =====================================================
    visualization_msgs::msg::MarkerArray createCircleMarkers(const std::vector<Point> &points)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int max_label = getMaxLabel(points);

        // クラスタごとの (x,y)合計 と 個数 を集計
        std::vector<double> sum_x(max_label + 1, 0.0);
        std::vector<double> sum_y(max_label + 1, 0.0);
        std::vector<int> count(max_label + 1, 0);

        // クラスタごとの最大距離(＝0 で初期化)
        std::vector<double> max_dist(max_label + 1, 0.0);

        // まず各クラスタに属する点の合計をとる
        for (auto &p : points) {
            if (p.label >= 0) {
                sum_x[p.label] += p.coords[0];
                sum_y[p.label] += p.coords[1];
                count[p.label] += 1;
            }
        }

        // 重心を求め、そこから各点までの距離の最大値を取得
        for (auto &p : points) {
            if (p.label >= 0) {
                int cid = p.label;
                double cx = sum_x[cid] / count[cid];
                double cy = sum_y[cid] / count[cid];
                double dx = p.coords[0] - cx;
                double dy = p.coords[1] - cy;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > max_dist[cid]) {
                    max_dist[cid] = dist;
                }
            }
        }

        // max_dist[cid] を半径とする円を描画
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            if (count[cluster_id] <= 0) {
                continue;
            }

            double cx = sum_x[cluster_id] / count[cluster_id];
            double cy = sum_y[cluster_id] / count[cluster_id];
            double radius = max_dist[cluster_id];

            // 円をLINE_STRIPで可視化 (36分割=10度刻み)
            visualization_msgs::msg::Marker circle_marker;
            circle_marker.header.frame_id = "laser_frame";
            circle_marker.header.stamp = this->now();
            circle_marker.ns = "dbscan_circles";
            circle_marker.id = cluster_id;         
            circle_marker.type = visualization_msgs::msg::Marker::LINE_STRIP; 
            circle_marker.action = visualization_msgs::msg::Marker::ADD;

            circle_marker.scale.x = 0.005;  
            // 黒色
            circle_marker.color.r = 0.0f;
            circle_marker.color.g = 0.0f;
            circle_marker.color.b = 0.0f;
            circle_marker.color.a = 1.0f;

            const int N = 36; 
            for (int i = 0; i <= N; ++i) {
                double theta = 2.0 * M_PI * double(i) / double(N);
                double px = cx + radius * std::cos(theta);
                double py = cy + radius * std::sin(theta);

                geometry_msgs::msg::Point pt;
                pt.x = px;
                pt.y = py;
                pt.z = 0.0;
                circle_marker.points.push_back(pt);
            }

            marker_array.markers.push_back(circle_marker);
        }

        return marker_array;
    }

    // -------------------------
    // 指定 ID のマーカーを削除
    // （3つの ns それぞれに対して）
    // -------------------------
    void deleteMarkersForID(visualization_msgs::msg::MarkerArray &marker_array, int id)
    {
        // クラスタ点群
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "laser_frame";
            m.header.stamp = this->now();
            m.ns = "dbscan_clusters";
            m.id = id;
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
        // 重心
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "laser_frame";
            m.header.stamp = this->now();
            m.ns = "dbscan_centroids";
            m.id = id;
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
        // 円
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "laser_frame";
            m.header.stamp = this->now();
            m.ns = "dbscan_circles";
            m.id = id;
            m.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(m);
        }
    }

    // 最大のクラスタIDを返す (ラベルが -1 の点はノイズなので除外)
    int getMaxLabel(const std::vector<Point> &points) const
    {
        int max_label = -1;
        for (auto &p : points) {
            if (p.label > max_label) {
                max_label = p.label;
            }
        }
        // すべて -1 の場合は max_label は -1 のまま(=クラスタ無し)
        return (max_label < 0) ? -1 : max_label;
    }

    // クラスタIDに応じた色
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

    static constexpr int NOISE_MARKER_ID = 9999; // ノイズ用に固定IDを割り当てる

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
