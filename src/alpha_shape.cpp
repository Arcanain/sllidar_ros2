#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include "sllidar_ros2/dbscan.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>

//点を小さくする
//alphaってなんだ？
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

        hull_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "concave_hull_markers", 10
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
        double eps = 0.05;     // 近傍閾値 (m)
        int min_samples = 10; // コア点判定に必要な最小点数
        DBSCAN dbscan(eps, min_samples);
        dbscan.fit(scan_points);

        // DBSCAN の結果 (クラスタリングされた各点: coords, label 等が含まれる)
        const auto &clustered_points = dbscan.getPoints();

        // =========================
        // 今フレームのクラスタID一覧を抽出
        // =========================
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
        visualization_msgs::msg::MarkerArray marker_array; // まず削除コマンド用の配列

        for (int old_id : prev_cluster_ids_)
        {
            if (old_id == -1)
            {
                if (!has_noise)
                {
                    // 前フレームはノイズがあったが今回は無いので削除
                    deleteMarkersForID(marker_array, NOISE_MARKER_ID);
                }
            }
            else
            {
                // old_id >= 0 のクラスタで今回いなくなったIDは削除
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
            auto cluster_markers = createClusterMarkers(clustered_points);
            for (auto &m : cluster_markers.markers)
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

        // 4) ConcaveHull(凹包) の可視化
        {
            double alpha_param = 0.1; // 必要に応じて調整
            auto hulls = createConcaveHullMarkers(clustered_points, alpha_param);
            hull_marker_pub_->publish(hulls);
            //for (auto &m : hulls.markers)
            //{
                //marker_array.markers.push_back(m);
            //}
        }

        // ----------------------------
        // Publish & prev_cluster_ids_ 更新
        // ----------------------------
        marker_pub_->publish(marker_array);

        // 今回表示したIDを保存する
        std::unordered_set<int> current_ids = new_cluster_ids;
        if (has_noise)
        {
            current_ids.insert(-1);
        }
        prev_cluster_ids_ = current_ids;
    }

    visualization_msgs::msg::MarkerArray createClusterMarkers(const std::vector<Point> &points)
{
    visualization_msgs::msg::MarkerArray marker_array;

    int max_label = getMaxLabel(points);

    // クラスタ (0 ～ max_label) の POINTS
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

        // クラスタに属する点が無ければスキップ
        if (!marker.points.empty())
        {
            marker_array.markers.push_back(marker);
        }
    }

    // ノイズ(-1) 用（ID固定: NOISE_MARKER_ID）
    {
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
            noise_marker.id = NOISE_MARKER_ID;
            noise_marker.type = visualization_msgs::msg::Marker::POINTS;
            noise_marker.action = visualization_msgs::msg::Marker::ADD;
            noise_marker.pose.orientation.w = 1.0;
            noise_marker.scale.x = 0.01;
            noise_marker.scale.y = 0.01;
            noise_marker.color.r = 0.5f;
            noise_marker.color.g = 0.0f;
            noise_marker.color.b = 0.0f;
            noise_marker.color.a = 0.0f;
            noise_marker.points = noise_pts;

            marker_array.markers.push_back(noise_marker);
        }
    }

    // ※ ここでは「円(LineStrip)」「ConcaveHull」の追加や、publishは行わない

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

            //重心は表示しない
            //marker_array.markers.push_back(centroid_marker);
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

            //一旦、円は表示しない
            //marker_array.markers.push_back(circle_marker);
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

// 凹包(Concave Hull)の輪郭線を可視化するMarker生成
    visualization_msgs::msg::MarkerArray createConcaveHullMarkers(
        const std::vector<Point> &points,
        double alpha = 0.01)
    {
        visualization_msgs::msg::MarkerArray marker_array;

        int max_label = getMaxLabel(points);
        if (max_label < 0) {
            // 全部ノイズ(-1)の場合はクラスタ無し→何も返さない
            return marker_array;
        }

        // 各クラスタIDについてConcaveHullを計算
        for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id)
        {
            // 1) このクラスタの点だけを集めてPCL::PointCloudを生成
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto &p : points)
            {
                if (p.label == cluster_id)
                {
                    pcl::PointXYZ pt;
                    pt.x = p.coords[0];
                    pt.y = p.coords[1];
                    pt.z = 0.0f;
                    cloud->points.push_back(pt);
                }
            }
            if (cloud->points.size() < 3) {
                // 凹包を作るには最低3点必要
                continue;
            }

            // 2) ConcaveHullオブジェクトを生成・設定
            pcl::ConcaveHull<pcl::PointXYZ> chull;
            chull.setInputCloud(cloud);
            chull.setAlpha(alpha);
            //chull.setKeepInformation(true);
            chull.setDimension(2); // 2Dとして処理(実際にはxyzで持っていてもOK)

            // 3) 計算結果を受け取る変数
            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::Vertices> polygons; // 各ポリゴンを構成する頂点インデックス

            try {
                chull.reconstruct(*hull_cloud, polygons);
            }
            catch (std::exception &e) {
                RCLCPP_WARN(rclcpp::get_logger("dbscan_node"),
                            "ConcaveHull failed for cluster %d: %s", cluster_id, e.what());
                continue;
            }

            // 4) Polygons を可視化: LINE_STRIPでポリゴン外周を描く
            //    ConcaveHullは複数のポリゴン(holes含む)が出てくる場合もあるためループ
            for (size_t i = 0; i < polygons.size(); ++i)
            {
                const auto &poly = polygons[i];

                // Marker準備
                visualization_msgs::msg::Marker hull_marker;
                hull_marker.header.frame_id = "laser_frame";
                hull_marker.header.stamp = rclcpp::Clock().now();
                hull_marker.ns = "dbscan_concave_hull";
                // 各ポリゴンに対してIDを振る
                // クラスタIDをベースに少しオフセットをかければ衝突しない
                hull_marker.id = cluster_id * 100 + i; 
                hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                hull_marker.action = visualization_msgs::msg::Marker::ADD;

                // スケール (線の太さ)
                hull_marker.scale.x = 0.01;
                // 色 (クラスタIDに応じて変化させても良い)
                hull_marker.color.r = 1.0f;
                hull_marker.color.g = 0.0f;
                hull_marker.color.b = 1.0f;
                hull_marker.color.a = 1.0f;

                for (size_t j = 0; j < poly.vertices.size(); ++j) {
                    int idx1 = poly.vertices[j];
                    int idx2 = poly.vertices[(j + 1) % poly.vertices.size()];
                    
                    if (idx1 < hull_cloud->points.size() && idx2 < hull_cloud->points.size()) {
                        auto& p1 = hull_cloud->points[idx1];
                        auto& p2 = hull_cloud->points[idx2];
            
                        // 補間: 10点で滑らかな線を描く
                        for (int k = 0; k <= 10; ++k) {
                            geometry_msgs::msg::Point pt;
                            double t = static_cast<double>(k) / 10.0; // tの値を使って補間
                            pt.x = (1 - t) * p1.x + t * p2.x;
                            pt.y = (1 - t) * p1.y + t * p2.y;
                            pt.z = 0.0;
                            hull_marker.points.push_back(pt);
                        }
                    }
                }

                //頂点をマーカーに追加
                for (auto idx : poly.vertices)
                {
                    if (idx < hull_cloud->points.size()) {
                        geometry_msgs::msg::Point pt;
                        pt.x = hull_cloud->points[idx].x;
                        pt.y = hull_cloud->points[idx].y;
                        pt.z = hull_cloud->points[idx].z;
                        hull_marker.points.push_back(pt);
                    }
                }
                // ポリゴンの外周を閉じるため、最初の点をもう一度push_back
                if (!hull_marker.points.empty()) {
                    hull_marker.points.push_back(hull_marker.points.front());
                }

                marker_array.markers.push_back(hull_marker);
            }
        }
        return marker_array;
    }


    static constexpr int NOISE_MARKER_ID = 9999; // ノイズ用に固定IDを割り当てる

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dbscan>());
    rclcpp::shutdown();
    return 0;
}
