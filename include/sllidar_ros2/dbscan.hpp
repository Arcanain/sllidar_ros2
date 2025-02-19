#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <iostream>

struct Point
{
    std::vector<double> coords;
    bool visited;
    int label;
    std::string pointType;

    Point(const std::vector<double> &c)
        : coords(c), visited(false), label(-1), pointType("UNDEFINED") {}

    void visit()
    {
        visited = true;
    }
};

class DBSCAN
{
public:
    DBSCAN(double eps, int minSamples)
        : eps_(eps), minSamples_(minSamples), currentLabel_(0) {}

    // DBSCANクラスタリングを実行する関数
    void fit(const std::vector<std::vector<double>> &X)
    {
        // 1) 入力データを Point に変換して保持
        points_.clear();
        points_.reserve(X.size());
        for (const auto &x : X)
        {
            points_.emplace_back(x);
        }

        // 2) 各点に対してコア点判定→クラスタ拡張
        for (auto &p : points_)
        {
            if (!p.visited)
            {
                p.visit();
                auto neighborPts = rangeQuery(p);

                if (neighborPts.size() < static_cast<size_t>(minSamples_))
                {
                    // 近傍点数が閾値未満ならノイズ
                    p.pointType = "NOISE";
                }
                else
                {
                    // コア点
                    p.pointType = "CORE";
                    expandCluster(p, neighborPts);
                    currentLabel_++;
                }
            }
        }
    }

    // 結果確認用: 各点のラベルや座標をコンソールに出力
    void printClusters() const
    {
        for (size_t i = 0; i < points_.size(); ++i)
        {
            const auto &p = points_[i];
            std::cout << "Point " << i
                      << ": label=" << p.label
                      << ", type=" << p.pointType
                      << ", coords=(";
            for (size_t d = 0; d < p.coords.size(); ++d)
            {
                std::cout << p.coords[d];
                if (d + 1 < p.coords.size())
                    std::cout << ",";
            }
            std::cout << ")\n";
        }
    }

    // fit() 後の points_ を取得するための公開関数
    const std::vector<Point> &getPoints() const
    {
        return points_;
    }

private:
    // クラスタを拡張
    void expandCluster(Point &p, std::vector<Point *> &neighborPts)
    {
        // p のラベルを現在のラベル値に設定
        p.label = currentLabel_;

        // 近傍リストを走査
        for (size_t i = 0; i < neighborPts.size(); ++i)
        {
            Point *q = neighborPts[i];
            if (!q->visited)
            {
                q->visit();
                auto neighborPts_q = rangeQuery(*q);
                if (neighborPts_q.size() >= static_cast<size_t>(minSamples_))
                {
                    q->pointType = "CORE";
                    // 新たに得られた近傍を neighborPts に追加
                    neighborPts.insert(neighborPts.end(), neighborPts_q.begin(), neighborPts_q.end());
                }
            }
            // q がまだクラスタ未割り当てなら
            if (q->label == -1)
            {
                q->label = currentLabel_;
            }
        }
    }

    // p から eps_ 以下の距離にある点を取得
    std::vector<Point *> rangeQuery(Point &p)
    {
        std::vector<Point *> neighbors;
        neighbors.reserve(points_.size());
        for (auto &candidate : points_)
        {
            if (distance(p, candidate) <= eps_)
            {
                neighbors.push_back(&candidate);
            }
        }
        return neighbors;
    }

    // 2点間の距離 (多次元対応)
    double distance(const Point &a, const Point &b)
    {
        double sum = 0.0;
        for (size_t i = 0; i < a.coords.size(); ++i)
        {
            double diff = a.coords[i] - b.coords[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

private:
    std::vector<Point> points_;
    double eps_;
    int minSamples_;
    int currentLabel_;
};

