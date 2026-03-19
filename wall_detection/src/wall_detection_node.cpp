#include <cmath>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"

/**
 * @brief RANSACを用いてLD19の前方点群から壁を検知し、機体と壁の傾き角を計算・配信するノード。
 *
 * サブスクライブ:
 *   /ldlidar_node/scan  (sensor_msgs/LaserScan) - LD19ライダーのスキャンデータ
 *
 * パブリッシュ:
 *   /wall_detection/angle  (std_msgs/Float64) - 壁の傾き角 [rad]
 *     ロボット前方(x軸)から壁法線への反時計回り角度
 *     正値: 壁が右に傾いている (反時計回り, CCW)
 *     負値: 壁が左に傾いている (時計回り, CW)
 *     0: 壁がロボット正面に垂直 (完全正対)
 */
class WallDetectionNode : public rclcpp::Node {
public:
    WallDetectionNode() : Node("wall_detection_node"), rng_(std::random_device{}()) {
        // パラメータ宣言
        declare_parameter<std::string>("scan_topic", "/ldlidar_node/scan");
        declare_parameter<double>("fov_half_deg", 45.0);
        declare_parameter<int>("ransac_iterations", 200);
        declare_parameter<double>("ransac_threshold", 0.05);
        declare_parameter<int>("ransac_min_inliers", 10);

        const std::string scan_topic = get_parameter("scan_topic").as_string();
        fov_half_rad_ = get_parameter("fov_half_deg").as_double() * M_PI / 180.0;
        ransac_iterations_ = get_parameter("ransac_iterations").as_int();
        ransac_threshold_ = get_parameter("ransac_threshold").as_double();
        ransac_min_inliers_ = get_parameter("ransac_min_inliers").as_int();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&WallDetectionNode::scan_callback, this, std::placeholders::_1));

        angle_pub_ = create_publisher<std_msgs::msg::Float64>("/wall_detection/angle", 10);

        RCLCPP_INFO(get_logger(), "wall_detection_node started. Subscribing to '%s'", scan_topic.c_str());
        RCLCPP_INFO(get_logger(), "Front FOV: ±%.1f deg, RANSAC iterations: %d, threshold: %.3f m",
            get_parameter("fov_half_deg").as_double(), ransac_iterations_, ransac_threshold_);
    }

private:
    // ライン当てはめ結果
    struct LineResult {
        double a{0.0};       ///< 正規化直線係数 a (ax + by + c = 0)
        double b{0.0};       ///< 正規化直線係数 b
        double c{0.0};       ///< 正規化直線係数 c
        int inliers{0};      ///< インライア数
    };

    double fov_half_rad_{M_PI / 4.0};
    int ransac_iterations_{200};
    double ransac_threshold_{0.05};
    int ransac_min_inliers_{10};
    std::mt19937 rng_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 前方FOV内の有効点をデカルト座標に変換
        std::vector<std::pair<double, double>> points;
        points.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            // 前方FOV外はスキップ
            if (std::abs(angle) > fov_half_rad_) {
                continue;
            }

            const double range = msg->ranges[i];
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
                continue;
            }

            points.emplace_back(range * std::cos(angle), range * std::sin(angle));
        }

        if (static_cast<int>(points.size()) < std::max(ransac_min_inliers_, 2)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "前方有効点が不足しています (%zu 点, 最低 %d 点必要)",
                points.size(), ransac_min_inliers_);
            return;
        }

        // RANSACによる直線当てはめ
        const LineResult result = ransac_line_fit(points);

        if (result.inliers < ransac_min_inliers_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "RANSAC失敗: インライア数が不足しています (%d 点)", result.inliers);
            return;
        }

        // 直線 ax + by + c = 0 において法線ベクトルは (a, b)
        // ロボット前方(x軸正方向)に対する壁法線の傾き角を計算する
        //
        // 壁が完全にロボット前方に垂直(ロボットが壁を正面から正対)な場合:
        //   直線は y 方向に伸びる → a=±1, b=0 → 傾き角 = 0
        //
        // wall_angle = atan2(b, a): ロボット前方(x軸)から壁法線への反時計回り角度 [rad]
        //   壁法線が常にロボット側を向くよう a > 0 に正規化する
        //   正値: 壁が右に傾いている (CCW)
        //   負値: 壁が左に傾いている (CW)
        double na = result.a;
        double nb = result.b;
        if (na < 0.0) {
            na = -na;
            nb = -nb;
        }
        const double wall_angle = std::atan2(nb, na);

        RCLCPP_INFO(get_logger(),
            "壁傾き角: %.4f rad (%.2f deg) | インライア: %d / %zu 点",
            wall_angle, wall_angle * 180.0 / M_PI, result.inliers, points.size());

        std_msgs::msg::Float64 out;
        out.data = wall_angle;
        angle_pub_->publish(out);
    }

    /**
     * @brief 2D点群に対してRANSACで直線当てはめを行う。
     * @param points 入力点群 [(x, y), ...]
     * @return 最良直線の係数とインライア数
     */
    LineResult ransac_line_fit(const std::vector<std::pair<double, double>>& points) {
        LineResult best;
        std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

        for (int iter = 0; iter < ransac_iterations_; ++iter) {
            // ランダムに2点を選択
            size_t idx1 = dist(rng_);
            size_t idx2 = dist(rng_);
            while (idx2 == idx1) {
                idx2 = dist(rng_);
            }

            const double x1 = points[idx1].first;
            const double y1 = points[idx1].second;
            const double x2 = points[idx2].first;
            const double y2 = points[idx2].second;

            // 直線係数 ax + by + c = 0 を計算して正規化
            double a = y2 - y1;
            double b = x1 - x2;
            double c = -(a * x1 + b * y1);
            const double norm = std::sqrt(a * a + b * b);
            if (norm < 1e-6) {
                continue;
            }
            a /= norm;
            b /= norm;
            c /= norm;

            // インライア数を計算
            int inliers = 0;
            for (const auto& [x, y] : points) {
                if (std::abs(a * x + b * y + c) < ransac_threshold_) {
                    ++inliers;
                }
            }

            if (inliers > best.inliers) {
                best = {a, b, c, inliers};
            }
        }

        return best;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
