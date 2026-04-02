#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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
 *   /wall_detection/filtered_points  (sensor_msgs/PointCloud2) - フィルタリングされた前方点群
 *   /wall_detection/ransac_line  (geometry_msgs/PolygonStamped) - RANSAC結果の直線上の点群
 *   /wall_detection/ransac_params  (std_msgs/Float64MultiArray) - RANSAC結果パラメータ [a, b, c, inliers_count]
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
        declare_parameter<double>("min_inlier_ratio", 0.35);
        declare_parameter<double>("line_lpf_alpha", 0.35);
        declare_parameter<double>("angle_lpf_alpha", 0.25);
        declare_parameter<double>("max_angle_step_deg", 8.0);

        const std::string scan_topic = get_parameter("scan_topic").as_string();
        fov_half_rad_ = get_parameter("fov_half_deg").as_double() * M_PI / 180.0;
        ransac_iterations_ = get_parameter("ransac_iterations").as_int();
        ransac_threshold_ = get_parameter("ransac_threshold").as_double();
        ransac_min_inliers_ = get_parameter("ransac_min_inliers").as_int();
        min_inlier_ratio_ = get_parameter("min_inlier_ratio").as_double();
        line_lpf_alpha_ = std::clamp(get_parameter("line_lpf_alpha").as_double(), 0.0, 1.0);
        angle_lpf_alpha_ = std::clamp(get_parameter("angle_lpf_alpha").as_double(), 0.0, 1.0);
        max_angle_step_rad_ = std::max(0.0, get_parameter("max_angle_step_deg").as_double()) * M_PI / 180.0;

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10,
            std::bind(&WallDetectionNode::scan_callback, this, std::placeholders::_1));

        angle_pub_ = create_publisher<std_msgs::msg::Float64>("/wall_detection/angle", 10);
        filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/wall_detection/filtered_points", 10);
        ransac_line_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("/wall_detection/ransac_line", 10);
        ransac_params_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/wall_detection/ransac_params", 10);

        RCLCPP_INFO(get_logger(), "wall_detection_node started. Subscribing to '%s'", scan_topic.c_str());
        RCLCPP_INFO(get_logger(), "Front FOV: ±%.1f deg, RANSAC iterations: %d, threshold: %.3f m",
                    get_parameter("fov_half_deg").as_double(), ransac_iterations_, ransac_threshold_);
        RCLCPP_INFO(get_logger(), "Stabilization: min_ratio=%.2f, line_lpf=%.2f, angle_lpf=%.2f, max_step=%.1f deg",
                    min_inlier_ratio_, line_lpf_alpha_, angle_lpf_alpha_, max_angle_step_rad_ * 180.0 / M_PI);
    }

private:
    // ライン当てはめ結果
    struct LineResult {
        double a{0.0};  ///< 正規化直線係数 a (ax + by + c = 0)
        double b{0.0};  ///< 正規化直線係数 b
        double c{0.0};  ///< 正規化直線係数 c
        int inliers{0}; ///< インライア数
    };

    double fov_half_rad_{M_PI / 4.0};
    int ransac_iterations_{200};
    double ransac_threshold_{0.05};
    int ransac_min_inliers_{10};
    double min_inlier_ratio_{0.35};
    double line_lpf_alpha_{0.35};
    double angle_lpf_alpha_{0.25};
    double max_angle_step_rad_{8.0 * M_PI / 180.0};
    std::mt19937 rng_;

    bool has_smoothed_line_{false};
    LineResult smoothed_line_{};
    bool has_filtered_angle_{false};
    double filtered_wall_angle_{0.0};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr ransac_line_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ransac_params_pub_;

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
        const LineResult ransac_result = ransac_line_fit(points);

        const double inlier_ratio = static_cast<double>(ransac_result.inliers) / static_cast<double>(points.size());
        if (ransac_result.inliers < ransac_min_inliers_ || inlier_ratio < min_inlier_ratio_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "RANSAC失敗: インライア不足 (%d / %zu, ratio=%.2f)",
                                 ransac_result.inliers, points.size(), inlier_ratio);
            return;
        }

        // RANSACインライアでTLS再フィットし、その後に時系列平滑化
        LineResult result = refine_line_with_inliers(points, ransac_result);
        result = smooth_line_temporally(result);

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
        const double raw_wall_angle = std::atan2(nb, na);
        const double wall_angle = smooth_angle(raw_wall_angle);

        RCLCPP_INFO(get_logger(),
                    "壁傾き角: %.4f rad (%.2f deg) | インライア: %d / %zu 点",
                    wall_angle, wall_angle * 180.0 / M_PI, result.inliers, points.size());

        std_msgs::msg::Float64 out;
        out.data = wall_angle;
        angle_pub_->publish(out);

        // フィルタリングされた点群をPointCloud2で発行
        publish_filtered_points(msg, points);

        // RANSAC結果の直線をPolygonStampedで発行
        publish_ransac_line(msg, result, points);

        // RANSACパラメータを発行
        publish_ransac_params(result, points.size());
    }

    /**
     * @brief フィルタリングされた点群をPointCloud2形式で発行する
     */
    void publish_filtered_points(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                 const std::vector<std::pair<double, double>> &points) {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = msg->header.stamp;
        cloud.header.frame_id = msg->header.frame_id;
        cloud.height = 1;
        cloud.width = points.size();
        cloud.is_dense = true;
        cloud.fields.resize(2);

        cloud.fields[0].name = "x";
        cloud.fields[0].offset = 0;
        cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[0].count = 1;

        cloud.fields[1].name = "y";
        cloud.fields[1].offset = 4;
        cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[1].count = 1;

        cloud.point_step = 8;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        for (const auto &[x, y] : points) {
            *iter_x = static_cast<float>(x);
            *iter_y = static_cast<float>(y);
            ++iter_x;
            ++iter_y;
        }

        filtered_cloud_pub_->publish(cloud);
    }

    /**
     * @brief RANSAC結果の直線をPolygonStampedで発行する
     * 直線上の2点を計算して発行
     */
    void publish_ransac_line(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                             const LineResult &result,
                             const std::vector<std::pair<double, double>> &points) {
        geometry_msgs::msg::PolygonStamped polygon;
        polygon.header.stamp = msg->header.stamp;
        polygon.header.frame_id = msg->header.frame_id;

        // 直線 ax + by + c = 0 の外側の2点を計算
        // グラフの表示範囲を仮定して直線との交点を計算
        double x_min = 10.0, x_max = -10.0;
        double y_min = 10.0, y_max = -10.0;
        for (const auto &[x, y] : points) {
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }

        // 余裕をもたせる
        x_min -= 0.5;
        x_max += 0.5;
        y_min -= 0.5;
        y_max += 0.5;

        // 直線上の2点を計算
        std::vector<geometry_msgs::msg::Point32> points_on_line;
        const double eps = 1e-6;

        // b != 0 の場合: y = -(ax + c) / b
        if (std::abs(result.b) > eps) {
            double y_at_xmin = -(result.a * x_min + result.c) / result.b;
            double y_at_xmax = -(result.a * x_max + result.c) / result.b;

            if (y_at_xmin >= y_min && y_at_xmin <= y_max) {
                geometry_msgs::msg::Point32 p;
                p.x = static_cast<float>(x_min);
                p.y = static_cast<float>(y_at_xmin);
                p.z = 0.0f;
                points_on_line.push_back(p);
            }
            if (y_at_xmax >= y_min && y_at_xmax <= y_max) {
                geometry_msgs::msg::Point32 p;
                p.x = static_cast<float>(x_max);
                p.y = static_cast<float>(y_at_xmax);
                p.z = 0.0f;
                points_on_line.push_back(p);
            }
        }
        // a != 0 の場合: x = -(by + c) / a
        if (std::abs(result.a) > eps) {
            double x_at_ymin = -(result.b * y_min + result.c) / result.a;
            double x_at_ymax = -(result.b * y_max + result.c) / result.a;

            if (x_at_ymin >= x_min && x_at_ymin <= x_max) {
                geometry_msgs::msg::Point32 p;
                p.x = static_cast<float>(x_at_ymin);
                p.y = static_cast<float>(y_min);
                p.z = 0.0f;
                points_on_line.push_back(p);
            }
            if (x_at_ymax >= x_min && x_at_ymax <= x_max) {
                geometry_msgs::msg::Point32 p;
                p.x = static_cast<float>(x_at_ymax);
                p.y = static_cast<float>(y_max);
                p.z = 0.0f;
                points_on_line.push_back(p);
            }
        }

        // 重複排除と取得
        if (points_on_line.size() >= 2) {
            polygon.polygon.points.push_back(points_on_line[0]);
            polygon.polygon.points.push_back(points_on_line[1]);
        }

        ransac_line_pub_->publish(polygon);
    }

    /**
     * @brief RANSACパラメータを発行する
     */
    void publish_ransac_params(const LineResult &result, size_t total_points) {
        std_msgs::msg::Float64MultiArray params;
        params.data = {result.a, result.b, result.c, static_cast<double>(result.inliers), static_cast<double>(total_points)};
        ransac_params_pub_->publish(params);
    }

    /**
     * @brief 2D点群に対してRANSACで直線当てはめを行う。
     * @param points 入力点群 [(x, y), ...]
     * @return 最良直線の係数とインライア数
     */
    LineResult ransac_line_fit(const std::vector<std::pair<double, double>> &points) {
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
            for (const auto &[x, y] : points) {
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

    LineResult refine_line_with_inliers(const std::vector<std::pair<double, double>> &points,
                                        const LineResult &seed) const {
        std::vector<std::pair<double, double>> inliers;
        inliers.reserve(points.size());

        for (const auto &[x, y] : points) {
            if (std::abs(seed.a * x + seed.b * y + seed.c) < ransac_threshold_) {
                inliers.emplace_back(x, y);
            }
        }

        if (inliers.size() < 2) {
            return seed;
        }

        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto &[x, y] : inliers) {
            mean_x += x;
            mean_y += y;
        }
        mean_x /= static_cast<double>(inliers.size());
        mean_y /= static_cast<double>(inliers.size());

        double sxx = 0.0;
        double syy = 0.0;
        double sxy = 0.0;
        for (const auto &[x, y] : inliers) {
            const double dx = x - mean_x;
            const double dy = y - mean_y;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }

        // PCA方向: 最大固有値方向を線の向きベクトルとする
        const double theta = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
        const double dir_x = std::cos(theta);
        const double dir_y = std::sin(theta);

        // 線の法線ベクトル (a, b) は方向ベクトルと直交
        double a = -dir_y;
        double b = dir_x;
        double c = -(a * mean_x + b * mean_y);

        const double norm = std::sqrt(a * a + b * b);
        if (norm < 1e-9) {
            return seed;
        }
        a /= norm;
        b /= norm;
        c /= norm;

        int inlier_count = 0;
        for (const auto &[x, y] : points) {
            if (std::abs(a * x + b * y + c) < ransac_threshold_) {
                ++inlier_count;
            }
        }

        return {a, b, c, inlier_count};
    }

    LineResult smooth_line_temporally(const LineResult &current) {
        LineResult next = current;

        if (has_smoothed_line_) {
            // 直線の符号反転対策: 法線向きを前回と揃える
            if (smoothed_line_.a * next.a + smoothed_line_.b * next.b < 0.0) {
                next.a = -next.a;
                next.b = -next.b;
                next.c = -next.c;
            }

            next.a = (1.0 - line_lpf_alpha_) * smoothed_line_.a + line_lpf_alpha_ * next.a;
            next.b = (1.0 - line_lpf_alpha_) * smoothed_line_.b + line_lpf_alpha_ * next.b;
            next.c = (1.0 - line_lpf_alpha_) * smoothed_line_.c + line_lpf_alpha_ * next.c;

            const double norm = std::sqrt(next.a * next.a + next.b * next.b);
            if (norm > 1e-9) {
                next.a /= norm;
                next.b /= norm;
                next.c /= norm;
            }
        }

        smoothed_line_ = next;
        has_smoothed_line_ = true;
        return next;
    }

    static double normalize_angle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    double smooth_angle(double raw_angle) {
        if (!has_filtered_angle_) {
            filtered_wall_angle_ = raw_angle;
            has_filtered_angle_ = true;
            return filtered_wall_angle_;
        }

        double delta = normalize_angle(raw_angle - filtered_wall_angle_);
        if (max_angle_step_rad_ > 0.0) {
            delta = std::clamp(delta, -max_angle_step_rad_, max_angle_step_rad_);
        }

        filtered_wall_angle_ = normalize_angle(filtered_wall_angle_ + angle_lpf_alpha_ * delta);
        return filtered_wall_angle_;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
