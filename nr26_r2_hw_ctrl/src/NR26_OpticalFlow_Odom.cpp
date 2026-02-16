#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class OpticalFlowOdom : public rclcpp::Node {
public:
    OpticalFlowOdom()
        : Node("optical_flow_odom") {

        RCLCPP_INFO(get_logger(), "Node starting...");

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_FATAL(get_logger(), "Camera open failed");
            rclcpp::shutdown();
            return;
        }

        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        RCLCPP_INFO(get_logger(), "Camera opened successfully");

        last_time_ = now();
        timer_ = create_wall_timer(
            33ms,
            std::bind(&OpticalFlowOdom::process, this));

        RCLCPP_INFO(get_logger(), "Optical Flow Odometry Started");
    }

private:
    // ===== メイン処理 =====
    void process() {
        cv::Mat frame, gray;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Empty camera frame");
            return;
        }

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // ===== 初期化 =====
        if (!initialized_) {
            prev_gray_ = gray.clone();

            cv::goodFeaturesToTrack(
                prev_gray_,
                prev_pts_,
                300,   // max corners
                0.01,  // quality
                10.0); // min distance

            RCLCPP_INFO(
                get_logger(),
                "Initial feature points: %zu",
                prev_pts_.size());

            if (prev_pts_.empty()) {
                RCLCPP_WARN(get_logger(), "No feature points detected");
                return;
            }

            last_time_ = now();
            initialized_ = true;
            return;
        }

        // ===== Optical Flow =====
        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar> status;
        std::vector<float> err;

        cv::calcOpticalFlowPyrLK(
            prev_gray_,
            gray,
            prev_pts_,
            curr_pts,
            status,
            err);

        int tracked = 0;
        float dx_px = 0.0f;
        float dy_px = 0.0f;

        for (size_t i = 0; i < status.size(); i++) {
            if (status[i]) {
                dx_px += curr_pts[i].x - prev_pts_[i].x;
                dy_px += curr_pts[i].y - prev_pts_[i].y;
                tracked++;
            }
        }

        if (tracked == 0) {
            RCLCPP_WARN(get_logger(), "No tracked features");
            initialized_ = false;
            return;
        }

        dx_px /= tracked;
        dy_px /= tracked;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Tracked: %d / %zu | Pixel dx=%.2f dy=%.2f",
            tracked, status.size(), dx_px, dy_px);

        // ===== ピクセル → メートル変換 =====
        // 仮パラメータ（後で必ず調整）
        constexpr float CAMERA_HEIGHT = 0.18f; // [m]
        constexpr float FOV_Y = 60.0f * M_PI / 180.0f;

        float meters_per_pixel =
            2.0f * CAMERA_HEIGHT * tan(FOV_Y / 2.0f) / frame.rows;

        float dx = -dx_px * meters_per_pixel;
        float dy = dy_px * meters_per_pixel;

        // ===== 時間 =====
        auto now_t = now();
        double dt = (now_t - last_time_).seconds();
        last_time_ = now_t;

        if (dt <= 0.0)
            return;

        x_ += dx;
        y_ += dy;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[ODOM] X=%.3f Y=%.3f | dx=%.4f dy=%.4f dt=%.3f",
            x_, y_, dx, dy, dt);

        // ===== Odometry publish =====
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now_t;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        odom_pub_->publish(odom);

        // ===== 次フレーム =====
        prev_gray_ = gray.clone();
        prev_pts_ = curr_pts;

        // ===== 特徴点減少対策 =====
        if (prev_pts_.size() < 50) {
            cv::goodFeaturesToTrack(
                prev_gray_,
                prev_pts_,
                300, 0.01, 10.0);

            RCLCPP_WARN(
                get_logger(),
                "Re-detect features: %zu",
                prev_pts_.size());
        }
    }

    // ===== ROS =====
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ===== OpenCV =====
    cv::VideoCapture cap_;
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_pts_;

    // ===== 状態 =====
    bool initialized_ = false;
    double x_ = 0.0;
    double y_ = 0.0;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpticalFlowOdom>());
    rclcpp::shutdown();
    return 0;
}
