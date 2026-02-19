#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>

class MecanumAutoForward : public rclcpp::Node {
public:
    MecanumAutoForward() : Node("mecanum_auto_forward") {
        // ホイール名（URDFのjoint名に合わせる）
        wheel_names_ = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};

        // ホイールごとの publisher
        for (auto &name : wheel_names_) {
            wheel_pubs_.push_back(this->create_publisher<std_msgs::msg::Float64>(
                "/" + name + "/command", 10));
        }

        // タイマーで定期的に前進コマンドを送信
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MecanumAutoForward::publishWheelSpeed, this));

        RCLCPP_INFO(this->get_logger(), "Mecanum auto-forward node started");
    }

private:
    void publishWheelSpeed() {
        std_msgs::msg::Float64 wheel_msg;
        wheel_msg.data = forward_speed_ / wheel_radius_; // rad/s

        for (auto &pub : wheel_pubs_) {
            pub->publish(wheel_msg);
        }
    }

    std::vector<std::string> wheel_names_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_pubs_;
    rclcpp::TimerBase::SharedPtr timer_;
    const double wheel_radius_ = 0.05; // URDFに合わせる
    const double forward_speed_ = 0.1; // m/s
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MecanumAutoForward>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
