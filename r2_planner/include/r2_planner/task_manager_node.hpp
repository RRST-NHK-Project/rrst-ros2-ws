#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace r2_planner {

    class TaskManagerNode : public rclcpp::Node {
    public:
        TaskManagerNode();

    private:
        struct TaskStatus {
            int32_t state_code{0};
            int32_t color_code{-1};
            int32_t mff_cell{0};
            int32_t transition_mode_code{0};
        };

        struct StatePoseTarget {
            bool enabled{false};
            float x{0.0F};
            float y{0.0F};
            float yaw_rad{0.0F};
        };

        static constexpr int32_t kColorUnknown = -1;
        static constexpr int32_t kColorBlue = 0;
        static constexpr int32_t kColorRed = 1;

        static constexpr int32_t kMffCellMin = 1;
        static constexpr int32_t kMffCellMax = 12;

        static constexpr int32_t kStateWaiting = 0;
        static constexpr int32_t kStateMffEnter = 1;
        static constexpr int32_t kStateMffLeave = 2;
        static constexpr int32_t kStateStaffAssembly = 3;

        static constexpr int32_t kTransitionManual = 0;
        static constexpr int32_t kTransitionAuto = 1;

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr command_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr color_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr cell_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr transition_mode_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr state_sequence_sub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_pose_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr state_mode_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_send_enabled_sub_;

        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr status_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_text_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr auto_drive_target_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_mode_cmd_pub_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr auto_transition_timer_;

        TaskStatus status_;
        TaskStatus last_published_status_;
        bool has_published_status_{false};
        std::vector<int32_t> state_sequence_;
        std::unordered_map<int32_t, StatePoseTarget> state_pose_targets_;
        std::unordered_map<int32_t, int32_t> state_mode_targets_;
        bool auto_send_enabled_{true};

        void onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        void onState(const std_msgs::msg::Int32::SharedPtr msg);
        void onColor(const std_msgs::msg::Int32::SharedPtr msg);
        void onCell(const std_msgs::msg::Int32::SharedPtr msg);
        void onTransitionMode(const std_msgs::msg::Int32::SharedPtr msg);
        void onStateSequence(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        void onStatePose(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void onStateMode(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        void onAutoSendEnabled(const std_msgs::msg::Bool::SharedPtr msg);

        void setState(int32_t state_code);
        void setColor(int32_t color_code);
        void setCell(int32_t cell);
        void setTransitionMode(int32_t transition_mode_code);
        void advanceAutoTransition();
        int32_t nextStateCode(int32_t current_state_code) const;
        static std::vector<int32_t> normalizedStateSequence(const std::vector<int32_t> &sequence);
        void publishAutoDriveTargetForState(int32_t state_code);
        void publishAutoDriveModeForState(int32_t state_code);

        static std::string stateName(int32_t state_code);
        static std::string colorName(int32_t color_code);
        static std::string transitionModeName(int32_t transition_mode_code);
        static std::string buildStatusText(const TaskStatus &status);
        static bool hasChanged(const TaskStatus &lhs, const TaskStatus &rhs);

        void publishStatus(bool force = false);
    };

} // namespace r2_planner