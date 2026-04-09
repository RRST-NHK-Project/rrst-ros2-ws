#include "r2_planner/task_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace r2_planner {

    TaskManagerNode::TaskManagerNode()
        : Node("r2_task_manager") {
        declare_parameter<int>("initial_state_code", kStateWaiting);
        declare_parameter<int>("initial_color_code", kColorUnknown);
        declare_parameter<int>("initial_mff_cell", 0);
        declare_parameter<int>("initial_transition_mode", kTransitionManual);
        declare_parameter<int>("auto_transition_period_ms", 3000);
        declare_parameter<std::string>("command_topic", "r2/task_command");
        declare_parameter<std::string>("state_topic", "r2/task_state");
        declare_parameter<std::string>("color_topic", "r2/task_color");
        declare_parameter<std::string>("cell_topic", "r2/task_cell");
        declare_parameter<std::string>("transition_mode_topic", "r2/task_transition_mode");
        declare_parameter<std::string>("state_sequence_topic", "r2/task_state_sequence");
        declare_parameter<std::string>("state_pose_topic", "r2/task_state_pose");
        declare_parameter<std::string>("state_mode_topic", "r2/task_state_mode");
        declare_parameter<std::string>("state_odom_reset_topic", "r2/task_state_odom_reset");
        declare_parameter<std::string>("auto_send_enabled_topic", "r2/task_auto_send_enabled");
        declare_parameter<bool>("initial_auto_send_enabled", true);
        declare_parameter<std::string>("status_topic", "r2/task_status");
        declare_parameter<std::string>("status_text_topic", "r2/task_status_text");
        declare_parameter<std::string>("auto_drive_target_topic", "r2_autodrive_cmd");
        declare_parameter<std::string>("drive_mode_cmd_topic", "r2_drive_mode_cmd");
        declare_parameter<std::string>("odom_reset_topic", "odom_reset");

        status_.state_code = static_cast<int32_t>(get_parameter("initial_state_code").as_int());
        status_.color_code = static_cast<int32_t>(get_parameter("initial_color_code").as_int());
        status_.mff_cell = static_cast<int32_t>(get_parameter("initial_mff_cell").as_int());
        status_.transition_mode_code = static_cast<int32_t>(get_parameter("initial_transition_mode").as_int());

        const auto auto_transition_period_ms = get_parameter("auto_transition_period_ms").as_int();

        const auto command_topic = get_parameter("command_topic").as_string();
        const auto state_topic = get_parameter("state_topic").as_string();
        const auto color_topic = get_parameter("color_topic").as_string();
        const auto cell_topic = get_parameter("cell_topic").as_string();
        const auto transition_mode_topic = get_parameter("transition_mode_topic").as_string();
        const auto state_sequence_topic = get_parameter("state_sequence_topic").as_string();
        const auto state_pose_topic = get_parameter("state_pose_topic").as_string();
        const auto state_mode_topic = get_parameter("state_mode_topic").as_string();
        const auto state_odom_reset_topic = get_parameter("state_odom_reset_topic").as_string();
        const auto auto_send_enabled_topic = get_parameter("auto_send_enabled_topic").as_string();
        const auto status_topic = get_parameter("status_topic").as_string();
        const auto status_text_topic = get_parameter("status_text_topic").as_string();
        const auto auto_drive_target_topic = get_parameter("auto_drive_target_topic").as_string();
        const auto drive_mode_cmd_topic = get_parameter("drive_mode_cmd_topic").as_string();
        const auto odom_reset_topic = get_parameter("odom_reset_topic").as_string();
        auto_send_enabled_ = get_parameter("initial_auto_send_enabled").as_bool();

        state_sequence_ = {kStateWaiting, kStateRackMove, kStateStaffHandTrigger, kStateStaffAssembly, kStateMffEnter, kStateMffLeave};

        command_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            command_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onCommand, this, std::placeholders::_1));

        state_sub_ = create_subscription<std_msgs::msg::Int32>(
            state_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onState, this, std::placeholders::_1));

        color_sub_ = create_subscription<std_msgs::msg::Int32>(
            color_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onColor, this, std::placeholders::_1));

        cell_sub_ = create_subscription<std_msgs::msg::Int32>(
            cell_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onCell, this, std::placeholders::_1));

        transition_mode_sub_ = create_subscription<std_msgs::msg::Int32>(
            transition_mode_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onTransitionMode, this, std::placeholders::_1));

        state_sequence_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            state_sequence_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateSequence, this, std::placeholders::_1));

        state_pose_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            state_pose_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStatePose, this, std::placeholders::_1));

        state_mode_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            state_mode_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateMode, this, std::placeholders::_1));

        state_odom_reset_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            state_odom_reset_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateOdomReset, this, std::placeholders::_1));

        auto_send_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
            auto_send_enabled_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onAutoSendEnabled, this, std::placeholders::_1));

        status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            status_topic, rclcpp::QoS(1).reliable().transient_local());

        status_text_pub_ = create_publisher<std_msgs::msg::String>(
            status_text_topic, rclcpp::QoS(1).reliable().transient_local());

        auto_drive_target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            auto_drive_target_topic, rclcpp::QoS(10));

        drive_mode_cmd_pub_ = create_publisher<std_msgs::msg::Int32>(
            drive_mode_cmd_topic, rclcpp::QoS(10));

        odom_reset_pub_ = create_publisher<std_msgs::msg::Bool>(
            odom_reset_topic, rclcpp::QoS(10));

        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(200),
            [this]() { publishStatus(false); });

        auto_transition_timer_ = create_wall_timer(
            std::chrono::milliseconds(std::max<long>(200, auto_transition_period_ms)),
            [this]() { advanceAutoTransition(); });

        publishStatus(true);

        RCLCPP_INFO(
            get_logger(),
            "Task manager ready: state=%s(%ld) color=%s(%ld) cell=%ld mode=%s(%ld)",
            stateName(status_.state_code).c_str(),
            static_cast<long>(status_.state_code),
            colorName(status_.color_code).c_str(),
            static_cast<long>(status_.color_code),
            static_cast<long>(status_.mff_cell),
            transitionModeName(status_.transition_mode_code).c_str(),
            static_cast<long>(status_.transition_mode_code));
    }

    void TaskManagerNode::onCommand(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }

        if (status_.transition_mode_code == kTransitionAuto) {
            RCLCPP_WARN(get_logger(), "Ignoring direct task command while auto transition mode is active");
            return;
        }

        setState(msg->data[0]);

        if (msg->data.size() >= 2) {
            setColor(msg->data[1]);
        }

        if (msg->data.size() >= 3) {
            setCell(msg->data[2]);
        }

        publishStatus(true);
    }

    void TaskManagerNode::onState(const std_msgs::msg::Int32::SharedPtr msg) {
        if (status_.transition_mode_code == kTransitionAuto) {
            RCLCPP_WARN(get_logger(), "Ignoring manual state update while auto transition mode is active");
            return;
        }
        setState(msg->data);
        publishStatus(true);
    }

    void TaskManagerNode::onColor(const std_msgs::msg::Int32::SharedPtr msg) {
        setColor(msg->data);
        publishStatus(true);
    }

    void TaskManagerNode::onCell(const std_msgs::msg::Int32::SharedPtr msg) {
        setCell(msg->data);
        publishStatus(true);
    }

    void TaskManagerNode::onTransitionMode(const std_msgs::msg::Int32::SharedPtr msg) {
        setTransitionMode(msg->data);
        publishStatus(true);
    }

    void TaskManagerNode::onStateSequence(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        const auto next_sequence = normalizedStateSequence(msg->data);
        if (next_sequence.empty()) {
            RCLCPP_WARN(get_logger(), "Ignoring empty state sequence update");
            return;
        }

        state_sequence_ = next_sequence;
        std::ostringstream oss;
        for (size_t i = 0; i < state_sequence_.size(); ++i) {
            if (i > 0) {
                oss << " -> ";
            }
            oss << stateName(state_sequence_[i]);
        }
        RCLCPP_INFO(get_logger(), "State sequence updated: %s", oss.str().c_str());
    }

    void TaskManagerNode::onStatePose(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        const auto &data = msg->data;
        if (data.size() < 2) {
            RCLCPP_WARN(get_logger(), "Ignoring state pose update with insufficient length");
            return;
        }

        const int32_t state_code = static_cast<int32_t>(std::lround(data[0]));
        const bool enabled = data.size() >= 5 ? data[4] >= 0.5F : true;

        if (!enabled) {
            state_pose_targets_.erase(state_code);
            RCLCPP_INFO(get_logger(), "Cleared state pose target: state=%s(%ld)", stateName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        if (data.size() < 4) {
            RCLCPP_WARN(get_logger(), "Ignoring state pose update: state=%ld missing x/y/yaw", static_cast<long>(state_code));
            return;
        }

        StatePoseTarget target;
        target.enabled = true;
        target.x = data[1];
        target.y = data[2];
        target.yaw_rad = data[3];

        state_pose_targets_[state_code] = target;
        RCLCPP_INFO(
            get_logger(),
            "Updated state pose target: state=%s(%ld) x=%.3f y=%.3f yaw=%.3f",
            stateName(state_code).c_str(),
            static_cast<long>(state_code),
            static_cast<double>(target.x),
            static_cast<double>(target.y),
            static_cast<double>(target.yaw_rad));
    }

    void TaskManagerNode::onStateMode(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        const auto &data = msg->data;
        if (data.size() < 1) {
            RCLCPP_WARN(get_logger(), "Ignoring state mode update with no state code");
            return;
        }

        const int32_t state_code = data[0];
        const bool enabled = data.size() >= 2 && data[1] >= 0;

        if (!enabled) {
            state_mode_targets_.erase(state_code);
            RCLCPP_INFO(get_logger(), "Cleared state mode target: state=%s(%ld)", stateName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        if (data.size() < 2) {
            RCLCPP_WARN(get_logger(), "Ignoring state mode update: state=%ld missing mode code", static_cast<long>(state_code));
            return;
        }

        const int32_t mode_code = data[1];
        if (mode_code < 0 || mode_code > 2) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid mode code %ld for state=%s(%ld) (valid: 0-2)",
                        static_cast<long>(mode_code), stateName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        state_mode_targets_[state_code] = mode_code;
        RCLCPP_INFO(get_logger(), "Updated state mode target: state=%s(%ld) mode=%ld",
                    stateName(state_code).c_str(), static_cast<long>(state_code), static_cast<long>(mode_code));
    }

    void TaskManagerNode::onAutoSendEnabled(const std_msgs::msg::Bool::SharedPtr msg) {
        auto_send_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "Auto send on state transition: %s", auto_send_enabled_ ? "ENABLED" : "DISABLED");
    }

    void TaskManagerNode::onStateOdomReset(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        const auto &data = msg->data;
        if (data.size() < 1) {
            RCLCPP_WARN(get_logger(), "Ignoring state odom reset update with no state code");
            return;
        }

        const int32_t state_code = data[0];
        const bool enabled = data.size() >= 2 && data[1] > 0;

        if (!enabled) {
            state_odom_reset_targets_.erase(state_code);
            RCLCPP_INFO(get_logger(), "Cleared state odom reset target: state=%s(%ld)", stateName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        state_odom_reset_targets_[state_code] = true;
        RCLCPP_INFO(get_logger(), "Updated state odom reset target: state=%s(%ld) enabled=true",
                    stateName(state_code).c_str(), static_cast<long>(state_code));
    }

    void TaskManagerNode::setState(int32_t state_code) {
        if (status_.state_code == state_code) {
            return;
        }
        status_.state_code = state_code;
        if (!auto_send_enabled_) {
            return;
        }
        publishAutoDriveTargetForState(state_code);
        publishAutoDriveModeForState(state_code);
        publishOdomResetForState(state_code);
    }

    void TaskManagerNode::setColor(int32_t color_code) {
        if (color_code != kColorUnknown && color_code != kColorBlue && color_code != kColorRed) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid color code: %ld", static_cast<long>(color_code));
            return;
        }
        status_.color_code = color_code;
    }

    void TaskManagerNode::setCell(int32_t cell) {
        if (cell != 0 && (cell < kMffCellMin || cell > kMffCellMax)) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid MFF cell: %ld", static_cast<long>(cell));
            return;
        }
        status_.mff_cell = cell;
    }

    void TaskManagerNode::setTransitionMode(int32_t transition_mode_code) {
        if (transition_mode_code != kTransitionManual && transition_mode_code != kTransitionAuto) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid transition mode: %ld", static_cast<long>(transition_mode_code));
            return;
        }
        status_.transition_mode_code = transition_mode_code;
    }

    int32_t TaskManagerNode::nextStateCode(int32_t current_state_code) const {
        if (state_sequence_.empty()) {
            return current_state_code;
        }

        auto it = std::find(state_sequence_.begin(), state_sequence_.end(), current_state_code);
        if (it == state_sequence_.end()) {
            return state_sequence_.front();
        }

        ++it;
        if (it == state_sequence_.end()) {
            return state_sequence_.front();
        }

        return *it;
    }

    void TaskManagerNode::advanceAutoTransition() {
        if (status_.transition_mode_code != kTransitionAuto) {
            return;
        }

        const int32_t next_state = nextStateCode(status_.state_code);
        if (next_state == status_.state_code) {
            return;
        }

        setState(next_state);
        publishStatus(true);
    }

    std::string TaskManagerNode::stateName(int32_t state_code) {
        switch (state_code) {
        case kStateWaiting:
            return "WAITING";
        case kStateMffEnter:
            return "MFF_ENTER";
        case kStateMffLeave:
            return "MFF_LEAVE";
        case kStateStaffAssembly:
            return "STAFF_ASSEMBLY";
        case kStateRackMove:
            return "RACK_MOVE";
        case kStateStaffHandTrigger:
            return "STAFF_HAND_TRIGGER";
        default: {
            std::ostringstream oss;
            oss << "STATE_" << state_code;
            return oss.str();
        }
        }
    }

    std::string TaskManagerNode::colorName(int32_t color_code) {
        switch (color_code) {
        case kColorBlue:
            return "BLUE";
        case kColorRed:
            return "RED";
        case kColorUnknown:
        default:
            return "UNKNOWN";
        }
    }

    std::string TaskManagerNode::transitionModeName(int32_t transition_mode_code) {
        switch (transition_mode_code) {
        case kTransitionManual:
            return "MANUAL";
        case kTransitionAuto:
            return "AUTO";
        default: {
            std::ostringstream oss;
            oss << "MODE_" << transition_mode_code;
            return oss.str();
        }
        }
    }

    std::string TaskManagerNode::buildStatusText(const TaskStatus &status) {
        std::ostringstream oss;
        oss << "state=" << stateName(status.state_code)
            << " color=" << colorName(status.color_code)
            << " cell=" << status.mff_cell
            << " mode=" << transitionModeName(status.transition_mode_code);
        return oss.str();
    }

    bool TaskManagerNode::hasChanged(const TaskStatus &lhs, const TaskStatus &rhs) {
        return lhs.state_code != rhs.state_code ||
               lhs.color_code != rhs.color_code ||
               lhs.mff_cell != rhs.mff_cell ||
               lhs.transition_mode_code != rhs.transition_mode_code;
    }

    std::vector<int32_t> TaskManagerNode::normalizedStateSequence(const std::vector<int32_t> &sequence) {
        std::vector<int32_t> normalized;
        normalized.reserve(sequence.size());

        for (const auto state_code : sequence) {
            if (std::find(normalized.begin(), normalized.end(), state_code) == normalized.end()) {
                normalized.push_back(state_code);
            }
        }

        return normalized;
    }

    void TaskManagerNode::publishAutoDriveTargetForState(int32_t state_code) {
        auto it = state_pose_targets_.find(state_code);
        if (it == state_pose_targets_.end() || !it->second.enabled) {
            return;
        }

        std_msgs::msg::Float32MultiArray target_msg;
        target_msg.data = {it->second.x, it->second.y, it->second.yaw_rad};
        auto_drive_target_pub_->publish(target_msg);

        RCLCPP_INFO(
            get_logger(),
            "Published auto drive target for state=%s(%ld): x=%.3f y=%.3f yaw=%.3f",
            stateName(state_code).c_str(),
            static_cast<long>(state_code),
            static_cast<double>(it->second.x),
            static_cast<double>(it->second.y),
            static_cast<double>(it->second.yaw_rad));
    }

    void TaskManagerNode::publishAutoDriveModeForState(int32_t state_code) {
        auto it = state_mode_targets_.find(state_code);
        if (it == state_mode_targets_.end()) {
            return;
        }

        std_msgs::msg::Int32 mode_msg;
        mode_msg.data = it->second;
        drive_mode_cmd_pub_->publish(mode_msg);

        RCLCPP_INFO(
            get_logger(),
            "Published auto drive mode for state=%s(%ld): mode=%ld",
            stateName(state_code).c_str(),
            static_cast<long>(state_code),
            static_cast<long>(it->second));
    }

    void TaskManagerNode::publishOdomResetForState(int32_t state_code) {
        auto it = state_odom_reset_targets_.find(state_code);
        if (it == state_odom_reset_targets_.end() || !it->second) {
            return;
        }

        std_msgs::msg::Bool reset_msg;
        reset_msg.data = true;
        odom_reset_pub_->publish(reset_msg);

        RCLCPP_INFO(
            get_logger(),
            "Published odom reset for state=%s(%ld)",
            stateName(state_code).c_str(),
            static_cast<long>(state_code));
    }

    void TaskManagerNode::publishStatus(bool force) {
        if (!force && has_published_status_ && !hasChanged(status_, last_published_status_)) {
            return;
        }

        std_msgs::msg::Int32MultiArray status_msg;
        status_msg.data = {status_.state_code, status_.color_code, status_.mff_cell, status_.transition_mode_code};
        status_pub_->publish(status_msg);

        std_msgs::msg::String text_msg;
        text_msg.data = buildStatusText(status_);
        status_text_pub_->publish(text_msg);

        if (has_published_status_ && hasChanged(status_, last_published_status_)) {
            RCLCPP_INFO(
                get_logger(), "Task updated: %s", text_msg.data.c_str());
        }

        last_published_status_ = status_;
        has_published_status_ = true;
    }

} // namespace r2_planner

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<r2_planner::TaskManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}