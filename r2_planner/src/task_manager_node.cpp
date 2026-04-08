#include "r2_planner/task_manager_node.hpp"

#include <algorithm>
#include <chrono>
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
        declare_parameter<std::string>("status_topic", "r2/task_status");
        declare_parameter<std::string>("status_text_topic", "r2/task_status_text");

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
        const auto status_topic = get_parameter("status_topic").as_string();
        const auto status_text_topic = get_parameter("status_text_topic").as_string();

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

        status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            status_topic, rclcpp::QoS(1).reliable().transient_local());

        status_text_pub_ = create_publisher<std_msgs::msg::String>(
            status_text_topic, rclcpp::QoS(1).reliable().transient_local());

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

    void TaskManagerNode::setState(int32_t state_code) {
        status_.state_code = state_code;
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

    int32_t TaskManagerNode::nextStateCode(int32_t current_state_code) {
        switch (current_state_code) {
        case kStateWaiting:
            return kStateMffEnter;
        case kStateMffEnter:
            return kStateMffLeave;
        case kStateMffLeave:
        default:
            return kStateWaiting;
        }
    }

    void TaskManagerNode::advanceAutoTransition() {
        if (status_.transition_mode_code != kTransitionAuto) {
            return;
        }

        const int32_t next_state = nextStateCode(status_.state_code);
        if (next_state == status_.state_code) {
            return;
        }

        status_.state_code = next_state;
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