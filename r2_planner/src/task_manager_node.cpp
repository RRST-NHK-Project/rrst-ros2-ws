#include "r2_planner/task_manager_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>

namespace {
    constexpr std::array<int32_t, 19> kMffHeightByCell = {
        0,   // 0 (unused)
        200, // 1
        0,   // 2
        200, // 3
        0,   // 4
        200, // 5
        400, // 6
        200, // 7
        400, // 8
        200, // 9
        0,   // 10
        200, // 11
        0,   // 12
        0,   // 13 (1E - entrance, flat)
        0,   // 14 (2E - entrance, flat)
        0,   // 15 (3E - entrance, flat)
        0,   // 16 (1X - exit, flat)
        0,   // 17 (2X - exit, flat)
        0    // 18 (3X - exit, flat)
    };

    constexpr std::array<std::array<int32_t, 3>, 6> kMffLayoutRed = {{
        {{13, 14, 15}}, // 1E, 2E, 3E (entrance side)
        {{1, 2, 3}},    // 1, 2, 3
        {{4, 5, 6}},    // 4, 5, 6
        {{7, 8, 9}},    // 7, 8, 9
        {{10, 11, 12}}, // 10, 11, 12
        {{16, 17, 18}}, // 1X, 2X, 3X (exit side)
    }};

    constexpr std::array<std::array<int32_t, 3>, 6> kMffLayoutBlue = {{
        {{15, 14, 13}}, // 3E, 2E, 1E (entrance side)
        {{3, 2, 1}},    // 3, 2, 1
        {{6, 5, 4}},    // 6, 5, 4
        {{9, 8, 7}},    // 9, 8, 7
        {{12, 11, 10}}, // 12, 11, 10
        {{18, 17, 16}}, // 3X, 2X, 1X (exit side)
    }};
    std::string trimCopy(const std::string &text) {
        const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        });
        const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char ch) {
                             return std::isspace(ch) != 0;
                         }).base();

        if (begin >= end) {
            return "";
        }
        return std::string(begin, end);
    }

    std::vector<std::string> splitCommaSeparatedNames(const std::string &names_text) {
        std::vector<std::string> names;
        std::stringstream ss(names_text);
        std::string token;
        while (std::getline(ss, token, ',')) {
            const auto trimmed = trimCopy(token);
            if (!trimmed.empty()) {
                names.push_back(trimmed);
            }
        }
        return names;
    }
} // namespace

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
        declare_parameter<std::string>("mff_path_topic", "r2/task_mff_path");
        declare_parameter<std::string>("mff_path_advance_topic", "r2/task_mff_path_advance");
        declare_parameter<std::string>("mff_turn_cmd_topic", "r2_mff_turn_cmd");
        declare_parameter<std::string>("mff_step_cmd_topic", "r2_mff_step_cmd");
        declare_parameter<std::string>("mff_status_topic", "r2/task_mff_status");
        declare_parameter<int>("initial_mff_heading_deg", 0);
        declare_parameter<int>("fallback_drive_mode_on_unset", 0);
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
        const auto mff_path_topic = get_parameter("mff_path_topic").as_string();
        const auto mff_path_advance_topic = get_parameter("mff_path_advance_topic").as_string();
        const auto mff_turn_cmd_topic = get_parameter("mff_turn_cmd_topic").as_string();
        const auto mff_step_cmd_topic = get_parameter("mff_step_cmd_topic").as_string();
        const auto mff_status_topic = get_parameter("mff_status_topic").as_string();
        mff_heading_deg_ = normalizeHeadingDeg(static_cast<int32_t>(get_parameter("initial_mff_heading_deg").as_int()));
        fallback_drive_mode_on_unset_ = static_cast<int32_t>(get_parameter("fallback_drive_mode_on_unset").as_int());
        const auto odom_reset_topic = get_parameter("odom_reset_topic").as_string();
        auto_send_enabled_ = get_parameter("initial_auto_send_enabled").as_bool();

        state_sequence_ = {kStateWaiting, kStateRackMove, kStateStaffHandTrigger, kStateStaffAssembly, kStateMffEnter, kStateMffLeave};
        applyStateNameSequenceMapping();

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

        state_sequence_names_sub_ = create_subscription<std_msgs::msg::String>(
            "r2/task_state_sequence_names", rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateSequenceNames, this, std::placeholders::_1));

        state_pose_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            state_pose_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStatePose, this, std::placeholders::_1));

        state_mode_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            state_mode_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateMode, this, std::placeholders::_1));

        state_odom_reset_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            state_odom_reset_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onStateOdomReset, this, std::placeholders::_1));

        mff_path_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            mff_path_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onMffPath, this, std::placeholders::_1));

        mff_path_advance_sub_ = create_subscription<std_msgs::msg::Bool>(
            mff_path_advance_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onMffPathAdvance, this, std::placeholders::_1));

        auto_send_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
            auto_send_enabled_topic, rclcpp::QoS(10),
            std::bind(&TaskManagerNode::onAutoSendEnabled, this, std::placeholders::_1));

        status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            status_topic, rclcpp::QoS(1).reliable().transient_local());

        status_text_pub_ = create_publisher<std_msgs::msg::String>(
            status_text_topic, rclcpp::QoS(1).reliable().transient_local());

        auto_drive_target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
            auto_drive_target_topic, rclcpp::QoS(10));

        drive_mode_cmd_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            drive_mode_cmd_topic, rclcpp::QoS(10));

        mff_turn_cmd_pub_ = create_publisher<std_msgs::msg::Int32>(
            mff_turn_cmd_topic, rclcpp::QoS(10));

        mff_step_cmd_pub_ = create_publisher<std_msgs::msg::Int32>(
            mff_step_cmd_topic, rclcpp::QoS(10));

        mff_status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            mff_status_topic, rclcpp::QoS(1).reliable().transient_local());

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
            stateDisplayName(status_.state_code).c_str(),
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
        applyStateNameSequenceMapping();
        std::ostringstream oss;
        for (size_t i = 0; i < state_sequence_.size(); ++i) {
            if (i > 0) {
                oss << " -> ";
            }
            oss << stateDisplayName(state_sequence_[i]);
        }
        RCLCPP_INFO(get_logger(), "State sequence updated: %s", oss.str().c_str());
    }

    void TaskManagerNode::onStateSequenceNames(const std_msgs::msg::String::SharedPtr msg) {
        pending_state_sequence_names_ = parseStateNameSequence(msg->data);
        applyStateNameSequenceMapping();
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
            RCLCPP_INFO(get_logger(), "Cleared state pose target: state=%s(%ld)", stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
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
            stateDisplayName(state_code).c_str(),
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
        const bool rotate_only = data.size() >= 3 && data[2] != 0;

        if (!enabled) {
            state_mode_targets_.erase(state_code);
            state_rotate_only_targets_.erase(state_code);
            RCLCPP_INFO(get_logger(), "Cleared state mode target: state=%s(%ld)", stateDisplayName(state_code).c_str(), static_cast<long>(state_code));

            if (auto_send_enabled_ && state_code == status_.state_code) {
                publishAutoDriveModeForState(state_code);
                RCLCPP_INFO(get_logger(), "Applied fallback mode immediately for current state=%s(%ld)",
                            stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
            }
            return;
        }

        if (data.size() < 2) {
            RCLCPP_WARN(get_logger(), "Ignoring state mode update: state=%ld missing mode code", static_cast<long>(state_code));
            return;
        }

        const int32_t mode_code = data[1];
        if (mode_code < 0 || mode_code > 4) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid mode code %ld for state=%s(%ld) (valid: 0-4)",
                        static_cast<long>(mode_code), stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        state_mode_targets_[state_code] = mode_code;
        state_rotate_only_targets_[state_code] = rotate_only;
        RCLCPP_INFO(get_logger(), "Updated state mode target: state=%s(%ld) mode=%ld rotate_only=%s",
                    stateDisplayName(state_code).c_str(), static_cast<long>(state_code), static_cast<long>(mode_code),
                    rotate_only ? "true" : "false");

        if (auto_send_enabled_ && state_code == status_.state_code) {
            publishAutoDriveModeForState(state_code);
            RCLCPP_INFO(get_logger(), "Applied state mode immediately for current state=%s(%ld)",
                        stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
        }
    }

    void TaskManagerNode::onAutoSendEnabled(const std_msgs::msg::Bool::SharedPtr msg) {
        auto_send_enabled_ = msg->data;
        RCLCPP_INFO(get_logger(), "Auto send on state transition: %s", auto_send_enabled_ ? "ENABLED" : "DISABLED");
    }

    void TaskManagerNode::onMffPath(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::vector<int32_t> next_path;
        next_path.reserve(msg->data.size());

        for (const auto cell : msg->data) {
            if (cell < kMffCellMin || cell > kMffCellMax) {
                RCLCPP_WARN(get_logger(), "Ignoring invalid MFF path cell: %ld", static_cast<long>(cell));
                continue;
            }
            next_path.push_back(cell);
        }

        if (next_path.empty()) {
            RCLCPP_WARN(get_logger(), "Ignoring empty/invalid MFF path update");
            return;
        }

        mff_path_ = next_path;
        mff_path_index_ = 0;

        auto it = std::find(mff_path_.begin(), mff_path_.end(), status_.mff_cell);
        if (it != mff_path_.end()) {
            mff_path_index_ = static_cast<size_t>(std::distance(mff_path_.begin(), it));
        }

        std::ostringstream oss;
        for (size_t i = 0; i < mff_path_.size(); ++i) {
            if (i > 0) {
                oss << " -> ";
            }
            oss << mff_path_[i];
        }
        RCLCPP_INFO(get_logger(), "Updated MFF path (%zu): %s", mff_path_.size(), oss.str().c_str());
        publishMffRuntimeStatus(true);
    }

    void TaskManagerNode::onMffPathAdvance(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg->data) {
            return;
        }

        if (mff_path_.size() < 2) {
            RCLCPP_WARN(get_logger(), "Ignoring MFF path advance: path has fewer than 2 cells");
            return;
        }

        size_t current_index = mff_path_index_;
        auto it = std::find(mff_path_.begin(), mff_path_.end(), status_.mff_cell);
        if (it != mff_path_.end()) {
            current_index = static_cast<size_t>(std::distance(mff_path_.begin(), it));
        }

        if (current_index + 1 >= mff_path_.size()) {
            RCLCPP_INFO(get_logger(), "MFF path advance ignored: already at end of path");
            return;
        }

        const int32_t from_cell = mff_path_[current_index];
        const int32_t to_cell = mff_path_[current_index + 1];

        publishMffTransitionCommands(from_cell, to_cell);
        setCell(to_cell);
        mff_path_index_ = current_index + 1;
        publishStatus(true);
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
            RCLCPP_INFO(get_logger(), "Cleared state odom reset target: state=%s(%ld)", stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
            return;
        }

        state_odom_reset_targets_[state_code] = true;
        RCLCPP_INFO(get_logger(), "Updated state odom reset target: state=%s(%ld) enabled=true",
                    stateDisplayName(state_code).c_str(), static_cast<long>(state_code));
    }

    void TaskManagerNode::setState(int32_t state_code) {
        status_.state_code = state_code;
        publishStateSideEffects(state_code);
    }

    void TaskManagerNode::publishStateSideEffects(int32_t state_code) {
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

    std::string TaskManagerNode::stateDisplayName(int32_t state_code) const {
        auto it = state_name_overrides_.find(state_code);
        if (it != state_name_overrides_.end() && !it->second.empty()) {
            return it->second;
        }
        return stateName(state_code);
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

    std::string TaskManagerNode::buildStatusText(const TaskStatus &status) const {
        std::ostringstream oss;
        oss << "state=" << stateDisplayName(status.state_code)
            << " color=" << colorName(status.color_code)
            << " cell=" << status.mff_cell
            << " mode=" << transitionModeName(status.transition_mode_code);
        return oss.str();
    }

    void TaskManagerNode::applyStateNameSequenceMapping() {
        state_name_overrides_.clear();

        const auto sequenceSize = state_sequence_.size();
        for (size_t i = 0; i < sequenceSize; ++i) {
            const int32_t state_code = state_sequence_[i];
            if (i < pending_state_sequence_names_.size()) {
                const auto name = pending_state_sequence_names_[i];
                if (!name.empty()) {
                    state_name_overrides_[state_code] = name;
                }
            }
        }
    }

    std::vector<std::string> TaskManagerNode::parseStateNameSequence(const std::string &names_text) const {
        return splitCommaSeparatedNames(names_text);
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
            stateDisplayName(state_code).c_str(),
            static_cast<long>(state_code),
            static_cast<double>(it->second.x),
            static_cast<double>(it->second.y),
            static_cast<double>(it->second.yaw_rad));
    }

    void TaskManagerNode::publishAutoDriveModeForState(int32_t state_code) {
        auto it = state_mode_targets_.find(state_code);
        auto rotate_it = state_rotate_only_targets_.find(state_code);

        std_msgs::msg::Int32MultiArray mode_msg;
        if (it == state_mode_targets_.end()) {
            if (fallback_drive_mode_on_unset_ < 0 || fallback_drive_mode_on_unset_ > 4) {
                return;
            }
            mode_msg.data = {fallback_drive_mode_on_unset_, 0};
        } else {
            const bool rotate_only = (rotate_it != state_rotate_only_targets_.end() && rotate_it->second);
            mode_msg.data = {it->second, rotate_only ? 1 : 0};
        }
        drive_mode_cmd_pub_->publish(mode_msg);

        RCLCPP_INFO(
            get_logger(),
            "Published auto drive mode for state=%s(%ld): mode=%ld rotate_only=%s",
            stateDisplayName(state_code).c_str(),
            static_cast<long>(state_code),
            static_cast<long>(mode_msg.data[0]),
            mode_msg.data.size() > 1 && mode_msg.data[1] ? "true" : "false");
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
            stateDisplayName(state_code).c_str(),
            static_cast<long>(state_code));
    }

    void TaskManagerNode::publishMffTransitionCommands(int32_t from_cell, int32_t to_cell) {
        int32_t target_heading_deg = mff_heading_deg_;
        int32_t turn_deg = 0;
        int32_t step_cmd = 0;
        if (!computeMffTransition(from_cell, to_cell, mff_heading_deg_, target_heading_deg, turn_deg, step_cmd)) {
            return;
        }
        mff_heading_deg_ = normalizeHeadingDeg(target_heading_deg);

        std_msgs::msg::Int32 turn_msg;
        turn_msg.data = turn_deg;
        mff_turn_cmd_pub_->publish(turn_msg);

        std_msgs::msg::Int32 step_msg;
        step_msg.data = step_cmd;
        mff_step_cmd_pub_->publish(step_msg);

        RCLCPP_INFO(get_logger(),
                    "Published MFF transition commands: %ld -> %ld, turn=%ld deg, step=%ld",
                    static_cast<long>(from_cell), static_cast<long>(to_cell),
                    static_cast<long>(turn_deg), static_cast<long>(step_cmd));
    }

    bool TaskManagerNode::computeMffTransition(
        int32_t from_cell,
        int32_t to_cell,
        int32_t current_heading_deg,
        int32_t &target_heading_deg,
        int32_t &turn_deg,
        int32_t &step_cmd) const {
        if (from_cell < kMffCellMin || from_cell > kMffCellMax || to_cell < kMffCellMin || to_cell > kMffCellMax) {
            RCLCPP_WARN(get_logger(), "Skipping MFF transition command: invalid cell transition %ld -> %ld",
                        static_cast<long>(from_cell), static_cast<long>(to_cell));
            return false;
        }

        const std::array<std::array<int32_t, 3>, 6> layout = status_.color_code == kColorBlue ? kMffLayoutBlue : kMffLayoutRed;

        int32_t from_r = -1;
        int32_t from_c = -1;
        int32_t to_r = -1;
        int32_t to_c = -1;
        for (int32_t r = 0; r < 6; ++r) {
            for (int32_t c = 0; c < 3; ++c) {
                if (layout[r][c] == from_cell) {
                    from_r = r;
                    from_c = c;
                }
                if (layout[r][c] == to_cell) {
                    to_r = r;
                    to_c = c;
                }
            }
        }

        if (from_r < 0 || to_r < 0) {
            RCLCPP_WARN(get_logger(), "Skipping MFF transition command: cell not found in current layout");
            return false;
        }

        const int32_t dr = to_r - from_r;
        const int32_t dc = to_c - from_c;
        if (std::abs(dr) + std::abs(dc) != 1) {
            RCLCPP_WARN(get_logger(), "Skipping MFF transition command: non-adjacent transition %ld -> %ld",
                        static_cast<long>(from_cell), static_cast<long>(to_cell));
            return false;
        }

        target_heading_deg = current_heading_deg;
        if (dc == 1) {
            target_heading_deg = 0;
        } else if (dr == 1) {
            target_heading_deg = 90;
        } else if (dc == -1) {
            target_heading_deg = 180;
        } else if (dr == -1) {
            target_heading_deg = 270;
        }

        turn_deg = normalizeTurnDeg(target_heading_deg - current_heading_deg);

        const int32_t from_h = kMffHeightByCell[static_cast<size_t>(from_cell)];
        const int32_t to_h = kMffHeightByCell[static_cast<size_t>(to_cell)];
        step_cmd = 0;
        if (to_h > from_h) {
            step_cmd = 1;
        } else if (to_h < from_h) {
            step_cmd = -1;
        }

        return true;
    }

    void TaskManagerNode::publishMffRuntimeStatus(bool force) {
        if (!mff_status_pub_) {
            return;
        }
        if (!force && has_published_status_ && !hasChanged(status_, last_published_status_)) {
            return;
        }

        int32_t next_cell = 0;
        int32_t next_turn_deg = 0;
        int32_t next_step_cmd = 0;
        int32_t predicted_heading_deg = mff_heading_deg_;

        if (!mff_path_.empty()) {
            size_t current_index = mff_path_index_;
            const auto it = std::find(mff_path_.begin(), mff_path_.end(), status_.mff_cell);
            if (it != mff_path_.end()) {
                current_index = static_cast<size_t>(std::distance(mff_path_.begin(), it));
            }

            if (current_index + 1 < mff_path_.size()) {
                next_cell = mff_path_[current_index + 1];
                int32_t target_heading_deg = mff_heading_deg_;
                if (!computeMffTransition(status_.mff_cell, next_cell, mff_heading_deg_, target_heading_deg, next_turn_deg, next_step_cmd)) {
                    next_cell = 0;
                    next_turn_deg = 0;
                    next_step_cmd = 0;
                } else {
                    predicted_heading_deg = normalizeHeadingDeg(target_heading_deg);
                }
            }
        }

        std_msgs::msg::Int32MultiArray mff_status_msg;
        // [current_cell, next_cell, next_turn_deg, next_step_cmd, current_heading_deg, predicted_heading_deg]
        mff_status_msg.data = {
            status_.mff_cell,
            next_cell,
            next_turn_deg,
            next_step_cmd,
            mff_heading_deg_,
            predicted_heading_deg,
        };
        mff_status_pub_->publish(mff_status_msg);
    }

    int32_t TaskManagerNode::normalizeHeadingDeg(int32_t heading_deg) {
        int32_t normalized = heading_deg % 360;
        if (normalized < 0) {
            normalized += 360;
        }
        const int32_t snapped = static_cast<int32_t>(std::lround(static_cast<double>(normalized) / 90.0)) * 90;
        return (snapped + 360) % 360;
    }

    int32_t TaskManagerNode::normalizeTurnDeg(int32_t turn_deg) {
        int32_t normalized = turn_deg % 360;
        if (normalized <= -180) {
            normalized += 360;
        } else if (normalized > 180) {
            normalized -= 360;
        }
        return static_cast<int32_t>(std::lround(static_cast<double>(normalized) / 90.0)) * 90;
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

        publishMffRuntimeStatus(force);

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