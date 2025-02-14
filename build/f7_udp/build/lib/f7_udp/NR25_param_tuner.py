import sys
from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QSlider,
    QPushButton,
    QLineEdit,
)
from PySide6.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class ParameterNode(Node):
    def __init__(self):
        super().__init__("gui_parameter_node")
        self.declare_parameter("roller_speed_dribble_ab", 50)
        self.declare_parameter("roller_speed_dribble_cd", 50)
        self.declare_parameter("roller_speed_shoot_ab", 50)  # 新しいパラメータ
        self.declare_parameter("roller_speed_shoot_cd", 50)  # 新しいパラメータ

        self.publisher = self.create_publisher(Int32MultiArray, "parameter_array", 10)
        self.timer = self.create_timer(0.1, self.publish_parameters)

        # Shoot and Dribble state
        self.shoot_state = 0
        self.dribble_state = 0

    def get_param(self, name):
        return self.get_parameter(name).value

    def set_param(self, name, value):
        self.set_parameters(
            [rclpy.parameter.Parameter(name, rclpy.Parameter.Type.INTEGER, value)]
        )

    def set_shoot_state(self, state):
        self.shoot_state = state

    def set_dribble_state(self, state):
        self.dribble_state = state

    def publish_parameters(self):
        param_array = [
            self.get_param("roller_speed_dribble_ab"),
            self.get_param("roller_speed_dribble_cd"),
            self.get_param("roller_speed_shoot_ab"),  # 変更
            self.get_param("roller_speed_shoot_cd"),  # 変更
            self.shoot_state,
            self.dribble_state,
        ]
        msg = Int32MultiArray(data=param_array)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {param_array}")


class ParameterGUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("ROS 2 Parameter Tuning (Roller Speeds)")
        layout = QVBoxLayout()

        # roller_speed_dribble_ab
        self.ab_label = QLabel(
            f'roller_speed_dribble_ab: {self.node.get_param("roller_speed_dribble_ab")}'
        )
        layout.addWidget(self.ab_label)

        self.ab_slider = QSlider(Qt.Horizontal)
        self.ab_slider.setRange(0, 100)
        self.ab_slider.setValue(self.node.get_param("roller_speed_dribble_ab"))
        self.ab_slider.valueChanged.connect(
            lambda value: self.update_param("roller_speed_dribble_ab", value)
        )
        self.ab_slider.setSingleStep(1)
        layout.addWidget(self.ab_slider)

        self.ab_input = QLineEdit(str(self.node.get_param("roller_speed_dribble_ab")))
        self.ab_input.textChanged.connect(
            lambda text: self.update_param_from_input("roller_speed_dribble_ab", text)
        )
        layout.addWidget(self.ab_input)

        # roller_speed_dribble_cd
        self.cd_label = QLabel(
            f'roller_speed_dribble_cd: {self.node.get_param("roller_speed_dribble_cd")}'
        )
        layout.addWidget(self.cd_label)

        self.cd_slider = QSlider(Qt.Horizontal)
        self.cd_slider.setRange(0, 100)
        self.cd_slider.setValue(self.node.get_param("roller_speed_dribble_cd"))
        self.cd_slider.valueChanged.connect(
            lambda value: self.update_param("roller_speed_dribble_cd", value)
        )
        self.cd_slider.setSingleStep(1)
        layout.addWidget(self.cd_slider)

        self.cd_input = QLineEdit(str(self.node.get_param("roller_speed_dribble_cd")))
        self.cd_input.textChanged.connect(
            lambda text: self.update_param_from_input("roller_speed_dribble_cd", text)
        )
        layout.addWidget(self.cd_input)

        # roller_speed_shoot_ab
        self.shoot_ab_label = QLabel(
            f'roller_speed_shoot_ab: {self.node.get_param("roller_speed_shoot_ab")}'
        )
        layout.addWidget(self.shoot_ab_label)

        self.shoot_ab_slider = QSlider(Qt.Horizontal)
        self.shoot_ab_slider.setRange(0, 100)
        self.shoot_ab_slider.setValue(self.node.get_param("roller_speed_shoot_ab"))
        self.shoot_ab_slider.valueChanged.connect(
            lambda value: self.update_param("roller_speed_shoot_ab", value)
        )
        self.shoot_ab_slider.setSingleStep(1)
        layout.addWidget(self.shoot_ab_slider)

        self.shoot_ab_input = QLineEdit(
            str(self.node.get_param("roller_speed_shoot_ab"))
        )
        self.shoot_ab_input.textChanged.connect(
            lambda text: self.update_param_from_input("roller_speed_shoot_ab", text)
        )
        layout.addWidget(self.shoot_ab_input)

        # roller_speed_shoot_cd
        self.shoot_cd_label = QLabel(
            f'roller_speed_shoot_cd: {self.node.get_param("roller_speed_shoot_cd")}'
        )
        layout.addWidget(self.shoot_cd_label)

        self.shoot_cd_slider = QSlider(Qt.Horizontal)
        self.shoot_cd_slider.setRange(0, 100)
        self.shoot_cd_slider.setValue(self.node.get_param("roller_speed_shoot_cd"))
        self.shoot_cd_slider.valueChanged.connect(
            lambda value: self.update_param("roller_speed_shoot_cd", value)
        )
        self.shoot_cd_slider.setSingleStep(1)
        layout.addWidget(self.shoot_cd_slider)

        self.shoot_cd_input = QLineEdit(
            str(self.node.get_param("roller_speed_shoot_cd"))
        )
        self.shoot_cd_input.textChanged.connect(
            lambda text: self.update_param_from_input("roller_speed_shoot_cd", text)
        )
        layout.addWidget(self.shoot_cd_input)

        # Shoot Button
        shoot_button = QPushButton("Shoot")
        shoot_button.clicked.connect(self.shoot_action)
        layout.addWidget(shoot_button)

        # Dribble Button
        dribble_button = QPushButton("Dribble")
        dribble_button.clicked.connect(self.dribble_action)
        layout.addWidget(dribble_button)

        self.setLayout(layout)

        # Update labels periodically
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_labels)
        self.timer.start(1000)

    def update_param(self, name, value):
        self.node.set_param(name, value)

    def update_param_from_input(self, name, text):
        try:
            value = int(text)
            if 0 <= value <= 100:
                self.node.set_param(name, value)
                if name == "roller_speed_dribble_ab":
                    self.ab_slider.setValue(value)
                elif name == "roller_speed_dribble_cd":
                    self.cd_slider.setValue(value)
                elif name == "roller_speed_shoot_ab":
                    self.shoot_ab_slider.setValue(value)
                elif name == "roller_speed_shoot_cd":
                    self.shoot_cd_slider.setValue(value)
        except ValueError:
            pass

    def refresh_labels(self):
        self.ab_label.setText(
            f'roller_speed_dribble_ab: {self.node.get_param("roller_speed_dribble_ab")}'
        )
        self.cd_label.setText(
            f'roller_speed_dribble_cd: {self.node.get_param("roller_speed_dribble_cd")}'
        )
        self.shoot_ab_label.setText(
            f'roller_speed_shoot_ab: {self.node.get_param("roller_speed_shoot_ab")}'
        )
        self.shoot_cd_label.setText(
            f'roller_speed_shoot_cd: {self.node.get_param("roller_speed_shoot_cd")}'
        )
        self.ab_input.setText(str(self.node.get_param("roller_speed_dribble_ab")))
        self.cd_input.setText(str(self.node.get_param("roller_speed_dribble_cd")))
        self.shoot_ab_input.setText(str(self.node.get_param("roller_speed_shoot_ab")))
        self.shoot_cd_input.setText(str(self.node.get_param("roller_speed_shoot_cd")))

    def shoot_action(self):
        self.node.set_shoot_state(1)
        QTimer.singleShot(
            100, lambda: self.node.set_shoot_state(0)
        )  # 0.5秒後に状態をリセット

    def dribble_action(self):
        self.node.set_dribble_state(1)
        QTimer.singleShot(
            100, lambda: self.node.set_dribble_state(0)
        )  # 0.5秒後に状態をリセット


def main():
    rclpy.init()
    node = ParameterNode()

    app = QApplication(sys.argv)
    gui = ParameterGUI(node)
    gui.show()

    # Keep ROS 2 node spinning
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(100)

    sys.exit(app.exec())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
