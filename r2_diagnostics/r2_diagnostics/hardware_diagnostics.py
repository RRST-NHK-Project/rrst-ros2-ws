"""
Hardware diagnostics module for actuator testing and verification
"""

import time
from enum import Enum
from dataclasses import dataclass
from typing import Dict, List, Tuple


class DiagnosticStatus(Enum):
    """Diagnostic test status enumeration"""

    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    NOT_TESTED = "NOT_TESTED"


@dataclass
class DiagnosticResult:
    """Single diagnostic test result"""

    component: str
    component_id: int
    status: DiagnosticStatus
    value: float = 0.0
    expected: float = 0.0
    message: str = ""
    timestamp: float = 0.0

    def __str__(self):
        return f"[{self.status.value:12}] {self.component}[{self.component_id}]: {self.message}"


@dataclass
class DiagnosticSummary:
    """Summary of all diagnostic results"""

    timestamp: float
    total_tests: int
    passed: int
    failed: int
    warnings: int
    results: List[DiagnosticResult]

    @property
    def success_rate(self) -> float:
        """Calculate success rate as percentage"""
        if self.total_tests == 0:
            return 0.0
        return (self.passed / self.total_tests) * 100.0

    def __str__(self):
        summary_line = (
            f"Diagnostics Summary: {self.passed}/{self.total_tests} passed "
            f"({self.success_rate:.1f}%), {self.failed} failed, {self.warnings} warnings"
        )
        results_str = "\n".join(str(r) for r in self.results)
        return f"{summary_line}\n{results_str}"


class HardwareDiagnostics:
    """Hardware component diagnostics"""

    def __init__(self):
        """Initialize hardware diagnostics"""
        self.results: List[DiagnosticResult] = []
        self.motor_states: Dict[int, float] = {}
        self.servo_states: Dict[int, float] = {}
        self.solenoid_states: Dict[int, int] = {}
        self.encoder_values: Dict[int, int] = {}
        self.switch_states: Dict[str, bool] = {}

    def reset(self):
        """Reset all diagnostic results"""
        self.results.clear()

    def add_result(
        self,
        component: str,
        component_id: int,
        status: DiagnosticStatus,
        value: float = 0.0,
        expected: float = 0.0,
        message: str = "",
    ):
        """Add a diagnostic result"""
        result = DiagnosticResult(
            component=component,
            component_id=component_id,
            status=status,
            value=value,
            expected=expected,
            message=message,
            timestamp=time.time(),
        )
        self.results.append(result)

    def get_summary(self) -> DiagnosticSummary:
        """Get diagnostic summary"""
        passed = sum(1 for r in self.results if r.status == DiagnosticStatus.PASS)
        failed = sum(1 for r in self.results if r.status == DiagnosticStatus.FAIL)
        warnings = sum(1 for r in self.results if r.status == DiagnosticStatus.WARNING)
        total = len(self.results)

        return DiagnosticSummary(
            timestamp=time.time(),
            total_tests=total,
            passed=passed,
            failed=failed,
            warnings=warnings,
            results=self.results,
        )

    def check_motor_response(
        self, motor_id: int, timeout: float = 0.5
    ) -> Tuple[bool, str]:
        """Check if motor responds to commands"""
        if motor_id < 1 or motor_id > 8:
            return False, f"Invalid motor ID: {motor_id}"

        # Simulate motor response check (would interface with serial/ROS in real implementation)
        status = motor_id in self.motor_states
        message = f"Motor MD{motor_id} {'responding' if status else 'not responding'}"
        return status, message

    def check_servo_response(
        self, servo_id: int, timeout: float = 0.5
    ) -> Tuple[bool, str]:
        """Check if servo responds to commands"""
        if servo_id < 1 or servo_id > 8:
            return False, f"Invalid servo ID: {servo_id}"

        status = servo_id in self.servo_states
        message = (
            f"Servo SERVO{servo_id} {'responding' if status else 'not responding'}"
        )
        return status, message

    def check_solenoid_response(
        self, solenoid_id: int, timeout: float = 0.5
    ) -> Tuple[bool, str]:
        """Check if solenoid responds to commands"""
        if solenoid_id < 1 or solenoid_id > 7:
            return False, f"Invalid solenoid ID: {solenoid_id}"

        status = solenoid_id in self.solenoid_states
        message = (
            f"Solenoid TR{solenoid_id} {'responding' if status else 'not responding'}"
        )
        return status, message

    def check_encoder(self, encoder_id: int) -> Tuple[bool, int, str]:
        """Check encoder value"""
        if encoder_id not in self.encoder_values:
            return False, 0, f"Encoder {encoder_id} not found"

        value = self.encoder_values[encoder_id]
        message = f"Encoder {encoder_id} reading: {value}"
        return True, value, message

    def check_switch(self, switch_name: str) -> Tuple[bool, bool, str]:
        """Check switch state"""
        if switch_name not in self.switch_states:
            return False, False, f"Switch {switch_name} not found"

        state = self.switch_states[switch_name]
        message = f"Switch {switch_name} is {'ON' if state else 'OFF'}"
        return True, state, message

    def run_full_diagnostics(self) -> DiagnosticSummary:
        """Run complete hardware diagnostics"""
        self.reset()

        # Test all motors
        for i in range(1, 9):
            status_ok, message = self.check_motor_response(i)
            status = DiagnosticStatus.PASS if status_ok else DiagnosticStatus.FAIL
            self.add_result("Motor", i, status, message=message)

        # Test all servos
        for i in range(1, 9):
            status_ok, message = self.check_servo_response(i)
            status = DiagnosticStatus.PASS if status_ok else DiagnosticStatus.FAIL
            self.add_result("Servo", i, status, message=message)

        # Test all solenoids
        for i in range(1, 8):
            status_ok, message = self.check_solenoid_response(i)
            status = DiagnosticStatus.PASS if status_ok else DiagnosticStatus.FAIL
            self.add_result("Solenoid", i, status, message=message)

        return self.get_summary()

    def set_motor_state(self, motor_id: int, value: float):
        """Set motor state for testing"""
        self.motor_states[motor_id] = value

    def set_servo_state(self, servo_id: int, value: float):
        """Set servo state for testing"""
        self.servo_states[servo_id] = value

    def set_solenoid_state(self, solenoid_id: int, value: int):
        """Set solenoid state for testing"""
        self.solenoid_states[solenoid_id] = value

    def set_encoder_value(self, encoder_id: int, value: int):
        """Set encoder reading"""
        self.encoder_values[encoder_id] = value

    def set_switch_state(self, switch_name: str, state: bool):
        """Set switch state"""
        self.switch_states[switch_name] = state
