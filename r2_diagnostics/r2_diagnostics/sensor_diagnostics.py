"""
Sensor diagnostics module for sensor health verification
"""

import time
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Tuple


class SensorType(Enum):
    """Sensor type enumeration"""

    ENCODER = "encoder"
    SWITCH = "switch"
    IMU = "imu"
    LIDAR = "lidar"
    CAMERA = "camera"


@dataclass
class SensorReading:
    """Single sensor reading"""

    sensor_type: SensorType
    sensor_id: str
    value: float
    unit: str = ""
    timestamp: float = 0.0
    valid: bool = True


class SensorDiagnostics:
    """Sensor diagnostics and data verification"""

    def __init__(self):
        """Initialize sensor diagnostics"""
        self.sensor_readings: Dict[str, SensorReading] = {}
        self.sensor_limits: Dict[str, Tuple[float, float]] = {}

    def register_sensor(
        self,
        sensor_type: SensorType,
        sensor_id: str,
        min_limit: float = None,
        max_limit: float = None,
    ):
        """Register a sensor for monitoring"""
        key = f"{sensor_type.value}_{sensor_id}"
        if min_limit is not None and max_limit is not None:
            self.sensor_limits[key] = (min_limit, max_limit)

    def update_reading(
        self, sensor_type: SensorType, sensor_id: str, value: float, unit: str = ""
    ):
        """Update sensor reading"""
        key = f"{sensor_type.value}_{sensor_id}"

        # Check limits if defined
        valid = True
        if key in self.sensor_limits:
            min_limit, max_limit = self.sensor_limits[key]
            valid = min_limit <= value <= max_limit

        reading = SensorReading(
            sensor_type=sensor_type,
            sensor_id=sensor_id,
            value=value,
            unit=unit,
            timestamp=time.time(),
            valid=valid,
        )
        self.sensor_readings[key] = reading
        return valid

    def get_reading(self, sensor_type: SensorType, sensor_id: str) -> SensorReading:
        """Get latest sensor reading"""
        key = f"{sensor_type.value}_{sensor_id}"
        return self.sensor_readings.get(key)

    def check_sensor_health(
        self, sensor_type: SensorType, sensor_id: str, timeout: float = 1.0
    ) -> Tuple[bool, str]:
        """Check sensor health (connection and recent data)"""
        key = f"{sensor_type.value}_{sensor_id}"

        if key not in self.sensor_readings:
            return False, f"{sensor_type.value} {sensor_id} has no data"

        reading = self.sensor_readings[key]
        time_diff = time.time() - reading.timestamp

        if time_diff > timeout:
            return False, f"{sensor_type.value} {sensor_id} data timeout (>{timeout}s)"

        if not reading.valid:
            return False, f"{sensor_type.value} {sensor_id} value out of range"

        return (
            True,
            f"{sensor_type.value} {sensor_id} healthy ({reading.value}{reading.unit})",
        )

    def get_all_readings_summary(self) -> str:
        """Get summary of all sensor readings"""
        if not self.sensor_readings:
            return "No sensor readings"

        lines = ["=== Sensor Readings ==="]
        for key, reading in self.sensor_readings.items():
            status = "OK" if reading.valid else "OUT OF RANGE"
            lines.append(
                f"{reading.sensor_type.value.upper()} {reading.sensor_id}: "
                f"{reading.value}{reading.unit} [{status}]"
            )
        return "\n".join(lines)
