"""
Utility functions for diagnostics
"""

import json
import yaml
from pathlib import Path
from typing import Dict, Any
from datetime import datetime


def format_timestamp(timestamp: float) -> str:
    """Format timestamp as readable string"""
    return datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def save_diagnostics_report(report_data: Dict[str, Any], output_file: str):
    """Save diagnostics report to file (JSON or YAML)"""
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if output_file.endswith(".json"):
        with open(output_file, "w") as f:
            json.dump(report_data, f, indent=2, default=str)
    elif output_file.endswith(".yaml") or output_file.endswith(".yml"):
        with open(output_file, "w") as f:
            yaml.dump(report_data, f, default_flow_style=False)
    else:
        raise ValueError(f"Unsupported file format: {output_file}")


def load_diagnostics_config(config_file: str) -> Dict[str, Any]:
    """Load diagnostics configuration from YAML/JSON"""
    config_path = Path(config_file)

    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")

    if config_file.endswith(".json"):
        with open(config_file, "r") as f:
            return json.load(f)
    elif config_file.endswith(".yaml") or config_file.endswith(".yml"):
        with open(config_file, "r") as f:
            return yaml.safe_load(f)
    else:
        raise ValueError(f"Unsupported config format: {config_file}")


def print_colored_text(text: str, color: str = "default") -> str:
    """Add ANSI color codes to text"""
    colors = {
        "default": "\033[0m",
        "red": "\033[91m",
        "green": "\033[92m",
        "yellow": "\033[93m",
        "blue": "\033[94m",
        "cyan": "\033[96m",
        "bold": "\033[1m",
        "reset": "\033[0m",
    }
    color_code = colors.get(color, colors["default"])
    reset_code = colors["reset"]
    return f"{color_code}{text}{reset_code}"


def create_diagnostics_report(summary: Any) -> Dict[str, Any]:
    """Create structured diagnostics report from summary"""
    report = {
        "timestamp": summary.timestamp,
        "timestamp_readable": format_timestamp(summary.timestamp),
        "summary": {
            "total_tests": summary.total_tests,
            "passed": summary.passed,
            "failed": summary.failed,
            "warnings": summary.warnings,
            "success_rate_percent": summary.success_rate,
        },
        "results": [
            {
                "component": r.component,
                "component_id": r.component_id,
                "status": r.status.value,
                "value": r.value,
                "expected": r.expected,
                "message": r.message,
                "timestamp": r.timestamp,
            }
            for r in summary.results
        ],
    }
    return report
