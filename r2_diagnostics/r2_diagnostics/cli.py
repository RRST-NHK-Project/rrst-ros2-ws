"""
Command-line interface for running diagnostics
"""

import argparse
import sys
import time

from .hardware_diagnostics import HardwareDiagnostics, DiagnosticStatus
from .sensor_diagnostics import SensorDiagnostics, SensorType
from .utils import (
    print_colored_text,
    create_diagnostics_report,
    save_diagnostics_report,
)


def main(args=None):
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(description="R2 Robot Diagnostics CLI")
    parser.add_argument(
        "command",
        choices=["hardware", "sensors", "full", "report"],
        help="Diagnostic command to run",
    )
    parser.add_argument(
        "--output", "-o", type=str, help="Output file for report (JSON or YAML)"
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")

    parsed_args = parser.parse_args(args)

    if parsed_args.command == "hardware":
        run_hardware_diagnostics(parsed_args)
    elif parsed_args.command == "sensors":
        run_sensor_diagnostics(parsed_args)
    elif parsed_args.command == "full":
        run_full_diagnostics(parsed_args)
    elif parsed_args.command == "report":
        generate_report(parsed_args)


def run_hardware_diagnostics(args):
    """Run hardware diagnostics"""
    print(print_colored_text("\n=== Hardware Diagnostics ===\n", "cyan"))

    hw_diag = HardwareDiagnostics()

    # Simulate hardware states for testing
    for i in range(1, 9):
        hw_diag.set_motor_state(i, float(i * 10))
    for i in range(1, 9):
        hw_diag.set_servo_state(i, float(i * 15))
    for i in range(1, 8):
        hw_diag.set_solenoid_state(i, i % 2)

    # Run diagnostics
    summary = hw_diag.run_full_diagnostics()

    # Print results
    for result in summary.results:
        if result.status == DiagnosticStatus.PASS:
            color = "green"
        elif result.status == DiagnosticStatus.WARNING:
            color = "yellow"
        else:
            color = "red"

        print(print_colored_text(str(result), color))

    # Print summary
    print(
        "\n"
        + print_colored_text(
            f"Summary: {summary.passed}/{summary.total_tests} passed "
            f"({summary.success_rate:.1f}%), "
            f"{summary.failed} failed, {summary.warnings} warnings",
            "bold",
        )
    )

    # Save if requested
    if args.output:
        report = create_diagnostics_report(summary)
        save_diagnostics_report(report, args.output)
        print(print_colored_text(f"\nReport saved to: {args.output}", "green"))


def run_sensor_diagnostics(args):
    """Run sensor diagnostics"""
    print(print_colored_text("\n=== Sensor Diagnostics ===\n", "cyan"))

    sensor_diag = SensorDiagnostics()

    # Register sensors
    sensor_diag.register_sensor(
        SensorType.ENCODER, "left", min_limit=0, max_limit=100000
    )
    sensor_diag.register_sensor(
        SensorType.ENCODER, "right", min_limit=0, max_limit=100000
    )
    sensor_diag.register_sensor(SensorType.SWITCH, "micro1", min_limit=0, max_limit=1)
    sensor_diag.register_sensor(SensorType.SWITCH, "micro2", min_limit=0, max_limit=1)

    # Update readings
    sensor_diag.update_reading(SensorType.ENCODER, "left", 5000, "pulses")
    sensor_diag.update_reading(SensorType.ENCODER, "right", 4950, "pulses")
    sensor_diag.update_reading(SensorType.SWITCH, "micro1", 1, "")
    sensor_diag.update_reading(SensorType.SWITCH, "micro2", 0, "")

    # Check health
    print(print_colored_text(sensor_diag.get_all_readings_summary(), "green"))


def run_full_diagnostics(args):
    """Run full diagnostics"""
    print(print_colored_text("\n=== Full R2 Diagnostics ===\n", "cyan"))
    print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    # Run hardware diagnostics
    run_hardware_diagnostics(args)

    # Run sensor diagnostics
    print("\n")
    run_sensor_diagnostics(args)


def generate_report(args):
    """Generate and save diagnostic report"""
    print(print_colored_text("Generating diagnostics report...\n", "cyan"))

    if not args.output:
        print(
            print_colored_text(
                "Error: --output is required for report generation", "red"
            )
        )
        sys.exit(1)

    # Run diagnostics
    hw_diag = HardwareDiagnostics()
    for i in range(1, 9):
        hw_diag.set_motor_state(i, float(i * 10))
    for i in range(1, 9):
        hw_diag.set_servo_state(i, float(i * 15))

    summary = hw_diag.run_full_diagnostics()
    report = create_diagnostics_report(summary)

    # Save report
    save_diagnostics_report(report, args.output)
    print(print_colored_text(f"Report saved to: {args.output}", "green"))


if __name__ == "__main__":
    main()
