#!/usr/bin/env python3
"""Differential drive kinematic simulation for lateral centering.

This module simulates the PI guidance control for a differential-drive robot
tracking a target (ball) using camera-based lateral error feedback. The camera
provides lateral pixel error proportional to heading error under small-angle
approximation.

The simulation models:
    - Robot state: lateral offset (y) and heading (theta)
    - PI controller: proportional + integral action on pixel error
    - Differential steering: heading rate proportional to control output

Example usage:
    python kinematics_sim.py --kp 1.2 --ki 0.5 --duration 4.0
    python kinematics_sim.py --initial-offset 0.2 --no-plot
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False


class SimulationResult(NamedTuple):
    """Results from a kinematic simulation run."""
    time: np.ndarray
    lateral_offset: np.ndarray
    heading: np.ndarray
    pixel_error: np.ndarray
    control_output: np.ndarray


@dataclass
class SimulationConfig:
    """Configuration parameters for the kinematic simulation.
    
    Attributes:
        kp: Proportional gain for PI controller.
        ki: Integral gain for PI controller (0 for pure P control).
        sample_time: Control loop sample time in seconds.
        duration: Total simulation duration in seconds.
        forward_speed: Constant forward speed in m/s.
        fov_scale: Camera field-of-view scale (pixels per radian).
        detection_distance: Typical detection distance for pixel mapping in meters.
        initial_offset: Initial lateral offset from target in meters.
        initial_heading: Initial heading angle in radians.
    """
    kp: float = 1.2
    ki: float = 0.5
    sample_time: float = 0.02
    duration: float = 4.0
    forward_speed: float = 0.25
    fov_scale: float = 320 / 0.6  # pixels per radian
    detection_distance: float = 0.6
    initial_offset: float = 0.15
    initial_heading: float = 0.0


def run_simulation(config: SimulationConfig) -> SimulationResult:
    """Run the kinematic simulation with PI guidance control.
    
    Simulates a differential-drive robot approaching a target using
    vision-based lateral error feedback and PI control.
    
    Args:
        config: Simulation configuration parameters.
        
    Returns:
        SimulationResult containing time series of all state variables.
    """
    steps = int(config.duration / config.sample_time)
    
    # Pre-allocate arrays for results
    time_arr = np.arange(steps) * config.sample_time
    y_arr = np.zeros(steps)
    theta_arr = np.zeros(steps)
    error_arr = np.zeros(steps)
    control_arr = np.zeros(steps)
    
    # Initialize state
    y = config.initial_offset
    theta = config.initial_heading
    error_integral = 0.0
    
    # Pixel mapping constant
    pixel_per_meter = 320 / config.detection_distance
    
    for k in range(steps):
        # Camera model: pixel error from lateral offset
        # Assumes small-angle approximation at detection distance
        pixel_error = y * pixel_per_meter
        
        # PI control law
        error_integral += pixel_error * config.sample_time
        control = config.kp * pixel_error + config.ki * error_integral
        
        # Store results
        y_arr[k] = y
        theta_arr[k] = theta
        error_arr[k] = pixel_error
        control_arr[k] = control
        
        # State update
        # Differential steering: control maps to heading rate
        theta += (-control / config.fov_scale) * config.sample_time
        # Lateral motion: y += v * sin(theta) * dt
        y += config.forward_speed * np.sin(theta) * config.sample_time
    
    return SimulationResult(
        time=time_arr,
        lateral_offset=y_arr,
        heading=theta_arr,
        pixel_error=error_arr,
        control_output=control_arr
    )


def plot_results(result: SimulationResult, config: SimulationConfig) -> None:
    """Plot the simulation results.
    
    Args:
        result: Simulation result to plot.
        config: Configuration used (for annotation).
    """
    if not _HAS_MATPLOTLIB:
        print("Warning: matplotlib not available, skipping plot", file=sys.stderr)
        return
    
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    
    # Lateral offset
    axes[0].plot(result.time, result.lateral_offset * 100, 'b-', linewidth=1.5)
    axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[0].set_ylabel('Lateral Offset (cm)')
    axes[0].set_title(f'PI Guidance Simulation (Kp={config.kp}, Ki={config.ki})')
    axes[0].grid(True, alpha=0.3)
    
    # Heading
    axes[1].plot(result.time, np.degrees(result.heading), 'g-', linewidth=1.5)
    axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
    axes[1].set_ylabel('Heading (deg)')
    axes[1].grid(True, alpha=0.3)
    
    # Control signals
    axes[2].plot(result.time, result.pixel_error, 'r-', label='Pixel Error', linewidth=1.5)
    axes[2].plot(result.time, result.control_output, 'b-', label='Control Output', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Signal')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def print_summary(result: SimulationResult) -> None:
    """Print a summary of the simulation results.
    
    Args:
        result: Simulation result to summarize.
    """
    final_offset_cm = result.lateral_offset[-1] * 100
    final_heading_deg = np.degrees(result.heading[-1])
    max_offset_cm = np.max(np.abs(result.lateral_offset)) * 100
    
    print("=" * 50)
    print("Simulation Summary")
    print("=" * 50)
    print(f"Final lateral offset:  {final_offset_cm:+.2f} cm")
    print(f"Final heading:         {final_heading_deg:+.2f} deg")
    print(f"Max lateral offset:    {max_offset_cm:.2f} cm")
    print(f"Steady-state error:    {abs(final_offset_cm):.2f} cm")
    print("=" * 50)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.
    
    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Differential drive kinematic simulation with PI guidance control.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--kp", type=float, default=1.2,
        help="Proportional gain"
    )
    parser.add_argument(
        "--ki", type=float, default=0.5,
        help="Integral gain (0 for pure P control)"
    )
    parser.add_argument(
        "--duration", "-t", type=float, default=4.0,
        help="Simulation duration in seconds"
    )
    parser.add_argument(
        "--sample-time", "-s", type=float, default=0.02,
        help="Sample time in seconds"
    )
    parser.add_argument(
        "--initial-offset", "-y", type=float, default=0.15,
        help="Initial lateral offset in meters"
    )
    parser.add_argument(
        "--forward-speed", "-v", type=float, default=0.25,
        help="Forward speed in m/s"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Disable plotting (text output only)"
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for the simulation.
    
    Returns:
        Exit code (0 for success).
    """
    args = parse_args()
    
    config = SimulationConfig(
        kp=args.kp,
        ki=args.ki,
        duration=args.duration,
        sample_time=args.sample_time,
        initial_offset=args.initial_offset,
        forward_speed=args.forward_speed,
    )
    
    print(f"Running simulation: Kp={config.kp}, Ki={config.ki}, "
          f"duration={config.duration}s, y0={config.initial_offset}m")
    
    result = run_simulation(config)
    print_summary(result)
    
    if not args.no_plot:
        plot_results(result, config)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
