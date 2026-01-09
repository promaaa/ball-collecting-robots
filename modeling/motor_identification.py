#!/usr/bin/env python3
"""Motor system identification from step response data.

This module fits a first-order transfer function model G(s) = Km / (1 + T0*s)
to experimental step response data. The identification uses:
    - Steady-state gain (Km): Mean of final values
    - Time constant (T0): Time to reach 63.2% of steady-state

Example usage:
    python motor_identification.py data/step_response.csv
    python motor_identification.py data/step_response.csv -o data/motor_params.json --plot
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import numpy as np

try:
    import matplotlib.pyplot as plt
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False


class MotorParams(NamedTuple):
    """Identified motor parameters."""
    km: float  # Steady-state gain
    t0: float  # Time constant (seconds)


@dataclass
class StepResponseData:
    """Step response measurement data."""
    time: np.ndarray
    omega: np.ndarray
    
    @classmethod
    def from_csv(cls, path: Path) -> "StepResponseData":
        """Load step response data from a CSV file.
        
        Args:
            path: Path to CSV file with columns: time, omega
            
        Returns:
            StepResponseData instance.
            
        Raises:
            FileNotFoundError: If the CSV file doesn't exist.
            ValueError: If the CSV format is invalid.
        """
        if not path.exists():
            raise FileNotFoundError(f"CSV file not found: {path}")
        
        try:
            # Handle comment lines starting with #
            data = np.loadtxt(path, delimiter=",", comments="#", unpack=True)
            if data.shape[0] != 2:
                raise ValueError(f"Expected 2 columns (time, omega), got {data.shape[0]}")
            return cls(time=data[0], omega=data[1])
        except Exception as e:
            raise ValueError(f"Failed to parse CSV file: {e}") from e


def identify_motor_params(data: StepResponseData) -> MotorParams:
    """Identify first-order motor model parameters from step response.
    
    Uses the 63.2% rise time method for time constant estimation.
    
    Args:
        data: Step response measurement data.
        
    Returns:
        Identified motor parameters (Km, T0).
        
    Raises:
        ValueError: If 63.2% point cannot be found in data.
    """
    if len(data.time) < 3:
        raise ValueError("Insufficient data points for identification")
    
    # Steady-state gain: average of last 10% of samples (minimum 5)
    n_final = max(5, int(0.1 * len(data.omega)))
    km = float(np.mean(data.omega[-n_final:]))
    
    if km <= 0:
        raise ValueError(f"Invalid steady-state gain: {km} (must be positive)")
    
    # Time constant: time to reach 63.2% of steady-state
    omega_63 = 0.632 * km
    indices = np.where(data.omega >= omega_63)[0]
    
    if len(indices) == 0:
        raise ValueError(
            f"Data does not reach 63.2% of steady-state ({omega_63:.2f}). "
            f"Max value: {np.max(data.omega):.2f}"
        )
    
    t0 = float(data.time[indices[0]] - data.time[0])
    
    return MotorParams(km=km, t0=t0)


def model_response(t: np.ndarray, params: MotorParams) -> np.ndarray:
    """Generate first-order model step response.
    
    Args:
        t: Time array.
        params: Motor parameters.
        
    Returns:
        Modeled omega values.
    """
    return params.km * (1 - np.exp(-t / params.t0))


def save_params(params: MotorParams, output_path: Path) -> None:
    """Save identified parameters to a JSON file.
    
    Args:
        params: Motor parameters to save.
        output_path: Output file path.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    data = {
        "Km": params.km,
        "T0": params.t0,
        "description": "First-order motor model: G(s) = Km / (1 + T0*s)"
    }
    
    output_path.write_text(json.dumps(data, indent=2) + "\n")
    print(f"Parameters saved to: {output_path}")


def plot_identification(
    data: StepResponseData,
    params: MotorParams,
    output_path: Path | None = None
) -> None:
    """Plot step response data with identified model overlay.
    
    Args:
        data: Measured step response data.
        params: Identified motor parameters.
        output_path: Optional path to save the figure.
    """
    if not _HAS_MATPLOTLIB:
        print("Warning: matplotlib not available, skipping plot", file=sys.stderr)
        return
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Measured data
    ax.plot(data.time, data.omega, 'bo-', markersize=4, label='Measured Data')
    
    # Model response
    t_model = np.linspace(0, data.time[-1], 200)
    omega_model = model_response(t_model, params)
    ax.plot(t_model, omega_model, 'r-', linewidth=2, label=f'Model (Km={params.km:.2f}, T0={params.t0:.3f}s)')
    
    # Reference lines
    ax.axhline(params.km, color='g', linestyle='--', alpha=0.5, label=f'Steady-state Km={params.km:.2f}')
    ax.axhline(0.632 * params.km, color='orange', linestyle=':', alpha=0.5, label='63.2% level')
    ax.axvline(params.t0, color='purple', linestyle=':', alpha=0.5, label=f'T0={params.t0:.3f}s')
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angular Velocity (rad/s)')
    ax.set_title('Motor Step Response Identification')
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, data.time[-1])
    ax.set_ylim(0, params.km * 1.1)
    
    plt.tight_layout()
    
    if output_path:
        fig.savefig(output_path, dpi=150)
        print(f"Plot saved to: {output_path}")
    
    plt.show()


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.
    
    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Identify first-order motor model from step response data.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="Path to CSV file with step response data (columns: time, omega)"
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=Path("data/motor_params.json"),
        help="Output path for identified parameters JSON"
    )
    parser.add_argument(
        "--plot", "-p",
        action="store_true",
        help="Show identification plot"
    )
    parser.add_argument(
        "--save-plot",
        type=Path,
        default=None,
        help="Save plot to file"
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for motor identification.
    
    Returns:
        Exit code (0 for success, 1 for error).
    """
    args = parse_args()
    
    try:
        print(f"Loading step response data from: {args.csv_path}")
        data = StepResponseData.from_csv(args.csv_path)
        print(f"Loaded {len(data.time)} data points")
        
        params = identify_motor_params(data)
        
        print("\n" + "=" * 50)
        print("Identified Motor Parameters")
        print("=" * 50)
        print(f"  Steady-state gain (Km): {params.km:.4f} rad/s")
        print(f"  Time constant (T0):     {params.t0:.4f} s")
        print(f"  Transfer function:      G(s) = {params.km:.2f} / (1 + {params.t0:.3f}s)")
        print("=" * 50)
        
        save_params(params, args.output)
        
        if args.plot or args.save_plot:
            plot_identification(data, params, args.save_plot)
        
        return 0
        
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
