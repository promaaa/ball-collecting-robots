#!/usr/bin/env python3
"""Serial log parser for robot prototypes.

Parses serial output logs from both robot prototypes and generates
summary statistics. Supports output in JSON or CSV format.

Prototype 1 format: ERR=...,U=...,L=...,R=...
Prototype 2 format: wL=...,wR=...,vref=...,I_L=...,I_R=...

Example usage:
    python log_parser.py logs/proto1.log
    python log_parser.py logs/proto2.log --csv -o results.csv
    python log_parser.py logs/proto2.log --plot
"""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from statistics import mean, stdev
from typing import Iterator

try:
    import matplotlib.pyplot as plt
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False


# Regex patterns for parsing log lines
RE_PROTO1 = re.compile(
    r"ERR=(?P<err>-?\d+),"
    r"U=(?P<u>-?\d+\.?\d*),"
    r"L=(?P<L>-?\d+),"
    r"R=(?P<R>-?\d+)"
)

RE_PROTO2 = re.compile(
    r"wL=(?P<wL>-?\d+\.?\d*),"
    r"wR=(?P<wR>-?\d+\.?\d*),"
    r"vref=(?P<vref>-?\d+\.?\d*),"
    r"I_L=(?P<I_L>-?\d+\.?\d*),"
    r"I_R=(?P<I_R>-?\d+\.?\d*)"
)


@dataclass
class Proto1Sample:
    """Single sample from Prototype 1 log."""
    err: float  # Pixel error
    u: float    # Control output
    L: float    # Left servo command
    R: float    # Right servo command


@dataclass
class Proto2Sample:
    """Single sample from Prototype 2 log."""
    wL: float    # Left wheel angular velocity (rad/s)
    wR: float    # Right wheel angular velocity (rad/s)
    vref: float  # Reference velocity
    I_L: float   # Left integral term
    I_R: float   # Right integral term


@dataclass
class ParseResult:
    """Parsed log data from both prototypes."""
    proto1_samples: list[Proto1Sample] = field(default_factory=list)
    proto2_samples: list[Proto2Sample] = field(default_factory=list)
    unparsed_lines: int = 0
    total_lines: int = 0


def parse_log_file(path: Path) -> ParseResult:
    """Parse a serial log file for prototype data.
    
    Args:
        path: Path to the log file.
        
    Returns:
        ParseResult containing samples from both prototypes.
        
    Raises:
        FileNotFoundError: If the log file doesn't exist.
    """
    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")
    
    result = ParseResult()
    
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            result.total_lines += 1
            
            # Try Prototype 1 format
            match1 = RE_PROTO1.search(line)
            if match1:
                result.proto1_samples.append(Proto1Sample(
                    err=float(match1.group('err')),
                    u=float(match1.group('u')),
                    L=float(match1.group('L')),
                    R=float(match1.group('R'))
                ))
                continue
            
            # Try Prototype 2 format
            match2 = RE_PROTO2.search(line)
            if match2:
                result.proto2_samples.append(Proto2Sample(
                    wL=float(match2.group('wL')),
                    wR=float(match2.group('wR')),
                    vref=float(match2.group('vref')),
                    I_L=float(match2.group('I_L')),
                    I_R=float(match2.group('I_R'))
                ))
                continue
            
            # Line didn't match any pattern
            if line.strip():  # Only count non-empty lines
                result.unparsed_lines += 1
    
    return result


def compute_statistics(result: ParseResult) -> dict:
    """Compute summary statistics from parsed data.
    
    Args:
        result: Parsed log data.
        
    Returns:
        Dictionary with computed statistics.
    """
    stats: dict = {
        "total_lines": result.total_lines,
        "unparsed_lines": result.unparsed_lines
    }
    
    if result.proto1_samples:
        errors = [abs(s.err) for s in result.proto1_samples]
        commands = [s.u for s in result.proto1_samples]
        
        stats["proto1"] = {
            "samples": len(result.proto1_samples),
            "mean_abs_error": mean(errors),
            "max_abs_error": max(errors),
            "std_error": stdev(errors) if len(errors) > 1 else 0.0,
            "mean_command": mean(commands),
            "command_range": [min(commands), max(commands)]
        }
    
    if result.proto2_samples:
        wL_values = [s.wL for s in result.proto2_samples]
        wR_values = [s.wR for s in result.proto2_samples]
        vref_values = [s.vref for s in result.proto2_samples]
        
        # Tracking error
        tracking_errors = [
            abs(s.wL - s.vref) + abs(s.wR - s.vref) 
            for s in result.proto2_samples
        ]
        
        stats["proto2"] = {
            "samples": len(result.proto2_samples),
            "mean_wL": mean(wL_values),
            "mean_wR": mean(wR_values),
            "std_wL": stdev(wL_values) if len(wL_values) > 1 else 0.0,
            "std_wR": stdev(wR_values) if len(wR_values) > 1 else 0.0,
            "mean_vref": mean(vref_values),
            "mean_tracking_error": mean(tracking_errors)
        }
    
    return stats


def export_csv(result: ParseResult, output_path: Path) -> None:
    """Export parsed samples to CSV files.
    
    Creates separate files for each prototype with _proto1.csv and _proto2.csv suffixes.
    
    Args:
        result: Parsed log data.
        output_path: Base output path (suffixes will be added).
    """
    base_name = output_path.stem
    output_dir = output_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if result.proto1_samples:
        proto1_path = output_dir / f"{base_name}_proto1.csv"
        with open(proto1_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['err', 'u', 'L', 'R'])
            for s in result.proto1_samples:
                writer.writerow([s.err, s.u, s.L, s.R])
        print(f"Proto1 data exported to: {proto1_path}")
    
    if result.proto2_samples:
        proto2_path = output_dir / f"{base_name}_proto2.csv"
        with open(proto2_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wL', 'wR', 'vref', 'I_L', 'I_R'])
            for s in result.proto2_samples:
                writer.writerow([s.wL, s.wR, s.vref, s.I_L, s.I_R])
        print(f"Proto2 data exported to: {proto2_path}")


def plot_data(result: ParseResult) -> None:
    """Plot parsed log data.
    
    Args:
        result: Parsed log data.
    """
    if not _HAS_MATPLOTLIB:
        print("Warning: matplotlib not available, skipping plot", file=sys.stderr)
        return
    
    if result.proto1_samples:
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
        fig.suptitle('Prototype 1: Servo-Based Guidance')
        
        samples = range(len(result.proto1_samples))
        errors = [s.err for s in result.proto1_samples]
        commands = [s.u for s in result.proto1_samples]
        
        axes[0].plot(samples, errors, 'b-', linewidth=0.8)
        axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[0].set_ylabel('Pixel Error')
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(samples, commands, 'r-', linewidth=0.8)
        axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[1].set_xlabel('Sample')
        axes[1].set_ylabel('Control Command')
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    if result.proto2_samples:
        fig, axes = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
        fig.suptitle('Prototype 2: DC Motor Speed Control')
        
        samples = range(len(result.proto2_samples))
        wL = [s.wL for s in result.proto2_samples]
        wR = [s.wR for s in result.proto2_samples]
        vref = [s.vref for s in result.proto2_samples]
        I_L = [s.I_L for s in result.proto2_samples]
        I_R = [s.I_R for s in result.proto2_samples]
        
        axes[0].plot(samples, wL, 'b-', label='ωL', linewidth=0.8)
        axes[0].plot(samples, wR, 'g-', label='ωR', linewidth=0.8)
        axes[0].plot(samples, vref, 'r--', label='vref', linewidth=1.2)
        axes[0].set_ylabel('Angular Velocity (rad/s)')
        axes[0].legend(loc='upper right')
        axes[0].grid(True, alpha=0.3)
        
        axes[1].plot(samples, I_L, 'b-', label='I_L', linewidth=0.8)
        axes[1].plot(samples, I_R, 'g-', label='I_R', linewidth=0.8)
        axes[1].set_xlabel('Sample')
        axes[1].set_ylabel('Integral Term')
        axes[1].legend(loc='upper right')
        axes[1].grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    if result.proto1_samples or result.proto2_samples:
        plt.show()
    else:
        print("No data to plot")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.
    
    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Parse serial logs from robot prototypes and generate statistics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "log_path",
        type=Path,
        help="Path to serial log file"
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Output path for CSV export (enables CSV mode)"
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Export raw samples to CSV files"
    )
    parser.add_argument(
        "--plot", "-p",
        action="store_true",
        help="Plot the parsed data"
    )
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Suppress JSON output (useful with --csv or --plot)"
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for log parsing.
    
    Returns:
        Exit code (0 for success, 1 for error).
    """
    args = parse_args()
    
    try:
        result = parse_log_file(args.log_path)
        
        if not result.proto1_samples and not result.proto2_samples:
            print("Warning: No valid data found in log file", file=sys.stderr)
            return 1
        
        print(f"Parsed {args.log_path}: "
              f"{len(result.proto1_samples)} Proto1 samples, "
              f"{len(result.proto2_samples)} Proto2 samples")
        
        # Compute and display statistics
        stats = compute_statistics(result)
        if not args.quiet:
            print(json.dumps(stats, indent=2))
        
        # Export to CSV if requested
        if args.csv:
            output_path = args.output or Path("output/parsed_data.csv")
            export_csv(result, output_path)
        
        # Plot if requested
        if args.plot:
            plot_data(result)
        
        return 0
        
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
