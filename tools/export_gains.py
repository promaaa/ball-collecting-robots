#!/usr/bin/env python3
"""Export control gains from JSON to Arduino header file.

Generates a C/C++ header file containing control gain definitions
that can be included in Arduino firmware.

Example usage:
    python export_gains.py params.json firmware/prototype2/gains_config.h
    python export_gains.py params.json output.h --preview
    
JSON input format:
    {"KP": 0.29, "KI": 8.93, "TI": 0.17}
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


# Default gain values if not specified in JSON
DEFAULT_GAINS = {
    "KP": 0.0,
    "KI": 0.0,
    "KD": 0.0,
    "TI": 0.0,
    "TD": 0.0,
}


def validate_gains(data: dict[str, Any]) -> dict[str, float]:
    """Validate and extract gain values from JSON data.
    
    Args:
        data: Parsed JSON data.
        
    Returns:
        Dictionary of validated gain values.
        
    Raises:
        ValueError: If any gain value is invalid.
    """
    gains: dict[str, float] = {}
    
    for key in DEFAULT_GAINS:
        if key in data:
            value = data[key]
            if not isinstance(value, (int, float)):
                raise ValueError(f"Invalid value for {key}: {value} (must be numeric)")
            gains[key] = float(value)
        else:
            gains[key] = DEFAULT_GAINS[key]
    
    return gains


def generate_header(gains: dict[str, float], source_file: str) -> str:
    """Generate C/C++ header file content.
    
    Args:
        gains: Dictionary of gain values.
        source_file: Name of source JSON file (for comment).
        
    Returns:
        Header file content as string.
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    lines = [
        "#pragma once",
        "/**",
        " * Auto-generated control gain configuration",
        f" * Source: {source_file}",
        f" * Generated: {timestamp}",
        " * ",
        " * Do not edit manually - regenerate using export_gains.py",
        " */",
        "",
        "// PI/PID Controller Gains",
        f"#define KP_GAIN {gains['KP']:.6f}f",
        f"#define KI_GAIN {gains['KI']:.6f}f",
    ]
    
    # Only include derivative gain if non-zero
    if gains['KD'] != 0.0:
        lines.append(f"#define KD_GAIN {gains['KD']:.6f}f")
    
    # Include time constants if non-zero
    if gains['TI'] != 0.0:
        lines.append(f"#define TI_TIME {gains['TI']:.6f}f  // Integral time constant (s)")
    
    if gains['TD'] != 0.0:
        lines.append(f"#define TD_TIME {gains['TD']:.6f}f  // Derivative time constant (s)")
    
    lines.extend([
        "",
        "// End of auto-generated configuration",
        ""
    ])
    
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.
    
    Returns:
        Parsed arguments namespace.
    """
    parser = argparse.ArgumentParser(
        description="Export control gains from JSON to Arduino header file.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "json_path",
        type=Path,
        help="Path to JSON file containing gain parameters"
    )
    parser.add_argument(
        "output_path",
        type=Path,
        help="Output path for generated header file"
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Print header content without writing to file"
    )
    parser.add_argument(
        "--force", "-f",
        action="store_true",
        help="Overwrite existing output file without prompting"
    )
    return parser.parse_args()


def main() -> int:
    """Main entry point for gain export.
    
    Returns:
        Exit code (0 for success, 1 for error).
    """
    args = parse_args()
    
    # Load and validate JSON input
    try:
        if not args.json_path.exists():
            print(f"Error: Input file not found: {args.json_path}", file=sys.stderr)
            return 1
        
        data = json.loads(args.json_path.read_text())
        gains = validate_gains(data)
        
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in {args.json_path}: {e}", file=sys.stderr)
        return 1
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    
    # Generate header content
    header_content = generate_header(gains, args.json_path.name)
    
    # Preview mode
    if args.preview:
        print("=== Generated Header ===")
        print(header_content)
        print("=== End of Preview ===")
        return 0
    
    # Check for existing file
    if args.output_path.exists() and not args.force:
        response = input(f"File {args.output_path} exists. Overwrite? [y/N] ")
        if response.lower() != 'y':
            print("Cancelled.")
            return 0
    
    # Write output
    try:
        args.output_path.parent.mkdir(parents=True, exist_ok=True)
        args.output_path.write_text(header_content)
        
        print(f"Generated header: {args.output_path}")
        print(f"  KP = {gains['KP']:.6f}")
        print(f"  KI = {gains['KI']:.6f}")
        if gains['KD'] != 0.0:
            print(f"  KD = {gains['KD']:.6f}")
        
        return 0
        
    except OSError as e:
        print(f"Error writing file: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
