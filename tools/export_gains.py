"""Export control gains from JSON to Arduino header.
Usage:
  python tools/export_gains.py params.json firmware/prototype2/gains_config.h
JSON format: {"KP": 0.29, "KI": 8.93}
"""
import json, sys, pathlib

def main(src, dst):
    data = json.loads(pathlib.Path(src).read_text())
    lines = ["#pragma once","// Auto-generated gain configuration","#define KP_GAIN %.6ff"%data.get('KP',0.0),"#define KI_GAIN %.6ff"%data.get('KI',0.0),"// End"]
    pathlib.Path(dst).write_text("\n".join(lines)+"\n")
    print(f"Wrote header {dst}")

if __name__ == "__main__":
    if len(sys.argv)!=3:
        print("Usage: python tools/export_gains.py params.json firmware/prototype2/gains_config.h")
        raise SystemExit(1)
    main(sys.argv[1], sys.argv[2])
