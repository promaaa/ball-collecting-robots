"""Motor identification helper
Load a CSV of step response: time(s),omega(rad_s)
Fit first-order model G(s)=Km/(1+T0 s) by least squares on derivative form.
"""
import numpy as np
import json
from pathlib import Path

# Replace with your CSV path
CSV_PATH = Path("data/step_response.csv")
OUT_PARAMS = Path("data/motor_params.json")

if not CSV_PATH.exists():
    raise SystemExit(f"Missing CSV: {CSV_PATH} (expected columns: t,omega)")

t, w = np.loadtxt(CSV_PATH, delimiter=",", unpack=True)
# Assume input is unit step (u=1). Steady state gain Km ~ w_inf
Km = np.mean(w[-max(5,int(0.1*len(w))):])
# Time constant T0 via 63.2% rise time
w63 = 0.632 * Km
idx = np.where(w >= w63)[0]
if len(idx)==0:
    raise SystemExit("Could not find 63% point")
T0 = t[idx[0]] - t[0]
params = {"Km": float(Km), "T0": float(T0)}
print("Identified params:", params)
OUT_PARAMS.parent.mkdir(parents=True, exist_ok=True)
OUT_PARAMS.write_text(json.dumps(params, indent=2))
print("Saved to", OUT_PARAMS)
