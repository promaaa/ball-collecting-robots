# Autonomous Table Tennis Ball Collecting Robot

Centimeter‑level, vision-guided differential drive robot for table tennis ball interception. Two iterative prototypes: servo (P/PI) then DC motors + encoders (nested PI). This repository is a curated, cleaned version of the original exploratory workspace.

## Repository Structure
- docs/ – Detailed technical write‑ups (architecture, control design, experimental results).
- firmware/
  - prototype1/ – Minimal Pixy + servo PI guidance sketch.
  - prototype2/ – Encoder-based DC motor speed PI regulation + guidance interface skeleton.
- modeling/ – Python scripts for kinematic simulation & motor identification.
- tools/ – Log parsing and parameter export utilities.

## Highlights
- Guidance: camera lateral pixel error -> differential speed reference.
- Prototype1 limits: servo asymmetry + saturation.
- Prototype2 upgrades: DC motors + gearbox + incremental encoders; speed loop decouples actuation from guidance.
- Identified motor first-order model G(p)=Km/(1+T0 p), tuned PI (Kp=44, Ti=0.17 s) with ample stability margins.

## Portfolio Snapshot
- Iterative control refinement: P → PI → cascaded PI with experimental stability margins (phase 66° > 45° target).
- Hardware pivot justified by logged servo asymmetry & saturation metrics.
- Clear bottleneck migration from actuation to perception (maturity indicator).
- Lightweight, layered architecture ready for perception upgrades (depth / CNN) without rewriting motor layer.

## Directory Tree (Curated)
```
autonomous-ball-collector/
  README.md
  CHANGELOG.md
  LICENSE
  requirements.txt
  docs/
    architecture.md
    future_work.md
    figures/ (placeholders)
  firmware/
    prototype1/servo_guidance_pi.ino
    prototype2/motor_speed_pi.ino
  modeling/
    kinematics_sim.py
    motor_identification.py
  tools/
    log_parser.py
    export_gains.py
  data/step_response.csv
  logs/example_proto2.log
```

## Quick Start
1. Open firmware/prototype1/servo_guidance_pi.ino or firmware/prototype2/motor_speed_pi.ino in Arduino IDE.
2. Adjust pin mappings & gains in the CONFIG section.
3. (Prototype2) Run modeling/motor_identification.py with collected step response CSV to refine Km, T0.
4. Flash and monitor serial at 115200 baud for logs.

## Next Steps
See docs/future_work.md for perception & robustness roadmap.

---
Curated by refactor script.

## Contributing
1. Fork & create a feature branch.
2. Add or update tests / example logs if behavior changes.
3. Run lint & basic simulations (modeling scripts) before PR.
4. Document new parameters in README or architecture doc.

## License
MIT – see LICENSE file.
