# Autonomous Table Tennis Ball Collecting Robot

Autonomous differential drive robot centering on a table tennis ball via two‑layer control (vision guidance PI + per‑wheel speed PI).

## Architecture (Summary)
Vision (Pixy2 centroid) → lateral error ε → Guidance PI → (ω_L_ref, ω_R_ref) → Wheel PI (encoders) → Motors.
<div align="center"><img src="pictures/architecture_diagram.png" alt="Architecture" width="520" /></div>

## Core Equations
Differential drive:  
\( \omega_z = \frac{R}{L}(\omega_R-\omega_L),\; v=\frac{R}{2}(\omega_R+\omega_L) \)  
Bearing (small angle): \( \varepsilon = (x_{px}-c_x)/f_x \)  
Guidance PI: \( u = K_{p,g}\varepsilon + K_{i,g}\int \varepsilon dt \),  \( \Delta\omega = k_u u \)  
Refs: \( \omega_{L,ref}=\omega_f-\Delta\omega,\; \omega_{R,ref}=\omega_f+\Delta\omega \)  
Motor model: \( G_m(s)=K_m/(1+T_0 s) \)  
Incremental PI (wheel): \( u_k=u_{k-1}+K_{p,s}(e_k-e_{k-1})+K_{i,s}T_s e_k \)

## Key Metrics
| Item | Value |
|------|-------|
| Lateral accuracy | ±0.5–1 cm |
| Speed loop rate | 100 Hz |
| Motor T₀ (example) | ~35 ms |
| Phase margin (speed loop) | ~66° |
| Gain margin | ≫ 30 dB |

## Prototype Delta (Essentials)
| Aspect | Proto1 (Servos) | Proto2 (DC+Enc) |
|--------|-----------------|-----------------|
| Drive | Servos (open) | DC + encoders |
| Inner loop | None | PI speed |
| Bottleneck | Saturation/asymmetry | Vision latency |

## Quick Start
1. Open `firmware/prototype1/servo_guidance_pi.ino` (basic guidance) or `firmware/prototype2/motor_speed_pi.ino` (nested).  
2. Adjust pinout + `TICKS_PER_REV`.  
3. (Optional) Re‑identify motor: `python modeling/motor_identification.py`.  
4. Flash & monitor @115200 baud.  
5. Integrate your vision module by setting `vrefL`, `vrefR` (see prototype1 example).

## Logs & Tools
Prototype1 line: `ERR=-12,U=-9.6,L=-35,R=34`  
Prototype2 line: `wL=9.80,wR=9.75,vref=10.0,I_L=0.32,I_R=0.30`  
Parse: `python tools/log_parser.py logs/example_proto2.log`  
Export gains header: `python tools/export_gains.py params.json firmware/prototype2/gains_config.h`

## Structure
```
firmware/  (proto1 guidance PI, proto2 speed PI)
modeling/  (kinematics_sim, motor_identification)
tools/     (log_parser, export_gains)
data/      (step_response.csv)
logs/      (example log)
docs/      (architecture, future_work)
```

## Highlights
- Layered control (guidance vs motor)  
- Experimental identification (first‑order motor)  
- Stability margins > targets  
- Clear hardware pivot rationale  

## Roadmap & Docs
Roadmap: `docs/future_work.md`  
Details: `docs/architecture.md`

## License
MIT (see `LICENSE`).

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

## Prototype Evolution (Snapshot)
| Aspect | Prototype 1 (Servos) | Prototype 2 (DC + Encoders) |
|--------|----------------------|------------------------------|
| Actuation | Continuous rotation servos | DC motors + gearbox |
| Feedback | Camera lateral pixel only | Added wheel speed (encoders) |
| Control | P → PI on lateral error | Outer guidance + inner PI speed |
| Limits | Asymmetry, saturation | Vision latency, no range |
| Accuracy | ±0.5–1 cm (static ball) | Similar, faster convergence |
| Stability Margins | Not formalized early | Phase ≈ 66°, Gain → ∞ |

## Architecture Overview
Vision (centroid extraction) provides a lateral pixel error feeding a guidance PI which outputs differential wheel speed references. Inner per‑wheel PI speed loops track those references using encoder feedback.

<div align="center">
  <img src="pictures/architecture_diagram.png" alt="System Architecture Diagram" width="640" />
  <p><em>Architecture: vision extracts lateral error ε; guidance PI outputs differential speed references; per‑wheel PI loops regulate actual wheel speeds from encoder feedback.</em></p>
</div>

## Core Models & Control Laws

### 1. Differential Drive Kinematics
$$
\omega_z = \frac{R}{L}(\omega_R - \omega_L) \qquad\qquad v = \frac{R}{2}(\omega_R + \omega_L)
$$
Relates wheel angular speeds (\(\omega_L, \omega_R\)) to robot angular yaw rate \(\omega_z\) and linear velocity \(v\).

---
### 2. Optical Bearing (Small-Angle Approximation)
$$
\varepsilon = \frac{x_{px} - c_x}{f_x}
$$
Maps horizontal pixel offset to a lateral angular error; \(f_x\) is the horizontal focal length in pixels.

---
### 3. Guidance PI to Differential Speed
Guidance control law in the pixel / bearing domain:
$$
u(t)=K_{p,g}\,\varepsilon(t)+K_{i,g}\int_0^t \varepsilon(\tau)\,d\tau
$$
Converted to a differential wheel speed component (scaling \(k_u\)):
$$
\Delta\omega = k_u\,u
$$
Wheel speed references (around optional forward bias \(\omega_f\)):
$$
\omega_{L,ref}=\omega_f - \Delta\omega \qquad \omega_{R,ref}=\omega_f + \Delta\omega
$$

---
### 4. Identified Motor Model
First‑order approximation from step response identification:
$$
G_m(s)=\frac{\Omega(s)}{U(s)}=\frac{K_m}{1+T_0 s}
$$
Example dataset values: \(K_m\approx 4,\; T_0\approx 0.035\,\text{s}\).

---
### 5. Incremental PI Speed Regulation (Per Wheel)
Discrete (incremental) form minimizing floating drift:
$$
u_k = u_{k-1} + K_{p,s}\,(e_k - e_{k-1}) + K_{i,s} T_s e_k
$$
with error \(e_k = \omega_{ref,k} - \omega_k\).

---
### 6. Anti-Windup Principle
Integral state update is conditionally applied:
$$
I_{k}= I_{k-1} + K_{i,s} T_s e_k \quad \text{iff } u_{raw}\in[u_{min},u_{max}] \text{ or } e_k\, (u_{sat}-u_{raw})<0
$$
This prevents integrator growth that would worsen saturation.

## Key Metrics
- Lateral convergence: ±0.5–1 cm (static & slow rolling balls)
- Speed loop sampling: 100 Hz (SAMPLE_MS = 10 ms)
- Motor time constant: ~35 ms
- Phase margin (speed loop): ~66° (>45° target)
- Gain margin: ≫ 30 dB (no gain crossover)

## Typical Log Lines
Prototype 1 (guidance): `ERR=-12,U=-9.6,L=..,R=..`
Prototype 2 (speed loop): `wL=9.80,wR=9.75,vref=10.0,I_L=0.32,I_R=0.30`
Parse with: `python tools/log_parser.py logs/example_proto2.log`.

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
5. (Optional) Run `python modeling/motor_identification.py` to re‑fit motor params from a new step response.
6. (Optional) Export gains for firmware:
  ```bash
  python tools/export_gains.py params.json firmware/prototype2/gains_config.h
  ```

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

---
Further detail: `docs/architecture.md` (system) & `docs/future_work.md` (roadmap).

## Extended Write-up (Portfolio)
Full narrative, figures and experimental commentary are available in the portfolio HTML write‑up:

Local file (on your machine):
`file:///Users/promaa/Documents/Cloud/MegaSyncFiles/Code/Portfolio/ball-collecting-writeup.html`

> Note: A `file://` path is only accessible locally. For public viewers, consider one of:
> - Publishing the HTML through GitHub Pages (e.g. `docs/` + Pages) and linking the HTTPS URL.
> - Converting it to Markdown and placing it in `docs/` (e.g. `docs/full_writeup.md`).
> - Hosting the file on a static site (Netlify / Vercel) and updating this section.

Placeholder link (update when hosted): `[Hosted full write-up](https://your-hosted-url.example.com)`
