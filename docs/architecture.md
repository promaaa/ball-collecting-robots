# Autonomous Table Tennis Ball Collecting Robot – Architecture & Design

(Condensed from the original long-form French document.)

## 1. Objective
Autonomous differential-drive robot that detects and centers on a 40 mm table tennis ball with centimeter‑level accuracy (±0.5–1 cm) and fast convergence.

## 2. Engineering Approach
Iterative cycle: Modeling → Simulation → Prototyping → Measurement → Gap analysis → Improvement.

## 3. Prototypes
### Prototype 1 (Servos + Basic Vision)
- Drive: 2 continuous rotation servos (approximate speed control via pulse width).
- Sensor: Pixy2 color camera → ball centroid x coordinate.
- Guidance control: P then PI on lateral pixel error.
- Limitations: servo asymmetry, speed saturation, no true wheel speed feedback.

### Prototype 2 (DC Motors + Encoders)
- Drive: DC motors + gear reduction + incremental encoders.
- Inner loop: per‑wheel PI speed (rad/s) → decouples guidance from raw PWM.
- Outcome: higher stability and acceleration; bottleneck shifts to perception (latency / robustness).

## 4. Models
### Differential Kinematics
Ω = (R/L)(Ω_r − Ω_l); basis for trajectory prediction under differential speed commands.

### Identified Motor Model
G(s) = K_m / (1 + T_0 s) using step response fitting (example values: K_m ≈ 4, T_0 ≈ 35 ms).

## 5. Control Layers
| Layer | Prototype 1 | Prototype 2 |
|-------|-------------|-------------|
| Lateral guidance | P → PI on pixel error | Same (output becomes speed ref) |
| Wheel speed | Open loop | PI (Kp, Ki) |
| Saturation handling | Servo pulse geometry | PWM + (potential) anti‑windup clamp |

Prototype 2 achieved: Phase margin ≈ 66° (> target 45°), Gain margin ∞ (> 30 dB target).

## 6. Vision → Command Flow (Proto1)
1. Acquire blocks (Pixy2 signature filtering)
2. Select largest valid block (noise rejection)
3. Error e = x − x_center
4. u = Kp·e + Ki∫e dt
5. Differential mapping → servo neutral (90) and extremes (0/180)

## 7. Guidance → Speed Flow (Proto2)
1. Same lateral error extraction
2. Convert guidance output u_g to differential speed Δω (around optional forward bias ω_fwd)
3. Produce ω_L_ref, ω_R_ref fed to per‑wheel PI speed loops
4. Inner loop compensates battery variation / friction for consistent dynamics

## 8. Summary of Results
- Accuracy: ±0.5–1 cm convergence for static / slow rolling ball.
- Robustness: reduced oscillations vs P-only; improved stability with DC motors.
- Remaining bottleneck: vision latency & lack of range estimation.

## 9. Limitations & Gaps
- No distance estimation (only lateral angle) → cannot optimize approach speed profile.
- No explicit perception→actuation latency compensation.
- Vision sensitivity to lighting / logos.
- No long-duration energy / thermal model yet.

## 10. Priority Improvements
1. Short-baseline stereo or depth module.
2. Predictive filtering (EKF vision+IMU) + latency compensation.
3. Lightweight CNN detector (color + texture) for robustness.
4. Gain scheduling vs relative ball approach velocity.
5. Power budget + voltage feedforward.

## 11. Code Organization (This Repository)
- `firmware/prototype1`: minimal PI guidance (servos).
- `firmware/prototype2`: PI wheel speed loop skeleton + guidance interface.
- `modeling`: simulation & identification scripts.
- `tools`: log parser.
- `docs/future_work.md`: roadmap.

## 12. Portfolio Highlights
- Data‑driven transition (asymmetry / saturation) justified hardware pivot.
- Experimental identification + tuning with validated margins.
- Layered architecture (guidance vs actuation) improves portability.
- Bottleneck migration to perception indicates system maturation.

## 13. Figures
Original figures (schematics, trajectories) can be re‑added under `docs/figures/` (omitted here to keep the repo lightweight). Add as needed for richer publication.

---
Condensed technical reference for a public repository.
