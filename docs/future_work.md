# Future Work Roadmap

## Perception
- Add depth (stereo or lightweight ToF) to estimate range for predictive interception.
- Robust color + shape or tiny CNN to handle lighting variation & printed logos.
- Timestamped pipeline and explicit perception→actuation latency compensation.

## Control
- Derivative (filtered lead) on outer guidance for faster lateral transients.
- Gain scheduling vs estimated approach velocity or confidence.
- Voltage feedforward / battery SoC compensation for consistent dynamics.

## Fusion & State Estimation
- IMU + encoder EKF for smoother dead reckoning between vision frames.
- Ball motion classifier (static / rolling / bouncing) → adaptive pursuit profiles.

## Reliability
- Vision timeout watchdog → safe slowdown / stop fallback.
- Structured logging (CSV + JSON metadata) enabling automated regression plots.

## Energy & Endurance
- Motor + driver thermal model and safe operating envelope.
- Power budget + duty cycle optimization.

## Intelligence
- Predictive bounce modeling (inelastic impacts) for pre‑positioning.
- Multi‑ball prioritization (closest time‑to‑intercept heuristic).
