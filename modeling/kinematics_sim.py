"""Differential drive kinematic simulation for lateral centering.
Simplified: camera provides lateral pixel error ~ proportional to heading error.
"""
import numpy as np
try:
    import matplotlib.pyplot as plt  # optionnel
    _HAS_MPL = True
except Exception:
    _HAS_MPL = False

Kp = 1.2  # proportional gain
Ki = 0.5  # integral gain (set 0 for pure P)
Ts = 0.02
T  = 4.0

# State: lateral offset y, heading theta; forward speed v0 constant
v0 = 0.25  # m/s
fov_scale = 320/0.6  # pixel per rad (example)

steps = int(T/Ts)
err_i = 0.0

ys = []
ths = []
urs = []
errs = []

y = 0.15  # initial lateral offset (m)
th = 0.0

for k in range(steps):
    # assume small-angle pixel error ~ (y/L_detect)*fov_scale; simplified mapping
    pixel_err = (y/0.6)*320
    err = pixel_err
    err_i += err*Ts
    u = Kp*err + Ki*err_i
    # differential steering -> heading rate approx u scaled
    th += (-u/ fov_scale)*Ts
    # Lateral motion update (small-angle): y += v * sin(th) * Ts
    y += v0*np.sin(th)*Ts
    ys.append(y); ths.append(th); urs.append(u); errs.append(err)

if _HAS_MPL:
    fig,ax=plt.subplots(2,1,sharex=True)
    ax[0].plot(np.arange(steps)*Ts, ys)
    ax[0].set_ylabel('y (m)')
    ax[1].plot(np.arange(steps)*Ts, errs, label='pixel_err')
    ax[1].plot(np.arange(steps)*Ts, urs, label='u')
    ax[1].legend()
    ax[1].set_xlabel('time (s)')
    plt.tight_layout()
    plt.show()
else:
    # Fallback texte
    print("Final lateral offset y=%.4f m, heading=%.4f rad" % (ys[-1], ths[-1]))
