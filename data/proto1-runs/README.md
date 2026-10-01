# Prototype 1 run logs

Wheel commands logged by prototype 1 (servo drive) while it centred on a table-tennis ball.

| Folder | Ball | Controller |
|---|---|---|
| `static-ball/P/` | static | proportional |
| `static-ball/PI/` | static | proportional-integral |
| `moving-ball/P/` | moving | proportional |
| `moving-ball/PI/` | moving | proportional-integral |

Each file is one run. One row per guidance update, two semicolon-separated columns, no header:

```
left;right
```

Each value is a wheel command measured from the servo's stop position: 0 is stopped, 90 is full speed. This matches the `L=` and `R=` values printed by `firmware/prototype1/servo_guidance_pi.ino`.

With the P controller one wheel often sits at 90, the servo saturation listed in the results table. With the PI controller the two commands stay inside their range and sum to about 90.
