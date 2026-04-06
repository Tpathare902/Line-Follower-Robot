# 🤖 Line Follower Robot

A PID-controlled line following robot using a 7-sensor QTR analog array.

## 🛠️ Hardware
- Arduino Uno/Nano
- QTR-7A Analog Sensor Array
- TB6612FNG Motor Driver
- DC Gear Motors (x2)
- Push Button (Start)

## 📚 Libraries Required
- `QTRSensors` — Install via Arduino Library Manager

## ⚙️ How It Works
- 400-sample auto-calibration on boot
- PID correction at ~500Hz
- Pattern-based left/right turn detection
- Lost-line recovery using last-seen-side memory
- Press button to start after calibration

## 📐 PID Gains
| Gain | Value  |
|------|--------|
| Kp   | 0.0061 |
| Ki   | 0.000  |
| Kd   | 0.09   |
