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
<img width="963" height="1280" alt="image" src="https://github.com/user-attachments/assets/ad218dff-e6c4-4158-8f65-ac21a1c53e48" />
<img width="1280" height="963" alt="image" src="https://github.com/user-attachments/assets/e25da6c1-1f36-44d4-b199-bc7fca471fdd" />


https://github.com/user-attachments/assets/56612b87-acda-4e51-af72-2b1d24df8d32

