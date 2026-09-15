# Autonomous Drone Mission System (Pi 5 + APM 2.8)

This project contains a fully autonomous vision-based drone mission system. It utilizes a Raspberry Pi 5 companion computer for image processing (OpenCV) and an APM 2.8 flight controller for navigation (DroneKit).

## Features
- Multi-threaded video stream processing on Raspberry Pi 5.
- Contour-based concentric circle target detection with lighting invariance (CLAHE) and HSV filtering.
- DroneKit automated mission: Takeoff -> Lawnmower Search -> Visual Servoing Centering -> Land -> 3s Hold.
- Maximum 3 consecutive landings, followed by Return to Launch (RTL).
- Safe telemetry rate (5Hz) to prevent APM 2.8 CPU overload.

## Hardware Setup

### Raspberry Pi 5 to APM 2.8 Wiring
The APM 2.8 communicates over a serial port (UART). You will need to connect the Raspberry Pi 5's GPIO UART pins to the APM's Telemetry port.

| Raspberry Pi 5 | APM 2.8 Telemetry Port |
| :--- | :--- |
| GND (Pin 6) | GND (Black) |
| TXD (Pin 8 - GPIO 14) | RX (Yellow) |
| RXD (Pin 10 - GPIO 15) | TX (Green) |
| 5V (Pin 2) | **DO NOT CONNECT** (Power Pi separately) |

*Note: The Raspberry Pi uses 3.3V logic, while the APM 2.8 is 5V. In many cases, the 3.3V TX from the Pi is enough to register as HIGH on the APM, and the 5V TX from the APM won't fry the Pi's RX pin (though a logic level shifter is highly recommended for safety).*

### Enable Serial on Pi 5
1. Run `sudo raspi-config`
2. Go to **Interface Options** -> **Serial Port**
3. Select **No** to "Would you like a login shell to be accessible over serial?"
4. Select **Yes** to "Would you like the serial port hardware to be enabled?"
5. Reboot the Pi. The serial port is now available at `/dev/serial0`.

## Software Setup

### Prerequisites
Install the required Python packages on your Raspberry Pi:
```bash
pip install opencv-python numpy dronekit pymavlink
```

## Running Software-In-The-Loop (SITL)

Before testing on actual hardware, you should validate the logic in a simulated environment.

1. **Install SITL on your PC:**
   ```bash
   pip install dronekit-sitl
   ```
2. **Start the simulator:**
   ```bash
   dronekit-sitl copter
   ```
   *Note: This will start a simulated ArduCopter instance listening on TCP port 5760.*
3. **Change connection string:**
   In `drone_mission.py`, ensure the connection string is set for SITL:
   ```python
   connection_string = 'tcp:127.0.0.1:5760'
   ```
4. **Run the script:**
   ```bash
   python drone_mission.py
   ```
   You can point your PC's webcam at a picture of the target.

## Tuning Guide

1. **Vision Tuning**:
   - The default HSV bounds in `detect_target()` are very broad (`[0, 50, 50]` to `[180, 255, 255]`). You must measure the HSV values of your specific target in its operational environment to narrow this down for reliable detection.
2. **PID Tuning**:
   - `Kp_xy` and `Kd_xy` define how aggressively the drone responds to visual offsets. If the drone overshoots the target, decrease `Kp_xy`. If it reacts too slowly, increase it.
