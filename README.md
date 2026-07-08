# ESP32 Wi-Fi Remote Control Car

A high-performance, low-latency mobile robotics platform leveraging an **ESP32** microcontroller to host an active asynchronous Web Control Deck over an independent Wi-Fi Access Point. The system drives dual high-torque **N20 Micro Gear Motors** via a **DRV8833 H-Bridge Driver**, featuring programmatic inductive energy back-EMF protection and dynamic PWM speed calibration.

---

## Hardware Architecture & Connections

### Power Architecture
* **Logic & Radio Rails:** An external buck converter steps down raw battery voltage to a regulated **5.0V** delivered directly to the ESP32 **VIN** pin to satisfy the onboard LDO regulator's dropout voltage during high-power Wi-Fi transmission.
* **Motor Drive Rail:** The **VM** pin on the DRV8833 driver connects directly to the external battery supply matching your N20 nominal voltage (e.g., 6.0V), rated for up to **2.5A** to survive simultaneous peak stall currents.
* **Common Ground:** All negative terminals (Battery, Buck Output, ESP32 GND, and DRV8833 GND) must be tied together to ensure a unified signal reference plane.

### Physical Pin Map Matrix

| DRV8833 Terminal | ESP32 GPIO Pin | Channel Purpose | Technical State |
| :--- | :--- | :--- | :--- |
| **IN1** | **GPIO 27** | Left Motor Direction A | PWM Output (8-bit) |
| **IN2** | **GPIO 26** | Left Motor Direction B | PWM Output (8-bit) |
| **IN3** | **GPIO 33** | Right Motor Direction A | PWM Output (8-bit) |
| **IN4** | **GPIO 25** | Right Motor Direction B | PWM Output (8-bit) |
| **VCC / VM** | *External V+* | Motor Main Power Rail | 2.7V - 10.8V Input |
| **GND** | **GND** | Common Ground Plane | Shared System Reference |
| **EEP** | **3.3V** | Driver Hardware Wake | Pulled Logic HIGH |
| **OUT1 / OUT2** | *Motor Terminals* | Left Wheel Actuator | N20 Phase Outputs |
| **OUT3 / OUT4** | *Motor Terminals* | Right Wheel Actuator | N20 Phase Outputs |

---

## Firmware Highlights & Features

### 1. Advanced Back-EMF Suppression Engine
Standard H-bridge implementations are prone to failures caused by sudden direction flips or abrupt halts due to inductive current spikes from the DC motor windings. This firmware intercepts motor state transitions using software tracking hooks (`lastSpeedA`/`lastSpeedB`). If a phase inversion or sharp termination is detected, it forces an absolute hardware coast state (`stopAll()`) for `60ms`, letting the magnetic field safely collapse through the driver’s internal flyback protection diodes.

### 2. Native Mobile WebKit Touch Handling
The hosted Control Deck bypasses standard desktop click triggers in favor of asynchronous touch hooks (`touchstart`/`touchend`). Combined with `fetch()` API calls, it prevents mobile browser UI thread locking, eliminating the typical 300ms click delay. The inclusion of `pointerleave` handles edge cases where the controller finger exits a button bounding box, acting as a failsafe to immediately kill motor power.

---

## Quick Start Deployment

### Prerequisites
* **Arduino IDE** (v2.x or later)
* **ESP32 Board Manager Core v3.x** installed (Note: This codebase leverages the upgraded `ledcAttach` API specific to v3.x architectures).

### Firmware Verification & Setup
1. Clone this repository or open the sketch in your IDE.
2. Update the `ssid` and `password` variables in the code if custom network credentials are required.
3. Flash the code to your ESP32. *(If the bootloader fails to initialize during download, hold the onboard physical **BOOT** button when the console displays `Connecting........`)*.
4. Open the Serial Monitor at **115200 Baud** to verify initialization.

### UI Operation
1. Connect your control device (phone/tablet) to the Wi-Fi network broadcasting as **`ESP32-RC-Car`**.
2. Launch a mobile browser window and navigate directly to: `192.168.4.1`
3. Use the range slider to set your target PWM duty cycle ceiling (scaled between `90` and `255` to prevent motor deadband stalls), then hold down the directional buttons to drive.
