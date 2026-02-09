# Earth Resistivity Tester: ESP32-Based Soil Analysis

A specialized IoT-enabled tool designed to measure soil resistivity for electrical grounding system protection. This device implements the **Schlumberger configuration** to ensure accurate measurements even in non-homogeneous soil conditions.

---

## 📝 Project Overview
Soil resistivity is a vital factor in ensuring the safety and reliability of electrical installations. This project utilizes the **ESP32** microcontroller combined with high-precision sensors to calculate resistivity based on current injection and potential difference measurements.

## Schlumberger Configuration Method
![Schlumberger Configuration Method](Hardware/Schlumberger-Configuration.png)

## ✨ Key Features
* **Schlumberger Method**: Supports flexible electrode spacing for layered soil analysis.
* **Dual Monitoring**: View results via a local **16x2 LCD** and **Serial Monitor**.
* **Integrated Interface**: Features a **3x3 Keypad** for direct distance input (AM and MN).
* **High Precision**: Uses a 16-bit ADC for sensitive voltage measurements.

## 🛠️ Hardware Specifications
* **Microcontroller**: ESP32 (NodeMCU).
* **Current Sensor**: INA-219 (Bi-directional current monitor).
* **Analog-to-Digital Converter**: ADS1115 (16-bit ADC + Programmable Gain Amplifier).
* **User Interface**: 16x2 I2C LCD & 3x3 Membrane Keypad.
* **Power & Control**: 9V Battery (Current source) and a Relay module for measurement control.

### Hardware Connection (Pinout)
| Component | ESP32 Pin | Interface |
| :--- | :---: | :--- |
| **ADS1115 (ADC)** | SDA (21), SCL (22) | I2C (Address 0x4A) |
| **INA-219 (Current)** | SDA (21), SCL (22) | I2C |
| **LCD 16x2** | SDA (21), SCL (22) | I2C (Address 0x27) |
| **Keypad Rows** | 19, 18, 5, 17 | Digital Input |
| **Keypad Columns** | 16, 4, 15 | Digital Input |
| **Relay Module** | (Specified in code) | Digital Output |

## Schematic Diagram
![Earth Resistivity Tester Schematic](Hardware/Schematic-Diagram.png)

## 📊 Scientific Calculation
Resistivity ($\rho_A$) is calculated using the Schlumberger formula:

$$p_{A}=\frac{V}{I}\pi\frac{b(b+a)}{a}$$

Where:
* $V$ = Measured potential difference.
* $I$ = Injected current.
* $b$ = Distance from current electrode to potential electrode.
* $a$ = Distance between potential electrodes.

## 🚀 How to Use
1. **Set Up Electrodes**: Position the four electrodes in the soil according to the Schlumberger configuration.
2. **Input Distances**: Use the keypad to enter the distances between electrodes (MN and AM).
3. **Start Measurement**: Press the '#' key to trigger the measurement cycle.
4. **Read Data**: The device will inject current, read sensors, and display the calculated **Ohm-m** value on the LCD.

