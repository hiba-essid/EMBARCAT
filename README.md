# 📟 STM32 ADC with DMA (Circular Mode) – Potentiometer Reading with Reactive LED

## 📌 Description

This project demonstrates how to read an **analog signal from a potentiometer** using the **ADC of an STM32F446RE (Nucleo board)** with **DMA in circular mode**, and send the results to a PC through **UART (USART2)**.
The output is displayed using **PuTTY** (or any serial terminal).

**New Feature:** The **LED LD2 blink speed changes dynamically** based on the measured voltage level, providing visual feedback of the potentiometer position.

The ADC uses a **4-sample circular buffer with averaging** to smooth readings. Values are displayed every **2 seconds** and the LED behavior updates every **100 ms**.

---

## 🧰 Hardware Requirements

* STM32F446RE **Nucleo board**
* Potentiometer (e.g. 10kΩ)
* Breadboard + jumper wires
* USB cable (for power, programming, and UART via ST-Link)

---

## 🔌 Hardware Connections

### Potentiometer

| Potentiometer Pin | STM32 Pin      |
| ----------------- | -------------- |
| One side          | 3.3V           |
| Other side        | GND            |
| Middle (wiper)    | PA0 (ADC1_IN0) |

> ⚠️ **Important:** Do not exceed 3.3V on PA0.

---

## 💻 Software Requirements

* **STM32CubeIDE**
* **STM32 HAL drivers**
* **PuTTY** (or Tera Term / minicom)
* ARM GCC toolchain (included with CubeIDE)

---

## ⚙️ Project Configuration

### ADC

* ADC: **ADC1**
* Channel: **ADC_CHANNEL_0 (PA0)**
* Resolution: **12-bit**
* Mode: **Continuous conversion**
* Sampling time: **480 cycles**
* DMA: **Enabled (circular mode)**
* Buffer: **4 samples** (for averaging)
* Sampling interval: **100 ms**

### DMA

* Controller: **DMA2**
* Stream: **DMA2_Stream0**
* Channel: **Channel 0**
* Mode: **Circular**
* Data alignment: **Half-word (16-bit)**

### UART

* Interface: **USART2**
* Baud rate: **115200**
* Data bits: **8**
* Parity: **None**
* Stop bits: **1**

---

## ▶️ How to Run the Project

1. Open the project in **STM32CubeIDE**
2. Build the project
3. Flash the firmware to the Nucleo board
4. Open **PuTTY**

   * Connection type: **Serial**
   * Serial line: COMx (check Device Manager)
   * Speed (baud): **115200**
5. Press **Open**

---

## 📊 Output Example (PuTTY)

```
ADC Demo - PA0
3.3V ref | 12-bit
Display: every 2s | LED: speed = voltage

ADC: 2048 | 1.650 V | 50.0% | LED: 1000ms
ADC: 3072 | 2.476 V | 75.0% | LED: 250ms
ADC: 1023 | 0.825 V | 25.0% | LED: 0ms (OFF)
```

* **ADC**: raw 12-bit value (0–4095)
* **V**: converted voltage (0–3.3 V)
* **%**: position of the potentiometer
* **LED**: current blink interval in milliseconds

---

## 💡 LED Behavior by Voltage

| Voltage (V)   | LED Behavior           | Blink Interval |
|---------------|------------------------|----------------|
| < 1.0         | OFF                    | -              |
| 1.0 – 1.5     | Very slow blink        | 2000 ms        |
| 1.5 – 2.0     | Slow blink             | 1000 ms        |
| 2.0 – 2.25    | Medium blink           | 500 ms         |
| 2.25 – 2.5    | Fast blink             | 250 ms         |
| > 2.5         | Very fast blink        | 100 ms         |

---

## 🔧 Notes

* DMA continuously updates the **4-sample circular buffer** without CPU intervention
* **Averaging** is applied to smooth ADC readings and avoid flickering
* Variables are declared `volatile` to ensure correct behavior
* **LED blink speed** reflects the measured voltage in real-time
* **Low power mode** (`__WFI()`) is used in the main loop to reduce power consumption
* Display interval: **2 seconds** | ADC sampling: **100 ms**

---

## ✅ Implemented Improvements

* ✔️ ADC averaging with 4-sample buffer
* ✔️ Voltage-based LED behavior (reactive feedback)
* ✔️ Low power mode with `__WFI()`
* ✔️ Optimized UART output format

---

## 🚀 Possible Future Improvements

* Add more ADC channels for multiple sensors
* Use interrupt callbacks instead of polling
* Send data via USB CDC instead of UART
* Add hysteresis to prevent LED flickering at threshold boundaries

---

## 🧑‍💻 Author

Developed for learning and demonstration purposes using STM32 HAL.

---

## 📜 License

Free to use for educational and personal projects.
