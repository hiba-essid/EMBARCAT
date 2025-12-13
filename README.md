# 📟 STM32 ADC with DMA (Circular Mode) – Potentiometer Reading via UART

## 📌 Description

This project demonstrates how to read an **analog signal from a potentiometer** using the **ADC of an STM32F446RE (Nucleo board)** with **DMA in circular mode**, and send the results to a PC through **UART (USART2)**.
The output is displayed using **PuTTY** (or any serial terminal).

The ADC value is continuously updated by DMA, converted into voltage and percentage, and printed every 500 ms.

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
=================================
Lecture ADC avec DMA Circulaire
STM32F446RE Nucleo - Potentiometre sur PA0 (ADC1_IN0)
=================================

ADC: 2048 | Tension: 1.650 V | %: 50.0%
ADC: 3072 | Tension: 2.476 V | %: 75.0%
ADC: 1023 | Tension: 0.825 V | %: 25.0%
```

* **ADC**: raw 12-bit value (0–4095)
* **Tension**: converted voltage (0–3.3 V)
* **%**: position of the potentiometer

---

## 💡 Notes

* DMA continuously updates `adc_value` without CPU intervention
* The variable is declared `volatile` to ensure correct behavior
* LED LD2 toggles every 500 ms to show the system is running

---

## 🚀 Possible Improvements

* Add ADC averaging or filtering
* Use interrupt callbacks instead of polling
* Send data via USB CDC instead of UART
* Add multiple ADC channels

---

## 🧑‍💻 Author

Developed for learning and demonstration purposes using STM32 HAL.

---

## 📜 License

Free to use for educational and personal projects.
