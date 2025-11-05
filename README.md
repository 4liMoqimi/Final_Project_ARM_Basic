# 🔷 STM32F407 Final Project — RGB, LCD, ADC & GSM SMS Control

This project is a complete embedded system based on **STM32F407**, combining ADC sensing, RGB control, LCD monitoring, and GSM communication through SMS.

---

## ⚙️ Features

- 🧠 **Dual Mode System**
  - **Auto Mode:** GSM SMS commands control the RGB LED and display info on LCD.  
  - **Manual Mode:** Potentiometer controls RGB LED directly.

- 📱 **GSM (SIM800L) Integration**
  - Receives commands via SMS:
    - `RGB <R> <G> <B>` → Set LED color  
    - `TEMP` → Send current temperature via SMS  
    - Invalid messages return: `Invalid command: <message>`
  - Sends startup message:  
    ```
    STM32: Microcontroller is Alive
    ```

- 🌈 **RGB LED Control**
  - Smooth PWM-based color blending using **TIM1** channels 2, 3, and 4.  
  - Direct and auto modes supported.

- 🌡️ **Temperature Sensor (ADC)**
  - Reads analog input, converts to °C, and displays/sends data.

- 🖥️ **LCD 16x2 Display**
  - Real-time display of system state:
    - `Auto Mode Enabled`
    - `Manual Mode`
    - `Temp sent`
    - `LCD Cleared`

- 🔘 **Button Control (Debounced via HAL_GetTick)**
  - Button 1 → Toggle Auto/Manual mode  
  - Button 2 → Clear LCD and pause system until Auto key pressed again  

- 🧩 **UART Circular Buffer**
  - Non-blocking DMA UART driver for stable GSM data flow.  
  - Separate buffers for TX/RX across UART1 and UART3.

---

## 🧠 System Overview

| Component | Function |
|------------|-----------|
| STM32F407 | Main controller |
| GSM SIM800L | SMS communication |
| LM35 | Temperature sensor |
| LCD 16x2 | Display output |
| RGB LED | PWM color display |
| Potentiometer | Manual color control |
| Buttons | Mode toggle + LCD clear |

---

## 🧩 Command List (SMS)

| Command | Description | Response |
|----------|--------------|-----------|
| `RGB R G B` | Set RGB LED color (0–255 each) | Displays color on LED |
| `TEMP` | Request current temperature | Sends °C value via SMS |
| *(anything else)* | Invalid command | `Invalid command: <text>` |

---

## 🧰 Technologies Used

- STM32 HAL (CubeIDE)
- PWM via TIM1
- ADC (LM35)
- UART with DMA + Circular Buffer
- GSM SIM800L AT Commands
- LCD16x2 Custom Driver
- Debounced GPIO Input (HAL_GetTick)

---

## 🧱 Folder Structure

```
📂 STM32F407_Final_Project
├── Core/
│   ├── Inc/
│   ├── Src/
├── Drivers/
├── GSM/
├── LCD16X2/
├── Util/
├── CircularBuffer/
└── README.md
```

---

## 🚀 Quick Start

1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/STM32F407_Final_Project.git
   ```
2. Open with **STM32CubeIDE**.
3. Connect **STM32F407 + SIM800L + LCD + RGB LED + LM35**.
4. Flash the firmware and power on.
5. Watch for:
   ```
   STM32: Microcontroller is Alive
   ```
   SMS on your phone.

---

## 🧑‍💻 Author

**Ali Moghimi**  
📅 *November 2025*  
💬 Passionate about embedded systems, hardware control, and low-level firmware design.

---

## 🏁 License

This project is released under the **MIT License** — feel free to use, modify, and share.
