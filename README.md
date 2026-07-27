# Traffic Light Controller Using Interrupts

## Project Overview

This project implements a real-time traffic light controller using an Arduino Uno. The system follows the **Sense–Think–Act** model and uses a **Finite State Machine (FSM)** to manage the traffic light sequence.

---

## Features

- Normal traffic light sequence
- Vehicle request button extends the green light duration
- Pedestrian crossing mode with pedestrian LED and buzzer
- Emergency mode with highest priority
- Startup self-test
- Serial Monitor output
- Timer1 interrupt
- Pin Change Interrupts

---

## Hardware Components

- Arduino Uno
- 6 Traffic LEDs
- 1 Pedestrian LED
- 3 Push Buttons
- Active Buzzer
- Breadboard
- Resistors
- Jumper Wires

---

## Pin Configuration

| Component | Pin |
|-----------|-----|
| North Green LED | D2 |
| North Yellow LED | D3 |
| North Red LED | D4 |
| East Green LED | D5 |
| East Yellow LED | D6 |
| East Red LED | D7 |
| Pedestrian LED | D8 |
| Vehicle Button | D9 |
| Emergency Button | D10 |
| Pedestrian Button | D11 |
| Buzzer | D12 |

---

## Author

**Name:** Sourav

**Unit:** SIT315 – Concurrent and Distributed Programming
