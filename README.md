# Resolución de Parcial 3 — Arquitectura de Computadoras y Microcontroladores

Este repositorio contiene la **resolución del Parcial 3** dividida en dos series:

- **Serie 1:** Máquina llenadora de bebidas en **STM32 Nucleo L053R8** (C, timers, interrupciones, USART).
- **Serie 2:** **FSM** de máquina dispensadora de bebidas + **reloj (clock HH:MM)** en **Basys-3** (SystemVerilog).

---

## Contenidos

- **Nucleo L053R8** — Código fuente, pinout, timers/IRQ, y README específico de la **máquina llenadora**.
- **BASYS 3** — Código SystemVerilog, FSMs, mapa de pines XDC y README del **dispensador + clock**.


---

## Serie 1 — Máquina Llenadora de Bebidas (STM32 Nucleo L053R8)

**Resumen:** Control de dos bebidas (A/B) con **keypad 4×4**, **LCD 16×2**, **7-segmentos 4 dígitos (MM:SS)**, **stepper** para orientar el surtidor y **electroválvulas** por relevador. Implementación **sin delays bloqueantes** (timers/IRQ + FSM para LCD). Logs por **USART2**.


**Desarrollo:**
1. Abrir en **STM32CubeIDE** (target: Nucleo L053R8, HSI16).
2. Verificar pinout en README de la serie (PA/PC/PB según tabla).
3. Conectar **driver stepper** (PC0..PC3), **relevadores** (PC4/PC7), **LCD**, **keypad** y **7-seg**.
4. Alimentación: 5 V lógica, 12 V válvulas (**GND común**).
5. Monitorear **USART2** a **9600 baud**.

---

## Serie 2 — FSM Dispensadora + Clock (Basys-3)

**Resumen:** Diseño en **SystemVerilog** de:
- **FSM Máquina dispensadora** (Moore: conteo de dinero, Mealy: Estructura de productos).
- **Reloj HH:MM** en 4 dígitos 7-segmentos (multiplexado), simulación de reloj 24h.


**Desarrollo:**
1. Abrir proyecto en **Vivado** (Basys-3, XC7A35T-1CPG236C).
2. Vincular archivo **XDC** con el mapa de pines (BTN/CLK/SW/7-seg).
3. Sintetizar, implementar y programar la FPGA.
4. Probar **FSM** (selección de bebida) y **clock** (incremento minutos/horas).

---

## Requisitos

- **Serie 1 (STM32):** STM32CubeIDE, placa **Nucleo L053R8**, módulos LCD 16×2 (4-bit), keypad 4×4, 7-seg 4 dígitos, driver stepper (ULN2003/L293/DRV), relevador 1 canal, válvulas 12 V.
- **Serie 2 (Basys-3):** Vivado AMD, placa **Basys-3**.

---




