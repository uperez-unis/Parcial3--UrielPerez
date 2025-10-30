# Máquina Llenadora de Bebidas (STM32 Nucleo L053R8)

## Descripción breve
Proyecto de una máquina llenadora de bebidas con STM32L053R8, programada en C usando timers, interrupciones y USART.

## Características principales
- Control por timers e interrupciones (sin delays bloqueantes).
- Selección de bebida con keypad 4×4.
- LCD 16×2 para estados y mensajes.
- Display 7 segmentos (4 dígitos) para tiempo (MM:SS).
- Stepper para seleccion de bebida.
- Electroválvulas por relevador.
- Buzzer indicador que esta llenando
- LEDs de estado.
- Maqueta en acrílico diseñada en Inkscape.

## Hardware utilizado (pines y utilidad)

| Módulo | Pines MCU | Utilidad / Descripción |
|---|---|---|
| LCD 16×2 (4-bit) | RS=PA0, E=PA1, D4=PA8, D5=PA10, D6=PA5, D7=PA6 | Mensajes de estado |
| Keypad 4×4 | Filas PB8..PB11 (IN PU), Columnas PB12..PB15 (OUT) | Entrada de A/B/C/D |
| 7-segmentos 4 dígitos | Segmentos PB0..PB7, Dígitos PC5/PC6/PC8/PC9 | Tiempo MM:SS |
| Stepper + driver | PC0..PC3 | Posicionar a bebida A/B y home |
| Electroválvulas (rele) | PC4 (VAL1 A), PC7 (VAL2 B) | Apertura/cierre de flujo |
| LEDs estado | PA11 (Bebida seleccionada), PA12 (Llenando), PA15 (Finalizado) | Indicadores |
| Buzzer | PA7 | Sonido durante llenado |
| USART2 | PA2(TX), PA3(RX) AF4 | Logs 9600 baud |
| Alimentación | 5V lógica, 9–12V válvulas (GND común) | Potencia y control |

## Diseño de la maqueta (Inkscape)
- Piezas en acrílico diseñadas en inskape.
- Archivos vectoriales (.svg) con capas para corte.

## Lógica de funcionamiento (resumen)
1. **Home**: seleccionar A o B en keypad.  
2. **Selección**: mover stepper a posición de bebida y Led de estado de bebida seleccionada encendido.  
3. **Inicio (C)**: reset de tiempo y apertura de válvula seleccionada.  
4. **Llenado**: conteo MM:SS en 7-seg, buzzer y LED llenando activos.  
5. **Finalizar (D)**: cerrar válvula, volver a home y mostrar finalizado en LCD y led.

## Timers e interrupciones

| Timer/IRQ | Frecuencia | Función | Descripción breve |
|---|---|---|---|
| SysTick | 1 kHz | LCD no bloqueante / debounce | Manejo de nibbles y pequeñas esperas |
| TIM21_IRQn | ~STEP_RATE_HZ | Stepper | Avance de fases y posicionamiento |
| TIM22_IRQn | ~2 kHz | 7-seg | Multiplexado de 4 dígitos |
| TIM2_IRQn | 1 Hz | Cronómetro | Incremento de MM:SS durante llenado |
| EXTI4_15_IRQn | Por flanco | Keypad | Lectura de A/B/C/D |
| USART2 | 9600 baud | Logs | Depuración por serie |

## Videos (YouTube)
- [Explicación del código](https://youtu.be/4FcIEX1wMjI)
- [Funcionamiento de la máquina](https://youtube.com/shorts/UG-UgMT1PaI)


