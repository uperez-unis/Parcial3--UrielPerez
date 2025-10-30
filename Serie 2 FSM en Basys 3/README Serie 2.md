# Máquina Expendedora + Reloj (Basys 3 · SystemVerilog)

## Descripción general
Proyecto en **Vivado (AMD/Xilinx)** para la **Basys 3 (XC7A35T-1CPG236C)** que integra:
- **Máquina expendedora** con dos FSM (Moore para crédito y Mealy para producto).
- **Visualización en 7 segmentos** del saldo (0..3) o **reloj HH:MM**.
- **Conmutación de vistas** por botones (**BTNU** = reloj, **BTNC** = máquina).
- **Control por switches**: billete, aceptar y código de producto.

---

## Funcionalidades principales
- **Ingreso de “billetes”**: mientras **SW0 = 1** y **SW1 = 0**, en cada flanco del reloj lento (**clk2**) el saldo **incrementa** `1 → 2 → 3` (saturado en 3).
- **Selección de producto** por **código (SW3..SW2)**:  
  `01 → precio 5`, `10 → precio 10`, `11 → precio 15`, `00 → sin selección`.
- **Confirmación (MEALY)**: si el **saldo** alcanza el precio, **autoriza** (LED de confirmado) y **descuenta** el saldo según el precio (1/2/3 pasos).
- **Display**:  
  - **Modo máquina**: muestra el **saldo** (0..3) en el dígito derecho.  
  - **Modo reloj** (**BTNU**): muestra **HH:MM**. **BTNC** regresa a máquina.
- **LEDs**:  
  - **LD2..LD3** = `P[1:0]` (precio seleccionado)  
  - **LD4** = `Confirmado`

---

## El reloj (HH:MM)
- Divisor de tiempo que genera un “tick” y avanza **segundos → minutos → horas** (formato **24h**: 00:00 → 23:59 → 00:00).
- El reloj solo afecta a la **visualización** cuando estás en **modo reloj** (BTNU).

---

## Códigos del proyecto

| Módulo / Archivo        | Rol en el sistema                                                                 | Puertos clave (resumen)                                                                 |
|-------------------------|------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
| `MAQUINA_EXPENDEDORA.sv`| **Wrapper/top**: integra Moore, Mealy, `clk_psc`, `display_7segments` y `clock`.  | `clk, reset, sw[3:0], BTNU, BTNC, P[1:0], Confimado, enabled[3:0], ag[6:0]`            |
| `Moore.sv`              | **FSM de crédito** (0..3). Con reloj lento `clk2` sube 1 por flanco si `B && ~A`. | `clk2, reset, A(SW1), B(SW0), C[1:0]`                                                  |
| `FSM_Mealy.sv`          | **FSM de producto**: valida saldo vs. código y saca `P` (precio) y `A` (confirmar).| `clk2, reset, B(saldo 0..3), C(código 2b), P[1:0], A`                                  |
| `display_7segments.sv`  | **Driver 7-seg** con multiplexado (~1ms por dígito) y tabla de segmentos (activos en bajo). | `clk, d3..d0, enabled[3:0], ag[6:0]`                                     |
| `clock.sv` / `Reloj.sv` | **Reloj HH:MM** (24h).                                                           | `clk, reset, hora_d, hora_u, min_d, min_u`                                             |
| `clk_psc.sv`            | **Prescaler**: genera `clk2` a partir de 100 MHz (p. ej. `myreg[28]`).            | `clk, clk_scaled`                                                                       |

> **Nota**: `clk2` controla el ritmo de **incremento del saldo** y el **momento de la confirmación/descuento**. Ajusta el bit en `clk_psc` (`myreg[26]`, `myreg[28]`, etc.) para hacer más **rápido/lento** el conteo.

---

## Funcionamiento de la FSM en la Basys 3

### Mapeo de IO
- **Switches**  
  - `SW0` → **B** (billete)  
  - `SW1` → **A** (aceptar)  
  - `SW3..SW2` → **C[1:0]** (código de producto)
- **Botones**  
  - `BTNU` → **ver reloj** (cambia vista a HH:MM)  
  - `BTNC` → **volver a máquina** (vista de saldo)
- **LEDs**  
  - `LD2..LD3` ← **P[1:0]** (precio)  
  - `LD4` ← **Confirmado**
- **Display 7-seg (anodos activos en bajo)**  
  - **Modo máquina**: saldo (0..3) en `d0`  
  - **Modo reloj**: `d3 d2 d1 d0 = H_d H_u M_d M_u`

### Reglas de la Moore (crédito)
- **Incremento**: si `SW0=1` y `SW1=0`, en **cada** flanco de **`clk2`** el estado pasa `S0→S1→S2→S3` (saturado en `S3`).  
- **Reset a cero** (opcional en tu diseño original): `~B && A` lleva a `S0`.

### Reglas de la Mealy (producto)
- **Entrada**: `B` recibe el **saldo** (0..3), `C` el **código**.  
- **Salida**: `P` (precio: 01,10,11) y `A` (**Confirmado**).  
- **Descuento**: en el **mismo flanco** de `clk2` donde `A=1`, se **resta** del saldo:  
  `01→−1`, `10→−2`, `11→−3` (si alcanza).

---

## Cómo correrlo (Vivado)
1. Crear proyecto RTL, **target**: *Basys 3 (XC7A35T-1CPG236C)*.  
2. Agregar fuentes: `MAQUINA_EXPENDEDORA.sv`, `Moore.sv`, `FSM_Mealy.sv`, `display_7segments.sv`, `clock.sv` (o `Reloj.sv`), `clk_psc.sv`.  
3. Agregar constraints `.xdc` con estos puertos del top:  
   `clk, reset, sw[3:0], BTNU, BTNC, P[1:0], Confimado, enabled[3:0], ag[6:0]`.  
4. **Set Top**: `MAQUINA_EXPENDEDORA`.  
5. Synth → Impl → Generate Bitstream → Program.

---

## Pruebas rápidas
- **Ingreso**: subir `SW0` (con `SW1=0`) y observar `0→1→2→3` en el display (ritmo de `clk2`).  
- **Selección**: poner `C=01/10/11`.  
- **Compra**: subir `SW1` (Aceptar). Si hay saldo suficiente, se enciende **LD4** y el saldo **disminuye** 1/2/3.  
- **Vista reloj**: `BTNU` → muestra **HH:MM**; `BTNC` → vuelve a máquina.

---

## Videos (YouTube)
- [Funcionamiento de la Basys 3](https://youtu.be/72ZD_XTAQ70)  
- [Explicación del código](https://youtu.be/H9Tlh2s3Ors)  
- [Proceso para subir el bitstream](https://youtu.be/ZLRo7jF3R0c)

---

## Notas útiles
- **Timing del conteo**: con `clk_psc` usando `myreg[28]` y `clk=100 MHz`, `clk2 ≈ 0.186 Hz` (≈ 5.37 s por ciclo). Ajusta el bit para cambiar la velocidad perceptible.  
- **7-segmentos activos en bajo**: la tabla de segmentos ya está invertida en `display_7segments.sv`.  
- Si el display “parpadea” muy lento/rápido, ajusta el **conteo interno** de `display_7segments` o el **`clk2`**.

---

