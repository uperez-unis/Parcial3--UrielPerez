#include <stdint.h>
#include "stm32l053xx.h"

/*
        * Keypad 4x4 (acá solo uso la columna PB12 y filas PB8..PB11: A/B/C/D)
        * LCD 16x2 en 4 bits (RS=PA0, E=PA1, D4=PA8, D5=PA10, D6=PA5, D7=PA6)
        * Display 7 segmentos 4 dígitos (PB0..PB7 segmentos, PC5/PC6/PC8/PC9 dígitos)
        * Stepper en PC0..PC3 (seleccionar válvula A o B)
        * Válvulas (relés activos en bajo): VAL1=PC4 (A), VAL2=PC7 (B)
        * LEDs de estado: Listo para llenar =PA11, Llenando =PA12, Finalizado =PA15
        * Buzzer en PA7 (suena mientras está llenando)
*/

//Config del motor stepper

#define STEPS_PER_REV   2048u	// Cantidad aproximada de pasos
#define STEP_RATE_HZ    400u	// Velocidad
#define STEPS_30_DEG    (STEPS_PER_REV/19)   // Angulo de giro aprox 20°

// USART2 para logs
// PA2=TX, PA3=RX con función alternativa AF4. Baud 9600 a 16 MHz.
static void USART2_init(void)
{
    RCC->APB1ENR |= (1u<<17);     // habilito clock de USART2
    RCC->IOPENR  |= (1u<<0);      // habilito clock de GPIOA

    // Pongo PA2 y PA3 en modo Alternate Function (AF4)
    GPIOA->MODER &= ~((3u<<(2*2)) | (3u<<(3*2)));
    GPIOA->MODER |=  ((2u<<(2*2)) | (2u<<(3*2)));
    GPIOA->AFR[0] &= ~((0xFu<<8) | (0xFu<<12));
    GPIOA->AFR[0] |=  ((4u<<8) | (4u<<12));

    USART2->BRR = 0x0683;         // Baud 9600
    USART2->CR1 = (1u<<2) | (1u<<3) | (1u<<0); // RE | TE | UE (recibir, transmitir, habilitar)
}
static void USART2_write(uint8_t ch)
{
    while(!(USART2->ISR & 0x0080))	//espero que el tx este listo
    {

    }
    USART2->TDR = ch;               // mando el char
}

static void USART2_PutstringE(const char *s)
{
    while(*s) USART2_write((uint8_t)*s++);
    USART2_write('\n'); USART2_write('\r');    // salto de línea
}

// LEDs de estado
// Listo para llenar =PA11, llenando =PA12, finalizado =PA15
#define LED1_PORT GPIOA
#define LED2_PORT GPIOA
#define LED3_PORT GPIOA
#define LED1_PIN  11u
#define LED2_PIN  12u
#define LED3_PIN  15u

//Funciones para encender/apagar leds
static inline void LED1_ON (void)
{
	LED1_PORT->ODR |=  (1u<<LED1_PIN);
}

static inline void LED1_OFF(void)
{
	LED1_PORT->ODR &= ~(1u<<LED1_PIN);
}

static inline void LED2_ON (void)
{
	LED2_PORT->ODR |=  (1u<<LED2_PIN);
}

static inline void LED2_OFF(void)
{
	LED2_PORT->ODR &= ~(1u<<LED2_PIN);
}

static inline void LED3_ON (void)
{
	LED3_PORT->ODR |=  (1u<<LED3_PIN);
}

static inline void LED3_OFF(void)
{
	LED3_PORT->ODR &= ~(1u<<LED3_PIN);
}

// Buzzer (PA7) encender/apagar
static inline void BZ_ON (void)
{
	GPIOA->ODR |=  (1u<<7);
}

static inline void BZ_OFF(void)
{
	GPIOA->ODR &= ~(1u<<7);
}

// Válvulas
// A = PC4, B = PC7
static inline void VAL1_ON (void)
{
	GPIOC->ODR &= ~(1u<<4);
}
static inline void VAL1_OFF(void)
{
	GPIOC->ODR |=  (1u<<4);
}
static inline void VAL2_ON (void)
{
	GPIOC->ODR &= ~(1u<<7);
}
static inline void VAL2_OFF(void)
{
	GPIOC->ODR |=  (1u<<7);
}

// Selección actual
typedef enum
{
	SEL_NONE=0,
	SEL_A=1,
	SEL_B=2
} sel_t;

static volatile sel_t sel = SEL_NONE;   // acá guardo si el usuario eligió A, B o nada
static volatile uint8_t val1_on=0, val2_on=0;  // flags para saber si cada válvula está abierta

// Esto es para detectar flancos en las teclas (A/B/C/D) y hacer un debounce simple por IRQ
static volatile uint8_t prev_A=0, prev_B=0, prev_C=0, prev_D=0;

// Stepper en PC0..PC3
// Escribo la fase en PC0..PC3 Avanzo/retrocedo hasta llegar a target_steps.
static inline void coils_write(uint8_t phase) {
    GPIOC->BSRR = (0x0Fu << 16);   // apago PC0-PC3
    switch (phase) {
        case 1:
        	GPIOC->BSRR = (1u<<0);
        	break;
        case 2:
        	GPIOC->BSRR = (1u<<1);
        	break;
        case 3:
        	GPIOC->BSRR = (1u<<2);
        	break;
        case 4:
        	GPIOC->BSRR = (1u<<3);
        	break;
        default:
        	break;
    }
}
static volatile int32_t pos_steps    = 0;  // posición actual
static volatile int32_t target_steps = 0;  // meta de pasos (positiva o negativa)
static volatile uint8_t phase        = 1;  // fase de la secuencia (1..4)

static inline void stepper_start_to_target(void)
{
    if (target_steps != pos_steps)
    	TIM21->CR1 |= TIM_CR1_CEN;  // enciendo el timer si hay trabajo
}

static inline void stepper_stop(void)
{
    TIM21->CR1 &= ~TIM_CR1_CEN;     // apago el timer si llegué
}

void TIM21_IRQHandler(void)
{
    if (TIM21->SR & TIM_SR_UIF)
    {           // cada tick de TIM21 es un paso
        TIM21->SR &= ~TIM_SR_UIF;

        if (pos_steps < target_steps) // tengo que avanzar
        {
            phase = (phase == 4) ? 1 : (phase + 1);
            coils_write(phase);
            pos_steps++;
            if (pos_steps == target_steps)
            {
            	stepper_stop(); USART2_PutstringE("Posicionado");
            }
        } else if (pos_steps > target_steps)
        {
            phase = (phase == 1) ? 4 : (phase - 1);
            coils_write(phase);
            pos_steps--;
            if (pos_steps == target_steps)
            {
            	stepper_stop(); USART2_PutstringE("Posicionado");
            }
        } else
        {
            stepper_stop();  // por si acaso
        }
    }
}

// Display 7 segmentos 4 dígitos (TIM22)
// Segmentos en PB0..PB7 y dígitos en PC5/PC6/PC8/PC9
static const uint8_t seg_lut[10] =
{
    0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
    0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111	// 9
};
static volatile uint8_t disp_digits[4] = {0,0,0,0};

static inline void digits_all_off(void){
    GPIOC->BSRR = (1u<<(5+16)) | (1u<<(6+16)) | (1u<<(8+16)) | (1u<<(9+16)); // apago todos los dígitos
}
static inline void digit_on(uint8_t idx)
{
    // Yo usé este orden: [0]=PC9, [1]=PC8, [2]=PC6, [3]=PC5
    static const uint8_t pins[4] = {9, 8, 6, 5};
    GPIOC->BSRR = (1u << pins[idx]);
}
static inline void seg_write(uint8_t pattern)
{
    GPIOB->BSRR = (0xFFu << 16);      // limpio segmentos
    GPIOB->BSRR = (uint32_t)pattern;  // escribo segmentos del dígito actual
}
static inline void update_digits_from_mmss(uint8_t mm, uint8_t ss)
{
    if (mm > 99) mm = 99; // minutos
    if (ss > 59) ss = 59; // segundos

    disp_digits[0] = (mm / 10) % 10;	// decenas minutos
    disp_digits[1] =  mm % 10;	     	//unidades minutos
    disp_digits[2] = (ss / 10) % 10;	// decenas segundos
    disp_digits[3] =  ss % 10;			// unicades segundos
}

void TIM22_IRQHandler(void)
{
    if (TIM22->SR & TIM_SR_UIF)
    {
        TIM22->SR &= ~TIM_SR_UIF;

        static uint8_t idx = 0;   // voy rotando el dígito activo
        digits_all_off();

        // Esto oculta ceros a la izquierda para que se vea más limpio
        uint8_t blank = 0;
        if (idx==0 && disp_digits[0]==0 && (disp_digits[1]|disp_digits[2]|disp_digits[3])!=0) blank=1;
        if (idx==1 && disp_digits[0]==0 && disp_digits[1]==0 && (disp_digits[2]|disp_digits[3])!=0) blank=1;
        if (idx==2 && disp_digits[0]==0 && disp_digits[1]==0 && disp_digits[2]==0 && disp_digits[3]!=0) blank=1;

        seg_write(blank ? 0x00 : seg_lut[disp_digits[idx]]);
        digit_on(idx);
        idx = (uint8_t)((idx + 1) & 0x03);
    }
}

// Contador de llenado a 1 Hz (TIM2)
// Solo cuenta cuando alguna válvula está abierta
static volatile uint8_t fill_min = 0;  // 0-99
static volatile uint8_t fill_sec = 0;  // 0-59
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;

        if (val1_on || val2_on)
        {
            // sumo 1 segundo
            fill_sec++;
            if (fill_sec >= 60)
            {
                fill_sec = 0;
                if (fill_min < 99) fill_min++;   // tope 99:59
            }
            update_digits_from_mmss(fill_min, fill_sec);
        }
    }
}


// LCD (SysTick a 1ms)
// Uso una cola de bytes + una FSM que “empuja” nibbles a la LCD mientras el SysTick va corriendo.
// Así no bloqueo el CPU y dejo el while(1) libre.
#define LCD_RS_PORT GPIOA
#define LCD_E_PORT  GPIOA
#define LCD_D_PORT  GPIOA
#define LCD_RS_PIN  0u
#define LCD_E_PIN   1u
#define LCD_D4_PIN  8u
#define LCD_D5_PIN  10u
#define LCD_D6_PIN  5u
#define LCD_D7_PIN  6u

static inline void LCD_RS(uint8_t v)
{
    if(v) LCD_RS_PORT->ODR |=  (1u<<LCD_RS_PIN);
    else  LCD_RS_PORT->ODR &= ~(1u<<LCD_RS_PIN);
}
static inline void LCD_E(uint8_t v)
{
    if(v) LCD_E_PORT->ODR |=  (1u<<LCD_E_PIN);
    else  LCD_E_PORT->ODR &= ~(1u<<LCD_E_PIN);
}
static inline void lcd_bus_write_nibble(uint8_t n)
{
    // limpio D4..D7
    uint32_t clr = (1u<<LCD_D4_PIN)|(1u<<LCD_D5_PIN)|(1u<<LCD_D6_PIN)|(1u<<LCD_D7_PIN);
    LCD_D_PORT->BSRR = (clr<<16);
    // seteo según el nibble
    uint32_t set = 0;
    if(n & 0x1) set |= (1u<<LCD_D4_PIN);
    if(n & 0x2) set |= (1u<<LCD_D5_PIN);
    if(n & 0x4) set |= (1u<<LCD_D6_PIN);
    if(n & 0x8) set |= (1u<<LCD_D7_PIN);
    if(set) LCD_D_PORT->BSRR = set;
}

// Estados de la FSM de la LCD
typedef enum {LCD_IDLE, LCD_PUT4_HI, LCD_PULSE_HI, LCD_PUT4_LO, LCD_PULSE_LO} lcd_state_t;
#define LCD_QSIZE 64  // tamaño de la cola

static volatile uint8_t  lcd_q[LCD_QSIZE]; // guardo de a pares
static volatile uint8_t  lcd_q_head=0, lcd_q_tail=0;
static volatile lcd_state_t lcd_state = LCD_IDLE;
static volatile uint8_t  lcd_cur_byte=0, lcd_is_data=0;
static volatile uint16_t lcd_wait_ms=0; // pequeñas esperas entre flancos

// Encolo un byte: is_data=0 (comando), is_data=1 (dato)
static void lcd_enqueue(uint8_t data, uint8_t is_data)
{
    uint8_t next = (uint8_t)((lcd_q_head+2)&(LCD_QSIZE-1));
    if(next == lcd_q_tail) return;         // cola llena
    lcd_q[lcd_q_head] = is_data;
    lcd_q[(lcd_q_head+1)&(LCD_QSIZE-1)] = data;
    lcd_q_head = next;
}
static inline void lcd_cmd_nb (uint8_t cmd){ lcd_enqueue(cmd, 0); }
static inline void lcd_data_nb(uint8_t ch ){ lcd_enqueue(ch, 1); }
static void lcd_print_nb(const char* s){ while(*s) lcd_data_nb((uint8_t)*s++); }

static void lcd_gpio_init(void)
{
    RCC->IOPENR |= (1u<<0); // habilito GPIOA
    // RS, E y D4..D7 como salidas
    GPIOA->MODER &= ~((3u<<(LCD_RS_PIN*2))|(3u<<(LCD_E_PIN*2))|
                      (3u<<(LCD_D4_PIN*2))|(3u<<(LCD_D5_PIN*2))|
                      (3u<<(LCD_D6_PIN*2))|(3u<<(LCD_D7_PIN*2)));
    GPIOA->MODER |=  ((1u<<(LCD_RS_PIN*2))|(1u<<(LCD_E_PIN*2))|
                      (1u<<(LCD_D4_PIN*2))|(1u<<(LCD_D5_PIN*2))|
                      (1u<<(LCD_D6_PIN*2))|(1u<<(LCD_D7_PIN*2)));
    LCD_RS(0); LCD_E(0);
}

static void lcd_init_nb_begin(void)
{
    lcd_cmd_nb(0x33);  // wake-up
    lcd_cmd_nb(0x32);  // paso a 4 bits
    lcd_cmd_nb(0x28);  // 4 bits, 2 líneas, 5x8
    lcd_cmd_nb(0x0C);  // display ON, cursor OFF
    lcd_cmd_nb(0x06);  // auto-incremento
    lcd_cmd_nb(0x01);  // clear
}

// El SysTick (1ms) va despachando nibbles a la LCD con los pulsos E y los tiempos mínimos
void SysTick_Handler(void)
{
    if(lcd_wait_ms){                 // si estoy esperando, solo decremento y me voy
        lcd_wait_ms--;
        return;
    }
    switch(lcd_state)
    {
    case LCD_IDLE:
        if(lcd_q_tail != lcd_q_head){               // si hay algo en la cola, lo agarro
            lcd_is_data = lcd_q[lcd_q_tail];
            lcd_q_tail = (uint8_t)((lcd_q_tail+1)&(LCD_QSIZE-1));
            lcd_cur_byte = lcd_q[lcd_q_tail];
            lcd_q_tail = (uint8_t)((lcd_q_tail+1)&(LCD_QSIZE-1));
            LCD_RS(lcd_is_data);
            lcd_state = LCD_PUT4_HI;
        }
        break;
    case LCD_PUT4_HI:
    {
        uint8_t hi = (lcd_cur_byte>>4)&0x0F;        // nibble alto
        lcd_bus_write_nibble(hi);
        LCD_E(1);                                   // pulso de enable
        lcd_wait_ms = 1;                            // le doy ~1ms alto
        lcd_state = LCD_PULSE_HI;
    } break;
    case LCD_PULSE_HI:
        LCD_E(0);
        lcd_wait_ms = 1;                            // ~1ms entre nibble alto y bajo
        lcd_state = LCD_PUT4_LO;
        break;
    case LCD_PUT4_LO: {
        uint8_t lo = lcd_cur_byte & 0x0F;           // nibble bajo
        lcd_bus_write_nibble(lo);
        LCD_E(1);
        lcd_wait_ms = 1;
        lcd_state = LCD_PULSE_LO;
    } break;
    case LCD_PULSE_LO:
        LCD_E(0);
        // Clear/Home tardan más, acá le doy 2ms extra y ya
        if(!lcd_is_data && (lcd_cur_byte==0x01u || lcd_cur_byte==0x02u)){
            lcd_wait_ms = 2;
        }
        lcd_state = LCD_IDLE;
        break;
    default:
        lcd_state = LCD_IDLE;
        break;
    }
}

// Keypad por EXTI (PB8..PB11 leen A/B/C/D en la columna PB12)
// pongo PB12=0 y las filas PB8..PB11 tienen pull-up.
// Si alguna fila lee 0, significa que esa tecla de esa columna se presionó.
static inline uint32_t read_rows(void)
{
	return (GPIOB->IDR >> 8) & 0x0Fu; // 0 = presionada
}

void EXTI4_15_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1u<<8)|(1u<<9)|(1u<<10)|(1u<<11));
    if (pending)
    {
        // Dejo todas las columnas en 1 y PB12 en 0 para “leer” esa columna
        GPIOB->ODR |=  (0x0Fu << 12);
        GPIOB->ODR &= ~(1u    << 12);

        uint32_t rows = read_rows();
        uint8_t A = ((rows & 0x1u) == 0u);
        uint8_t B = ((rows & 0x2u) == 0u);
        uint8_t C = ((rows & 0x4u) == 0u);
        uint8_t D = ((rows & 0x8u) == 0u);

        // A = Selecciono bebida A y muevo el stepper hacia A
        if (A && !prev_A)
        {
            sel = SEL_A;
            target_steps = -(int32_t)STEPS_30_DEG;    // giro hacia el lado A
            USART2_PutstringE("Bebida A seleccionada");
            stepper_start_to_target();
            LED1_ON(); LED2_OFF(); LED3_OFF(); BZ_OFF();

            // LCD: aviso que quedó lista para llenar A
            lcd_cmd_nb(0x01);
            lcd_cmd_nb(0x80); lcd_print_nb("A: Coca-coca");
            lcd_cmd_nb(0xC0); lcd_print_nb("C: Iniciar");
        }

        // B = Selecciono bebida B y muevo el stepper hacia B
        if (B && !prev_B)
        {
            sel = SEL_B;
            target_steps =  (int32_t)STEPS_30_DEG;    // giro hacia el lado B
            USART2_PutstringE("Bebida B seleccionada");
            stepper_start_to_target();
            LED1_ON(); LED2_OFF(); LED3_OFF(); BZ_OFF();

            // LCD: aviso que quedó lista para llenar B
            lcd_cmd_nb(0x01);
            lcd_cmd_nb(0x80); lcd_print_nb("B: Pepsi");
            lcd_cmd_nb(0xC0); lcd_print_nb("C: Iniciar");
        }

        // C = Inicio de llenado: abro la válvula según la selección y reseteo contador
        if (C && !prev_C)
        {
        	fill_min = 0;
        	fill_sec = 0;
        	update_digits_from_mmss(fill_min, fill_sec);

            if (sel == SEL_A)
            {
                VAL1_ON();  val1_on = 1;
                if (val2_on){ VAL2_OFF(); val2_on = 0; }
                USART2_PutstringE("Llenando bebida A...");
                lcd_cmd_nb(0x01);
				lcd_cmd_nb(0x80); lcd_print_nb("Llenando...");
				lcd_cmd_nb(0xC0); lcd_print_nb("D: cancelar");
				LED1_OFF(); LED2_ON(); LED3_OFF(); BZ_ON();
            } else if (sel == SEL_B)
            {
                VAL2_ON();  val2_on = 1;
                if (val1_on){ VAL1_OFF(); val1_on = 0; }
                USART2_PutstringE("Llenando bebida B...");
                lcd_cmd_nb(0x01);
				lcd_cmd_nb(0x80); lcd_print_nb("Llenando...");
				lcd_cmd_nb(0xC0); lcd_print_nb("D: cancelar");
				LED1_OFF(); LED2_ON(); LED3_OFF(); BZ_ON();
            } else
            {
                USART2_PutstringE("Seleccione bebida primero");
                lcd_cmd_nb(0x01);
				lcd_cmd_nb(0x80); lcd_print_nb("Seleccione");
				lcd_cmd_nb(0xC0); lcd_print_nb("Bebida A/B");
				LED1_OFF(); LED2_OFF(); LED3_OFF(); BZ_ON();
            }

        }

        // D = Finalizo el llenado: cierro válvulas y regreso el brazo a 0°
        if (D && !prev_D)
        {
            if (val1_on)
            {
            	VAL1_OFF(); val1_on=0;
            }
            if (val2_on)
            {
            	VAL2_OFF(); val2_on=0;
            }
            target_steps = 0;                 // regreso a 0°
            sel = SEL_NONE;
            USART2_PutstringE("Llenado finalizado");
            stepper_start_to_target();
            LED1_OFF(); LED2_OFF(); LED3_ON(); BZ_OFF();

            // LCD: listo otra vez para escoger
            lcd_cmd_nb(0x01);
            lcd_cmd_nb(0x80); lcd_print_nb("Finalizado");
            lcd_cmd_nb(0xC0); lcd_print_nb("Seleccione A/B");
        }

        // Actualizo previos para detectar flancos bien
        prev_A=A; prev_B=B; prev_C=C; prev_D=D;

        // Dejo PB12 activo de nuevo
        GPIOB->ODR |=  (0x0Fu << 12);
        GPIOB->ODR &= ~(1u    << 12);

        EXTI->PR = pending;   // limpio las banderas de la EXTI
    }
}

// inicializo todo y suelto las IRQs
int main(void){
    // HSI16 como reloj
    RCC->CR   |= (1u<<0);
    RCC->CFGR |= (1u<<0);

    // Habilito clocks de GPIO A/B/C
    RCC->IOPENR |= (1u<<0) | (1u<<1) | (1u<<2);

    // USART2 para logs
    USART2_init();
    USART2_PutstringE("Seleccione bebida");

    // Stepper en PC0..PC3 como salidas
    GPIOC->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
    GPIOC->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
    coils_write(1);   // arranco en fase 1

    // Válvulas en PC4 y PC7
    GPIOC->MODER &= ~((3u<<(4*2)) | (3u<<(7*2)));
    GPIOC->MODER |=  ((1u<<(4*2)) | (1u<<(7*2)));
    VAL1_OFF(); VAL2_OFF();

    // 7-seg: PB0..PB7 (segmentos) y PC5/PC6/PC8/PC9 (dígitos)
    GPIOB->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|
                      (3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
    GPIOB->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|
                      (1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));
    GPIOC->MODER &= ~((3u<<(5*2))|(3u<<(6*2))|(3u<<(8*2))|(3u<<(9*2)));
    GPIOC->MODER |=  ((1u<<(5*2))|(1u<<(6*2))|(1u<<(8*2))|(1u<<(9*2)));
    fill_min = 0;
    fill_sec = 0;
    update_digits_from_mmss(fill_min, fill_sec);


    // Keypad: PB12..PB15 como salidas (columnas) y PB8..PB11 como entradas con pull-up (filas)
    GPIOB->MODER &= ~(0xFFu << 24);
    GPIOB->MODER |=  (0x55u << 24);      // PB12..PB15 = salida
    GPIOB->ODR   |=  (0x0Fu << 12);      // todas columnas en 1
    GPIOB->ODR   &= ~(1u    << 12);      // PB12 = 0 (columna activa)
    GPIOB->MODER &= ~(0xFFu << 16);      // PB8..PB11 = entrada
    GPIOB->PUPDR &= ~(0xFFu << 16);
    GPIOB->PUPDR |=  (0x55u << 16);      // pull-up en filas

    // LEDs de estado (PA11, PA12, PA15)
    GPIOA->MODER &= ~((3u<<(11*2)) | (3u<<(12*2)) | (3u<<(15*2)));
    GPIOA->MODER |=  ((1u<<(11*2)) | (1u<<(12*2)) | (1u<<(15*2)));
    LED1_OFF();  LED2_OFF(); LED3_OFF();

    // Buzzer PA7
    GPIOA->MODER &= ~(3u<<(7*2));
    GPIOA->MODER |=  (1u<<(7*2));
    BZ_OFF();

    // LCD: config de pines + SysTick a 1 kHz
    lcd_gpio_init();
    SysTick->LOAD  = (16000u - 1u);  // 16 MHz -> 1 ms por tick
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    lcd_init_nb_begin();
    lcd_cmd_nb(0x80);  lcd_print_nb("Seleccione");
    lcd_cmd_nb(0xC0);  lcd_print_nb("Bebida A/B");

    // EXTI para PB8..PB11 (detecto flancos de las filas)
    RCC->APB2ENR |= (1u<<0);  // SYSCFG
    SYSCFG->EXTICR[2] &= ~((0xFu<<0)|(0xFu<<4)|(0xFu<<8)|(0xFu<<12));
    SYSCFG->EXTICR[2] |=  ((1u<<0)|(1u<<4)|(1u<<8)|(1u<<12)); // PB8..PB11
    EXTI->IMR  |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
    EXTI->FTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11); // flanco bajada
    EXTI->RTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11); // flanco subida
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    // TIM21: velocidad de pasos del stepper
    RCC->APB2ENR |= (1u<<2);
    TIM21->PSC = 160u - 1u;                    // 16MHz/160 = 100kHz
    TIM21->ARR = (100000u/STEP_RATE_HZ) - 1u;
    TIM21->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM21_IRQn);

    // TIM22: del 7-seg
    RCC->APB2ENR |= (1u<<5);
    TIM22->PSC = 160u - 1u;  // 100 kHz
    TIM22->ARR = 50u - 1u;   // 2 kHz
    TIM22->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= TIM_CR1_CEN;

    // TIM2: base de 1 Hz para contar los segundos de llenado
    RCC->APB1ENR |= (1u<<0);
    TIM2->PSC = 15999u;   // 16MHz/(15999+1) = 1000 Hz
    TIM2->ARR = 999u;     // 1000/ (999+1) = 1 Hz
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    __enable_irq();   // habilito interrupciones globales

    while (1)
    {

    }
}


