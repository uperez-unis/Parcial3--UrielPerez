#include <stdint.h>
#include "stm32l053xx.h"

//Configuración general
#define STEPS_PER_REV   2048u
#define STEP_RATE_HZ    400u
#define STEPS_45_DEG    (STEPS_PER_REV/8) // Giro del motor aprox 45°
#define STEPS_30_DEG    (STEPS_PER_REV/19) // Giro del motor aprox 20°

// USART2 logs al usuaro
static void USART2_init(void){
    RCC->APB1ENR |= (1u<<17);
    GPIOA->MODER &= ~(1u<<4);
    GPIOA->MODER &= ~(1u<<6);
    GPIOA->AFR[0] |= (1u<<10) | (1u<<14);
    USART2->BRR = 0x0682;
    USART2->CR1 |= (1u<<2) | (1u<<3) | (1u<<0);
}


static void USART2_write(uint8_t ch)
{
	while(!(USART2->ISR & 0x0080))
	{

	}
	USART2->TDR = ch;
}
static void USART2_PutstringE(const char *s)
{
	while(*s)
	USART2_write((uint8_t)*s++);
	USART2_write('\n');
	USART2_write('\r');
}

//Actuadores (válvulas activas en bajo)
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

//Indicadores leds
static inline void LED1_ON (void)
{
	GPIOA->ODR |=  (1u<<11); //selección
}
static inline void LED1_OFF(void)
{
	GPIOA->ODR &= ~(1u<<11);
}
static inline void LED2_ON (void)
{
	GPIOA->ODR |=  (1u<<12); //lenando
}
static inline void LED2_OFF(void)
{
	GPIOA->ODR &= ~(1u<<12);
}
static inline void LED3_ON (void)
{
	GPIOA->ODR |=  (1u<<15); //finalizado
}
static inline void LED3_OFF(void)
{
	GPIOA->ODR &= ~(1u<<15);
}

//Buzzer (PA7 activo en alto)
static inline void BZ_ON (void)
{
	GPIOA->ODR |=  (1u<<7);
}
static inline void BZ_OFF(void)
{
	GPIOA->ODR &= ~(1u<<7);
}

//Variables
typedef enum { SEL_NONE=0, SEL_A=1, SEL_B=2 } sel_t;
static volatile sel_t sel = SEL_NONE;
static volatile uint8_t val1_on=0, val2_on=0;
static volatile uint8_t prev_A=0, prev_B=0, prev_C=0, prev_D=0;

//Stepper (PC0..PC3) con TIM21
static inline void coils_write(uint8_t phase) {
    GPIOC->BSRR = (0x0Fu << 16);
    switch (phase) {
        case 1: GPIOC->BSRR = (1u<<0); break;
        case 2: GPIOC->BSRR = (1u<<1); break;
        case 3: GPIOC->BSRR = (1u<<2); break;
        case 4: GPIOC->BSRR = (1u<<3); break;
        default: break;
    }
}
static volatile int32_t pos_steps    = 0;
static volatile int32_t target_steps = 0;
static volatile uint8_t phase        = 1;

static inline void stepper_start_to_target(void)
{
	if (target_steps != pos_steps)
		TIM21->CR1 |= TIM_CR1_CEN;
}
static inline void stepper_stop(void)
{
	TIM21->CR1 &= ~TIM_CR1_CEN;
}

void TIM21_IRQHandler(void)
{
    if (TIM21->SR & TIM_SR_UIF) {
        TIM21->SR &= ~TIM_SR_UIF;

        if (pos_steps < target_steps) {
            phase = (phase == 4) ? 1 : (phase + 1);
            coils_write(phase);
            pos_steps++;
            if (pos_steps == target_steps) { stepper_stop(); USART2_PutstringE("Posicionado"); }
        } else if (pos_steps > target_steps) {
            phase = (phase == 1) ? 4 : (phase - 1);
            coils_write(phase);
            pos_steps--;
            if (pos_steps == target_steps) { stepper_stop(); USART2_PutstringE("Posicionado"); }
        } else {
            stepper_stop();
        }
    }
}

//Display 7-seg (PB0..PB7 seg, PC5/PC6/PC8/PC9 dígitos) con TIM22
static const uint8_t seg_lut[10] = {
    0b00111111,
	0b00000110,
	0b01011011,
	0b01001111,
	0b01100110,
    0b01101101,
	0b01111101,
	0b00000111,
	0b01111111,
	0b01101111
};
static volatile uint8_t disp_digits[4] = {0,0,0,0};

static inline void digits_all_off(void){
    GPIOC->BSRR = (1u<<(5+16)) | (1u<<(6+16)) | (1u<<(8+16)) | (1u<<(9+16));
}
static inline void digit_on(uint8_t idx){
    static const uint8_t pins[4] = {9, 8, 6, 5};
    GPIOC->BSRR = (1u << pins[idx]);
}
static inline void seg_write(uint8_t pattern){
    GPIOB->BSRR = (0xFFu << 16);
    GPIOB->BSRR = (uint32_t)pattern;
}
static inline void update_digits_from_seconds(uint16_t sec){
    if (sec > 9999u) sec = 9999u;
    disp_digits[3] =  sec        % 10u;
    disp_digits[2] = (sec/10u)   % 10u;
    disp_digits[1] = (sec/100u)  % 10u;
    disp_digits[0] = (sec/1000u) % 10u;
}

void TIM22_IRQHandler(void)
{
    if (TIM22->SR & TIM_SR_UIF) {
        TIM22->SR &= ~TIM_SR_UIF;

        static uint8_t idx = 0;
        digits_all_off();

        uint8_t blank = 0;
        if (idx==0 && disp_digits[0]==0 && (disp_digits[1]|disp_digits[2]|disp_digits[3])!=0) blank=1;
        if (idx==1 && disp_digits[0]==0 && disp_digits[1]==0 && (disp_digits[2]|disp_digits[3])!=0) blank=1;
        if (idx==2 && disp_digits[0]==0 && disp_digits[1]==0 && disp_digits[2]==0 && disp_digits[3]!=0) blank=1;

        seg_write(blank ? 0x00 : seg_lut[disp_digits[idx]]);
        digit_on(idx);
        idx = (uint8_t)((idx + 1) & 0x03);
    }
}

// Contador de llenado (TIM2 → 1 Hz)
static volatile uint16_t fill_seconds = 0;

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        if (val1_on || val2_on) {
            if (fill_seconds < 9999u) fill_seconds++;
            update_digits_from_seconds(fill_seconds);
        }
    }
}

// Keypad (EXTI4_15) – columna PB12 (A/B/C/D)
static inline uint32_t read_rows(void){ return (GPIOB->IDR >> 8) & 0x0Fu; }  // 0 = presionada

void EXTI4_15_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & ((1u<<8)|(1u<<9)|(1u<<10)|(1u<<11));
    if (pending) {
        GPIOB->ODR |=  (0x0Fu << 12);
        GPIOB->ODR &= ~(1u    << 12);

        uint32_t rows = read_rows();
        uint8_t A = ((rows & 0x1u) == 0u);
        uint8_t B = ((rows & 0x2u) == 0u);
        uint8_t C = ((rows & 0x4u) == 0u);
        uint8_t D = ((rows & 0x8u) == 0u);

        if (A && !prev_A) {
            sel = SEL_A;
            target_steps = -(int32_t)STEPS_30_DEG;
            USART2_PutstringE("Bebida A seleccionada");
            stepper_start_to_target();
            LED1_ON(); LED2_OFF(); LED3_OFF(); BZ_OFF();
        }

        if (B && !prev_B) {
            sel = SEL_B;
            target_steps =  (int32_t)STEPS_30_DEG;
            USART2_PutstringE("Bebida B seleccionada");
            stepper_start_to_target();
            LED1_ON(); LED2_OFF(); LED3_OFF(); BZ_OFF();
        }

        if (C && !prev_C) {
            // inicio de llenado: reinicio contador y abro válvula de la selección
            fill_seconds = 0;
            update_digits_from_seconds(0);

            if (sel == SEL_A) {
                VAL1_ON();  val1_on = 1;
                if (val2_on){ VAL2_OFF(); val2_on = 0; }
                USART2_PutstringE("Llenando bebida A...");
            } else if (sel == SEL_B) {
                VAL2_ON();  val2_on = 1;
                if (val1_on){ VAL1_OFF(); val1_on = 0; }
                USART2_PutstringE("Llenando bebida B...");
            } else {
                USART2_PutstringE("Seleccione bebida primero");
            }

            LED1_OFF(); LED2_ON(); LED3_OFF(); BZ_ON();
        }

        if (D && !prev_D) {
            // fin de ciclo: cierro válvulas, regreso a 0°, señalo finalizado
            if (val1_on){ VAL1_OFF(); val1_on=0; }
            if (val2_on){ VAL2_OFF(); val2_on=0; }
            target_steps = 0;
            sel = SEL_NONE;
            USART2_PutstringE("Llenado finalizado");
            stepper_start_to_target();
            LED1_OFF(); LED2_OFF(); LED3_ON(); BZ_OFF();
        }

        prev_A=A; prev_B=B; prev_C=C; prev_D=D;

        GPIOB->ODR |=  (0x0Fu << 12);
        GPIOB->ODR &= ~(1u    << 12);

        EXTI->PR = pending;
    }
}


int main(void)
{
    // reloj base
    RCC->CR   |= (1u<<0);
    RCC->CFGR |= (1u<<0);

    // GPIO A/B/C
    RCC->IOPENR |= (1u<<0) | (1u<<1) | (1u<<2);

    // USART2
    USART2_init();
    USART2_PutstringE("Seleccione bebida");

    // Stepper: PC0..PC3
    GPIOC->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2)));
    GPIOC->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2)));
    coils_write(1);

    // Válvulas: PC4, PC7
    GPIOC->MODER &= ~((3u<<(4*2)) | (3u<<(7*2)));
    GPIOC->MODER |=  ((1u<<(4*2)) | (1u<<(7*2)));
    VAL1_OFF(); VAL2_OFF();

    // Display: PB0..PB7 seg, PC5/6/8/9 dígitos
    GPIOB->MODER &= ~((3u<<(0*2))|(3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|
                      (3u<<(4*2))|(3u<<(5*2))|(3u<<(6*2))|(3u<<(7*2)));
    GPIOB->MODER |=  ((1u<<(0*2))|(1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|
                      (1u<<(4*2))|(1u<<(5*2))|(1u<<(6*2))|(1u<<(7*2)));
    GPIOC->MODER &= ~((3u<<(5*2))|(3u<<(6*2))|(3u<<(8*2))|(3u<<(9*2)));
    GPIOC->MODER |=  ((1u<<(5*2))|(1u<<(6*2))|(1u<<(8*2))|(1u<<(9*2)));
    digits_all_off();
    update_digits_from_seconds(0);

    // Keypad: columnas PB12..PB15 (PB12 activa en 0), filas PB8..PB11 pull-up
    GPIOB->MODER &= ~(0xFFu << 24);
    GPIOB->MODER |=  (0x55u << 24);
    GPIOB->ODR   |=  (0x0Fu << 12);
    GPIOB->ODR   &= ~(1u    << 12);
    GPIOB->MODER &= ~(0xFFu << 16);
    GPIOB->PUPDR &= ~(0xFFu << 16);
    GPIOB->PUPDR |=  (0x55u << 16);

    // LEDs: PA11, PA12, PA15
    GPIOA->MODER &= ~((3u<<(11*2)) | (3u<<(12*2)) | (3u<<(15*2)));
    GPIOA->MODER |=  ((1u<<(11*2)) | (1u<<(12*2)) | (1u<<(15*2)));
    LED1_OFF(); LED2_OFF(); LED3_OFF();

    // Buzzer: PA7
    GPIOA->MODER &= ~(3u<<(7*2));
    GPIOA->MODER |=  (1u<<(7*2));
    BZ_OFF();

    // EXTI PB8..PB11
    RCC->APB2ENR |= (1u<<0);
    SYSCFG->EXTICR[2] &= ~((0xFu<<0)|(0xFu<<4)|(0xFu<<8)|(0xFu<<12));
    SYSCFG->EXTICR[2] |=  ((1u<<0)|(1u<<4)|(1u<<8)|(1u<<12));
    EXTI->IMR  |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
    EXTI->FTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
    EXTI->RTSR |= (1u<<8)|(1u<<9)|(1u<<10)|(1u<<11);
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    // TIM21 (stepper)
    RCC->APB2ENR |= (1u<<2);
    TIM21->PSC = 160u - 1u;
    TIM21->ARR = (100000u/STEP_RATE_HZ) - 1u;
    TIM21->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM21_IRQn);

    // TIM22 (display ~2 kHz)
    RCC->APB2ENR |= (1u<<5);
    TIM22->PSC = 160u - 1u;
    TIM22->ARR = 50u - 1u;
    TIM22->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM22_IRQn);
    TIM22->CR1 |= TIM_CR1_CEN;

    // TIM2 (1 Hz contador)
    RCC->APB1ENR |= (1u<<0);
    TIM2->PSC = 15999u;
    TIM2->ARR = 999u;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    __enable_irq();

    while (1)
    {

    }
}
