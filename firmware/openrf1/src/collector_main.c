#include "stm32f10x.h"
#include "collector_control.h"

static Collector controller;
static volatile uint32_t milliseconds;
static USART_TypeDef *reply_uart;
static TIM_TypeDef *const encoders[WHEEL_COUNT] = {TIM3, TIM4, TIM5, TIM2};
static const unsigned pwm_channel[WHEEL_COUNT] = {1, 3, 0, 2};
static GPIO_TypeDef *const direction_port[WHEEL_COUNT] = {GPIOA, GPIOC, GPIOA, GPIOA};
static const unsigned direction_pin[WHEEL_COUNT] = {GPIO_Pin_11, GPIO_Pin_10, GPIO_Pin_8, GPIO_Pin_12};
static volatile uint16_t *const compare[WHEEL_COUNT] = {&TIM8->CCR2, &TIM8->CCR4, &TIM8->CCR1, &TIM8->CCR3};

void SysTick_Handler(void)
{
    ++milliseconds;
}

static void encoder_init(TIM_TypeDef *timer)
{
    timer->ARR = 0xffff;
    timer->CCMR1 = 0xa1a1;
    timer->SMCR = 3;
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->CR1 = TIM_CR1_CEN;
}

static void pwm_init(void)
{
    GPIO_InitTypeDef gpio = {GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12, GPIO_Speed_2MHz, GPIO_Mode_Out_PP};
    GPIOA->BRR = gpio.GPIO_Pin;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_10;
    GPIOC->BRR = GPIO_Pin_10;
    GPIO_Init(GPIOC, &gpio);

    TIM8->PSC = SystemCoreClock / 1000000 - 1;
    TIM8->ARR = 999;
    TIM8->CCMR1 = TIM8->CCMR2 = 0x6060;  /* PWM mode 1, all compares initially zero. */
    TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->BDTR = TIM_BDTR_MOE;
    TIM8->CR1 = TIM_CR1_CEN;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &gpio);
}

static void apply_outputs(void)
{
    static int last_direction[WHEEL_COUNT];
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        int duty = controller.duty[i];
        int dir = (duty > 0) - (duty < 0);
        if (dir != last_direction[i]) {
            // Clear PWM before changing the other bridge input.
            *compare[i] = 0;
            unsigned channel = pwm_channel[i];
            volatile uint16_t *mode = channel < 2 ? &TIM8->CCMR1 : &TIM8->CCMR2;
            unsigned shift = 8 * (channel % 2);
            // Reverse drives IN1 low during the active pulse while IN2 is high.
            *mode = (*mode & ~(0xffu << shift)) | ((dir < 0 ? 0x70u : 0x60u) << shift);
            direction_port[i]->BSRR = dir < 0 ? direction_pin[i] : direction_pin[i] << 16;
            last_direction[i] = dir;
        }
        *compare[i] = duty < 0 ? -duty : duty;
    }
}

static void send_char(char value)
{
    while (!(reply_uart->SR & USART_SR_TXE)) {}
    reply_uart->DR = value;
}

static void receive(USART_TypeDef *uart, uint32_t now, int motion_port)
{
    static struct { char data[64]; unsigned size; int discard; } lines[2];
    unsigned status = uart->SR;
    const unsigned errors = USART_SR_ORE | USART_SR_FE | USART_SR_PE | USART_SR_NE;
    if (!(status & (USART_SR_RXNE | errors))) return;
    unsigned value = uart->DR & 0xff;
    if (status & errors) {
        lines[motion_port].discard = 1;
        return;
    }
    if (value == '\n') {
        unsigned size = lines[motion_port].size;
        lines[motion_port].data[size] = 0;
        reply_uart = uart;
        if (size || lines[motion_port].discard) {
            const char *command = lines[motion_port].discard ? "?" : lines[motion_port].data;
            collector_line(&controller, now, command, motion_port, send_char);
            apply_outputs();
        }
        lines[motion_port].size = 0;
        lines[motion_port].discard = 0;
    } else if (value != '\r') {
        unsigned size = lines[motion_port].size;
        if (size >= sizeof lines[motion_port].data - 1 || value < ' ' || value > '~') {
            lines[motion_port].discard = 1;
        } else {
            lines[motion_port].data[size] = (char)value;
            ++lines[motion_port].size;
        }
    }
}

int main(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM8EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
                    RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    GPIOA->CRH = (GPIOA->CRH & ~(15u << 4)) | (11u << 4);
    GPIOA->CRL = (GPIOA->CRL & ~(15u << 8)) | (11u << 8);
    USART_InitTypeDef uart;
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200;
    USART_Init(USART1, &uart);
    USART_Init(USART2, &uart);
    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) encoder_init(encoders[i]);
    pwm_init();
    SysTick_Config(SystemCoreClock / 1000);

    // Reset the MCU if the main loop hangs. The 500 ms command timeout is separate.
    IWDG->KR = 0x5555;
    IWDG->PR = 4;
    IWDG->RLR = 500;
    IWDG->KR = 0xcccc;
    uint32_t sample_time = 0;
    for (;;) {
        uint32_t now = milliseconds;
        if (now - sample_time >= CONTROL_PERIOD_MS) {
            uint16_t counts[WHEEL_COUNT];
            for (unsigned i = 0; i < WHEEL_COUNT; ++i) counts[i] = encoders[i]->CNT;
            collector_tick(&controller, now, counts);
            apply_outputs();
            sample_time = now;
            IWDG->KR = 0xaaaa;
        }
        receive(USART1, now, 0);
        receive(USART2, now, 1);
    }
}
