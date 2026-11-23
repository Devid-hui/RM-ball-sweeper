#include "stm32f10x.h"

static void encoder(TIM_TypeDef *timer)
{
    timer->ARR = 0xffff;
    timer->CCMR1 = 0xa1a1; /* CH1/CH2 direct input with equal digital filters */
    timer->SMCR = 3;       /* quadrature on both edges */
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->CR1 = TIM_CR1_CEN;
}

static void send_char(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = c;
    while (!(USART2->SR & USART_SR_TXE)) {}
    USART2->DR = c;
}

static void send_count(char channel, unsigned value)
{
    char digits[5];
    unsigned n = 0;
    send_char(channel);
    send_char('=');
    do {
        digits[n++] = '0' + value % 10;
        value /= 10;
    } while (value);
    while (n) send_char(digits[--n]);
}

/* One hardware-timed burst; CFGR in the supplied SystemInit selects
   PCLK2 = HCLK. PWM2 is inactive at CNT=0 after one-pulse stop. */
static void jog(unsigned wheel)
{
    const unsigned channel[] = {1,3,0,2}; /* A .. D, zero-based TIM8 channel */
    const unsigned pin = 6 + channel[wheel];
    volatile uint16_t *compare[] = {&TIM8->CCR1, &TIM8->CCR2, &TIM8->CCR3, &TIM8->CCR4};
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_TIM8EN;
    RCC->APB2RSTR |= RCC_APB2RSTR_TIM8RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM8RST;
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_ResetBits(GPIOA,gpio.GPIO_Pin);
    GPIO_Init(GPIOA,&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_ResetBits(GPIOC,gpio.GPIO_Pin);
    GPIO_Init(GPIOC,&gpio);

    TIM8->CR1 = TIM_CR1_OPM;
    TIM8->PSC = SystemCoreClock / 1000000 - 1;
    TIM8->ARR = 999;
    TIM8->RCR = 199;
    TIM8->CCMR1 = TIM8->CCMR2 = 0x7070;
    *compare[channel[wheel]] = 800;
    TIM8->CCER = TIM_CCER_CC1E << (4 * channel[wheel]);
    TIM8->EGR = TIM_EGR_UG;
    TIM8->BDTR = TIM_BDTR_MOE;
    gpio.GPIO_Pin = 1u << pin;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC,&gpio);
    TIM8->CR1 |= TIM_CR1_CEN;
    while (TIM8->CR1 & TIM_CR1_CEN) {}
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC,&gpio);
    TIM8->BDTR = 0;
}

struct line {
    char data[32];
    unsigned length;
    unsigned discard;
};

static void reply(const char *text)
{
    while (*text) send_char(*text++);
    send_char('\r');
    send_char('\n');
}

static int equals(const char *a, const char *b)
{
    while (*a && *a == *b) { ++a; ++b; }
    return *a == *b;
}

static void execute(const struct line *line, unsigned *used)
{
    const char *cmd = line->data;
    if (equals(cmd, "PING")) reply("PONG");
    else if (equals(cmd, "ENC")) {
        send_count('A', TIM3->CNT); send_char(' ');
        send_count('B', TIM4->CNT); send_char(' ');
        send_count('C', TIM5->CNT); send_char(' ');
        send_count('D', TIM2->CNT); reply("");
    } else if (line->length == 5 && cmd[0]=='J' && cmd[1]=='O' && cmd[2]=='G' &&
               cmd[3]==' ' && cmd[4]>='A' && cmd[4]<='D') {
        unsigned wheel = cmd[4] - 'A';
        if (*used & (1u << wheel)) { reply("ALREADY"); return; }
        *used |= 1u << wheel;
        jog(wheel);
        reply("DONE");
    } else reply("ERR");
}

static void receive(USART_TypeDef *uart, struct line *line, unsigned *used)
{
    unsigned status = uart->SR;
    unsigned errors = USART_SR_ORE | USART_SR_FE | USART_SR_PE | USART_SR_NE;
    if (!(status & (USART_SR_RXNE | errors))) return;
    unsigned c = uart->DR & 255;
    if (status & errors) { line->discard = 1; return; }
    if (c == '\n') {
        if (line->discard) reply("ERR");
        else if (line->length) {
            line->data[line->length] = 0;
            execute(line, used);
        }
        line->length = line->discard = 0;
    } else if (c != '\r') {
        if (line->length >= sizeof line->data - 1 || c < ' ' || c > '~') line->discard = 1;
        else line->data[line->length++] = (char)c;
    }
}

int main(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN |
                    RCC_APB1ENR_TIM4EN | RCC_APB1ENR_TIM5EN | RCC_APB1ENR_USART2EN;
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE | AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    /* Encoder pins use their reset configuration (floating inputs).
       Only PA9 and PA2 become outputs, for USART1 and USART2 TX. */
    GPIOA->CRH = (GPIOA->CRH & ~(15u << 4)) | (11u << 4);
    GPIOA->CRL = (GPIOA->CRL & ~(15u << 8)) | (11u << 8);
    USART_InitTypeDef uart;
    USART_StructInit(&uart);
    uart.USART_BaudRate = 115200;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &uart);
    USART_Init(USART2, &uart);
    USART_Cmd(USART1, ENABLE);
    USART_Cmd(USART2, ENABLE);
    encoder(TIM3); encoder(TIM4); encoder(TIM5); encoder(TIM2);
    static struct line lines[2];
    unsigned used = 0;
    for (;;) {
        receive(USART1, &lines[0], &used);
        receive(USART2, &lines[1], &used);
    }
}
