#include "tivaware/gpio.h"
#include "tivaware/hw_ints.h"
#include "tivaware/hw_memmap.h"
#include "tivaware/pin_map.h"
#include "tivaware/pwm.h"
#include "tivaware/rom.h"
#include "tivaware/sysctl.h"
#include "tivaware/timer.h"
#include "tivaware/uart.h"
#include "screen.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "pid.h"

volatile uint32_t delay_timer;

void systick_handler(void) {
    --delay_timer;
}

void wait_ms(uint32_t t) {
    delay_timer = t;
    ROM_SysTickPeriodSet(80000 - 1);
    ROM_SysTickIntEnable();
    while (delay_timer != 0) {}
    ROM_SysTickIntDisable();
}

void timer0_init() {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, 40000000); // .5 seconds
    ROM_IntEnable(INT_TIMER0A);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerEnable(TIMER0_BASE, TIMER_BOTH);
}

void timer0a_handler(void) {
    ROM_TimerIntClear(TIMER0_BASE, TIMER_A);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,
                     ~ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7));
}

void pwm_init(void) {
    //ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
    ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);
    ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);
    ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6);
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    //ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DBG_RUN);
    //ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 25000); // 20ms period for servos
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 25000);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 25000); // 100Hz for LEDs
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 1420);  // 880Hz for buzzer
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 12500);
    //ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1875); // 1.5ms for servo pulse
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1875);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1); // 0 pulse width doesn't work
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);
    // TODO: check if 0 width by default
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_2_BIT | PWM_OUT_4_BIT,
                       true);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT | PWM_OUT_6_BIT, true);
    //ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_3);
}

void uart_init(void) {
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1 | GPIO_PIN_0);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE);
    ROM_UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    ROM_UARTFIFOEnable(UART0_BASE);
    ROM_UARTEnable(UART0_BASE);

    setvbuf(stdout, NULL, _IONBF, 0); // disable buffered output
}

void init(void) {
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ROM_SysTickEnable();
    pwm_init();
    uart_init();
    adc_init();
}

const int servo_range = 625;
const int x_servo_zero = 1875;
const int y_servo_zero = 1875;

// pos ∈ [-1,1]
void set_x(float pos) {

    pos *= -1;
    pos += .18;

    pos = fmin(pos, 0.8);
    pos = fmax(pos, -0.8);
    
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         x_servo_zero + pos * servo_range);
}

// pos ∈ [-1,1]
void set_y(float pos) {
    
    pos *= -1;
    pos -= .12;

    pos = fmin(pos, 0.8);
    pos = fmax(pos, -0.8);

    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                         y_servo_zero + pos * servo_range);
}

int max(int a, int b) {
    return a > b ? a : b;
}

// brightness ∈ [0,1]
void set_blue(float brightness) {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, max(1, brightness * 12500));
}

// brightness ∈ [0,1]
void set_red(float brightness) {
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, max(1, brightness * 12500));
}

void blink_red(void) {
    set_red(0.1f);
    wait_ms(50);
    set_red(0);
}

int main(void) {
    init();
    printf("%f\n\r", 1.4f);

    pid xpid = pid_init(0.7, 0.0, 0.0, 100.0);
    pid ypid = pid_init(0.7, 0.0, 0.0, 100.0);
        float xpos = 0.0;
        float ypos = 0.0;
        set_x(0.0f);
        set_y(0.0f);

    while (1) {
        /*blink_red();
        char c = ROM_UARTCharGet(UART0_BASE);
        switch (c) {
        case 'w': set_y(ypos = fmin(1, ypos + 0.01f)); break;
        case 'a': set_x(xpos = fmax(-1, xpos - 0.01f)); break;
        case 's': set_y(ypos = fmax(-1, ypos - 0.01f)); break;
        case 'd': set_x(xpos = fmin(1, xpos + 0.01f)); break;
        }
        printf("%.2f, %.2f\n\r", xpos, ypos);*/

        float xpos = pid_update(&xpid, screen_getX());
        float ypos = pid_update(&ypid, screen_getY());

        set_x(xpos);
        set_y(ypos);
        
        printf("%.2f %.2f\n\r", screen_getX(), screen_getY());
    }
}
