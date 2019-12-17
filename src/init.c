#include "init.h"
#include "tivaware/adc.h"
#include "tivaware/gpio.h"
#include "tivaware/hw_ints.h"
#include "tivaware/hw_memmap.h"
#include "tivaware/pin_map.h"
#include "tivaware/pwm.h"
#include "tivaware/rom.h"
#include "tivaware/sysctl.h"
#include "tivaware/timer.h"
#include "tivaware/uart.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

void timer0_init(uint32_t freq) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, 80000000 / freq);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ROM_IntEnable(INT_TIMER0A);
    ROM_IntPrioritySet(INT_TIMER1A, 0xFF);
    ROM_TimerEnable(TIMER0_BASE, TIMER_BOTH);
}

void timer1_init(uint32_t freq) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 80000000 / freq);
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    ROM_IntEnable(INT_TIMER1A);
    ROM_IntPrioritySet(INT_TIMER1A, 0);
    ROM_TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

void adc_init(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                                 ADC_CTL_CH5 | ADC_CTL_END | ADC_CTL_IE);
    ROM_ADCSequenceStepConfigure(ADC1_BASE, 3, 0,
                                 ADC_CTL_CH6 | ADC_CTL_END | ADC_CTL_IE);
    ROM_ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ROM_ADCHardwareOversampleConfigure(ADC1_BASE, 64);
    ROM_ADCSequenceEnable(ADC0_BASE, 3);
    ROM_ADCSequenceEnable(ADC1_BASE, 3);
    ROM_ADCIntClear(ADC0_BASE, 3);
    ROM_ADCIntClear(ADC1_BASE, 3);

    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_4);
}

void pwm_init(void) {
    ROM_GPIOPinConfigure(GPIO_PB6_M0PWM0);
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2);
    ROM_GPIOPinConfigure(GPIO_PE4_M0PWM4);
    ROM_GPIOPinConfigure(GPIO_PE5_M1PWM3);
    ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6);
    ROM_GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DBG_STOP);
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DBG_RUN);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 25000); // 20ms period for servos
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 25000);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 12500); // 100Hz for LEDs
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 12500);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 1420);  // 880Hz for buzzer
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1750); // neutral servo positions
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 1700);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 1); // 0 pulse width doesn't work
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_2_BIT | PWM_OUT_4_BIT,
                       true);
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT | PWM_OUT_6_BIT, true);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
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
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}
