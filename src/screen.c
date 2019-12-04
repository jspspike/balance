#include "tivaware/adc.h"
#include "tivaware/hw_memmap.h"
#include "tivaware/sysctl.h"
#include "tivaware/pin_map.h"
#include "tivaware/gpio.h"
#include "tivaware/timer.h"
#include "tivaware/rom.h"
#include "screen.h"
#include "tivaware/hw_ints.h"
#include <stdint.h>

void timer1_init(void);
void adc_init(void){
    //Port Init
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    timer1_init();
    ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3 , 0, ADC_CTL_CH4 | ADC_CTL_END | ADC_CTL_IE); 
    ROM_ADCSequenceStepConfigure(ADC1_BASE, 3 , 0, ADC_CTL_CH5 | ADC_CTL_END | ADC_CTL_IE); 
    ROM_ADCIntEnable(ADC0_BASE, 3);
    ROM_ADCIntEnable(ADC1_BASE, 3);
    ROM_ADCSequenceEnable(ADC0_BASE, 3);
    ROM_ADCSequenceEnable(ADC1_BASE, 3);
    ROM_IntEnable(INT_ADC0SS3);
    ROM_IntEnable(INT_ADC1SS3);
    //ROM_ADCIntClear(ADC0_BASE,3);
    //ROM_ADCIntClear(ADC1_BASE,3);
}

void timer1_init(void){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 80000);  //1khz
    ROM_TimerControlTrigger(TIMER1_BASE, TIMER_A, true);  
    ROM_IntDisable(INT_TIMER1A);
    ROM_TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

uint32_t screen_x;
void adc0_sequence3_handler(void){
    ROM_ADCIntClear(ADC0_BASE,3);
    ROM_ADCSequenceDataGet(ADC0_BASE, 3, &screen_x);
}

uint32_t screen_y;
void adc1_sequence3_handler(void){
    ROM_ADCIntClear(ADC1_BASE,3);
    ROM_ADCSequenceDataGet(ADC1_BASE, 3, &screen_y);
}

uint32_t screen_getX(void){
    return screen_x;
}

uint32_t screen_getY(void){
    return screen_y;
}