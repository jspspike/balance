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
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); 
    ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    timer1_init();
    //ROM_ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_FULL, 1);
    //ROM_ADCSequenceDisable(ADC0_BASE, 3);
    ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ROM_ADCSequenceStepConfigure(ADC0_BASE, 3 , 0, ADC_CTL_CH0 | ADC_CTL_END | ADC_CTL_IE); //Change
    ROM_ADCIntEnable(ADC0_BASE, 3);
    //ROM_ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);
    ROM_ADCSequenceEnable(ADC0_BASE, 3);
}

void timer1_init(void){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 80000);  //1khz
    ROM_TimerControlTrigger(TIMER1_BASE, TIMER_A, true);  
    //ROM_TimerADCEventSet(TIMER1_BASE, TIMER_ADC_TIMEOUT_A);
    ROM_IntDisable(INT_TIMER1A);
    ROM_TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

uint32_t adc_value;
void adc0_sequence3_handler(void){
    ROM_ADCIntClear(ADC0_BASE,3);
    ROM_ADCSequenceDataGet(ADC0_BASE, 3, &adc_value);
}
