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
  timer1_init();
  ROM_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 3 , 0, ADC_CTL_CH5 | ADC_CTL_END | ADC_CTL_IE); 
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 3 , 0, ADC_CTL_CH6 | ADC_CTL_END | ADC_CTL_IE); 
  //ROM_ADCIntEnable(ADC0_BASE, 3);
  //ROM_ADCIntEnable(ADC1_BASE, 3);
  ROM_ADCSequenceEnable(ADC0_BASE, 3);
  ROM_ADCSequenceEnable(ADC1_BASE, 3);
  //ROM_IntEnable(INT_ADC0SS3);
  //ROM_IntEnable(INT_ADC1SS3);
  ROM_ADCIntClear(ADC0_BASE,3);
  ROM_ADCIntClear(ADC1_BASE,3);
  //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
}

void timer1_init(void){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 800000);  //1khz
  //ROM_TimerControlTrigger(TIMER1_BASE, TIMER_A, true);  
  //ROM_IntDisable(INT_TIMER1A);
  ROM_IntEnable(INT_TIMER1A);
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  ROM_TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

uint32_t screen_x;
uint32_t screen_y;

void adc0_read() { 
  /*ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
  ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);*/

  ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_DIR_MODE_OUT);
  ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_OUT);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
  ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0);

  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);
  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

  ROM_ADCProcessorTrigger(ADC0_BASE, 3);
  while (!ROM_ADCIntStatus(ADC0_BASE, 3, false)){}
  ROM_ADCIntClear(ADC0_BASE, 3);
  ROM_ADCSequenceDataGet(ADC0_BASE, 3, &screen_y);

  //ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}

void adc1_read() { 
  ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_DIR_MODE_OUT);
  ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_DIR_MODE_OUT);
  ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
  ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);
  ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);
  
  ROM_ADCProcessorTrigger(ADC1_BASE, 3);
  while (!ROM_ADCIntStatus(ADC1_BASE, 3, false)){}
  ROM_ADCIntClear(ADC1_BASE, 3);
  ROM_ADCSequenceDataGet(ADC1_BASE, 3, &screen_x);
}

void adc0_sequence3_handler() {
    ROM_ADCIntClear(ADC0_BASE, 3);
}

void timer1a_handler(void){
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
  static bool read = false;
  if (read)
    adc0_read();
  else
    adc1_read();
  read = !read;
}

float x_pos;
float y_pos;

float screen_getX(void){
  int x = screen_x - 850;
  x_pos = ((x / ((2750 - 850) / 2.f)) - 1);
  return x_pos;
}

float screen_getY(void){
  int y = screen_y - 1550;
  y_pos = (y / ((2450 - 1550) / 2.f)) - 1 + x_pos;
  return y_pos;
}
