#include "tivaware/adc.h"
#include "tivaware/hw_memmap.h"
#include "tivaware/sysctl.h"
#include "tivaware/pin_map.h"
#include "tivaware/gpio.h"
#include "tivaware/timer.h"
#include "tivaware/rom.h"
#include "screen.h"
#include "pid.h"
#include "tivaware/hw_ints.h"
#include <stdint.h>
#include <stdlib.h>

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

pid xpid; 
pid ypid; 

void timer1_init(void){
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 800000);  //1khz
  //ROM_TimerControlTrigger(TIMER1_BASE, TIMER_A, true);  
  //ROM_IntDisable(INT_TIMER1A);
  //ROM_IntEnable(INT_TIMER1A);
  ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  ROM_TimerEnable(TIMER1_BASE, TIMER_BOTH);
  xpid = pid_init(0.1, 0.0, 0.0, 100.0);
  ypid = pid_init(0.1, 0.0, 0.0, 100.0);
}


int x_buf[8];
int y_buf[8];

void adc0_read() { 

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
  
  uint32_t screen_y;

  ROM_ADCProcessorTrigger(ADC0_BASE, 3);
  while (!ROM_ADCIntStatus(ADC0_BASE, 3, false)){}
  ROM_ADCIntClear(ADC0_BASE, 3);
  ROM_ADCSequenceDataGet(ADC0_BASE, 3, &screen_y);

  static uint32_t index = 0;

  y_buf[index % 8] = screen_y;
  index++;

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

  uint32_t screen_x;
  
  ROM_ADCProcessorTrigger(ADC1_BASE, 3);
  while (!ROM_ADCIntStatus(ADC1_BASE, 3, false)){}
  ROM_ADCIntClear(ADC1_BASE, 3);
  ROM_ADCSequenceDataGet(ADC1_BASE, 3, &screen_x);


  static uint32_t index = 0;

  x_buf[index % 8] = screen_x;
  index++;
}

void adc0_sequence3_handler() {
    ROM_ADCIntClear(ADC0_BASE, 3);
}

void timer1a_handler(void){
  ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
  /*static bool read = false;
  if (read) {
    adc0_read();
    float ypos = pid_update(&ypid, screen_getY());
    set_y(ypos);
    //printf("ypos %.5f \n\r", ypos);
  } else {
    adc1_read();
    float xpos = pid_update(&xpid, screen_getX());
    set_x(xpos);
    //printf("xpos %.2f \n\r", xpos);
  }
  read = !read;*/
}

float x_pos;
float y_pos;

int cmp(const void* first, const void* second) {
  int* f = (int*)first;
  int* s = (int*)second;

  return &f - &s;
}

float screen_getX(void){
  //printf("%d \n\r", screen_x);

  int x_buf_sort[8];
  memcpy(x_buf_sort, x_buf, sizeof(int) * 8);
  qsort(x_buf_sort, 8, sizeof(int), cmp);

  int x = x_buf_sort[4] - 850;
  x_pos = ((x / ((2750 - 850) / 2.f)) - 1);
  return x_pos;
}

float screen_getY(void){
  //printf("%d \n\r", screen_y);
  int y_buf_sort[8];
  memcpy(y_buf_sort, y_buf, sizeof(int) * 8);
  qsort(y_buf_sort, 8, sizeof(int), cmp);

  int y = y_buf_sort[4] - 1700;
  y_pos = (y / ((2500 - 1700) / 2.f)) - 1 + x_pos;
  return y_pos;
}
