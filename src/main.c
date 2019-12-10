#include "init.h"
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
#include <string.h>

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

void timer0a_handler(void) {
    ROM_TimerIntClear(TIMER0_BASE, TIMER_A);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7,
                     ~ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7));
}

const int servo_range = 625;
const int x_servo_zero = 1750;
const int y_servo_zero = 1700;

// pos ∈ [-1,1]
void set_x(float pos) {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         x_servo_zero -
                             fmin(fmax(-1.f, pos), 1.f) * servo_range);
}

// pos ∈ [-1,1]
void set_y(float pos) {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,
                         y_servo_zero -
                             fmin(fmax(-1.f, pos), 1.f) * servo_range);
}

int max(int a, int b) {
    return a > b ? a : b;
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

uint16_t read_adc(char axis) {
    bool xaxis = axis == 'x';
    int high = xaxis ? GPIO_PIN_0 : GPIO_PIN_1;
    int low = xaxis ? GPIO_PIN_2 : GPIO_PIN_3;
    // int z = xaxis ? GPIO_PIN_3 : GPIO_PIN_4;
    // int in = xaxis ? GPIO_PIN_1 : GPIO_PIN_2;
    int adc_mod = xaxis ? ADC1_BASE : ADC0_BASE;

    ROM_GPIODirModeSet(GPIO_PORTD_BASE, high, GPIO_DIR_MODE_OUT);
    ROM_GPIODirModeSet(GPIO_PORTD_BASE, low, GPIO_DIR_MODE_OUT);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, high, GPIO_STRENGTH_8MA,
                         GPIO_PIN_TYPE_STD);
    ROM_GPIOPadConfigSet(GPIO_PORTD_BASE, low, GPIO_STRENGTH_8MA,
                         GPIO_PIN_TYPE_STD);
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, high, high);
    ROM_GPIOPinWrite(GPIO_PORTD_BASE, low, 0);

    ROM_ADCProcessorTrigger(adc_mod, 3);
    while (!ROM_ADCIntStatus(adc_mod, 3, false)) {}
    ROM_ADCIntClear(adc_mod, 3);
    uint32_t temp;
    ROM_ADCSequenceDataGet(adc_mod, 3, &temp);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, high);
    ROM_GPIOPinTypeADC(GPIO_PORTD_BASE, low);
    return temp;
}

int cmp(const void* a, const void* b) {
    return (*(uint16_t*)a) - (*(uint16_t*)b);
}

const size_t bufsize = 16;
const uint32_t adc_read_freq = 100;

volatile size_t idx = 0;
volatile uint16_t x_buf[bufsize];
volatile uint16_t y_buf[bufsize];
volatile float X;
volatile float dX = 0;
volatile float Y;
volatile float dY = 0;

const int xmax = 3700;
const int xmin = 900;
const int ymax = 3650;
const int ymin = 2150;

uint16_t median_filter(uint16_t* buf) {
    static uint16_t buf_sort[bufsize];
    memcpy(buf_sort, buf, sizeof(uint16_t) * bufsize);
    qsort(buf_sort, bufsize, sizeof(uint16_t), cmp);
    return buf_sort[bufsize / 2];
}

void read_pos() {
    uint16_t x = read_adc('x');
    if (x > xmin && x < xmax) {
        x_buf[idx] = x;
        x = median_filter(x_buf);
        float new_x = ((x - xmin) / (float)(xmax - xmin)) * -2 + 1;
        dX = (new_x - X) * adc_read_freq;
        X = new_x;
    } else {
        dX = 0;
    }
    uint16_t y = read_adc('y');
    if (y > ymin && y < ymax) {
        y_buf[idx] = y;
        y = median_filter(y_buf);
        float new_y = ((y - ymin) / (float)(ymax - ymin)) * -2 + 1;
        dY = (new_y - Y) * adc_read_freq;
        Y = new_y;
    } else {
        dY = 0;
    }
    idx = (idx + 1) % bufsize;
}

void timer1a_handler(void) {
    ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
    read_pos();
}

void manual_servo_control() {
    float xpos = 0;
    float ypos = 0;
    const float delta = 0.1;
    while (true) {
        blink_red();
        char c = ROM_UARTCharGet(UART0_BASE);
        switch (c) {
        case 'w': set_y(ypos = fmin(1, ypos + delta)); break;
        case 'a': set_x(xpos = fmax(-1, xpos - delta)); break;
        case 's': set_y(ypos = fmax(-1, ypos - delta)); break;
        case 'd': set_x(xpos = fmin(1, xpos + delta)); break;
        }
        printf("%.2f, %.2f\n\r", xpos, ypos);
    }
}

void p_control() {
    timer1_init(adc_read_freq);
    wait_ms(100);
    const float xp = 0.3;
    const float yp = 0.3;
    while (true) {
        wait_ms(10);
        set_x(-X * xp);
        set_y(-Y * yp);
        printf("%.2f\t%.2f\t\n\r", X, Y);
    }
}

void pd_control() {
    timer1_init(adc_read_freq);
    wait_ms(100);
    const float xp = 0.3;
    const float xd = 0.125;
    const float yp = 0.3;
    const float yd = 0.125;
    while (true) {
        set_x(-X * xp + -dX * xd);
        set_y(-Y * yp + -dY * yd);
        printf("%.2f\t%.2f\t%.2f\t%.2f\t\n\r", X, Y, dX, dY);
        wait_ms(10);
    }
}

void print_pos() {
    timer1_init(adc_read_freq);
    wait_ms(100);
    while (1) {
        wait_ms(10);
        // printf("%d\n\r", read_adc('y'));
        printf("%.2f\t%.2f\t\n\r", X, Y);
        // printf("%d\t%d\t\n\r", (int)(2048 * (X + 1)), (int)(2048 * (Y + 1)));
    }
}

int main(void) {
    init();
    set_x(0.f);
    set_y(0.f);
    for (int i = 0; i < bufsize; ++i) {
        x_buf[i] = (xmin + xmax) / 2;
        y_buf[i] = (ymin + ymax) / 2;
    }
    // manual_servo_control();
    // print_pos();
    // p_control();
    pd_control();
    while (1) {}
}
