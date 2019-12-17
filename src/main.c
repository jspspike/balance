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

// PID constants
const float xp = 0.3f;
const float xd = 0.15f;
const float yp = 0.3f;
const float yd = 0.125f;
const float xi = 0.0001f;
const float yi = 0.0001f;

const size_t bufsize = 16;  // median filter
const size_t d_bufsize = 8; // avg filter
const uint32_t adc_read_freq = 250;

// setpoints' distance from origin
const float x_dist = 0.5f;
const float y_dist = 0.4f;

// touchscreen reading extremes
const int xmax = 3100;
const int xmin = 450;
const int ymax = 3650;
const int ymin = 2150;

const int servo_range = 625;
const int x_servo_zero = 1725;
const int y_servo_zero = 1775;

enum {
    NORMAL,
    CIRCLE,
    REVERSE_CIRCLE,
    RECT,
    REVERSE_RECT,
} mode;

void debug_reset_cause(void) {
    uint32_t cause = ROM_SysCtlResetCauseGet();
    if (SYSCTL_CAUSE_BOR & cause) {
        printf("\n\rRESET DUE TO BROWN OUT\n\r");
    }
    if (SYSCTL_CAUSE_EXT & cause) {
        printf("\n\rExternal Reset\n\r");
    }
    if (SYSCTL_CAUSE_POR & cause) {
        printf("\n\rPower On Reset\n\r");
    }
    if (SYSCTL_CAUSE_SW & cause) {
        printf("\n\rSoftware Reset\n\r");
    }
    if (SYSCTL_CAUSE_HSRVREQ & cause) {
        printf("\n\rHardware System Service Request Reset\n\r");
    }
}

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

// pos ∈ [-1,1]
void set_x(float pos) {
    // printf("%d\t\n\r",
    //        (int)(x_servo_zero - fmin(fmax(-1.f, pos), 1.f) * servo_range));
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                         x_servo_zero -
                             fmin(fmax(-1.f, pos), 1.f) * servo_range);
}

// pos ∈ [-1,1]
void set_y(float pos) {
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         y_servo_zero -
                             fmin(fmax(-1.f, pos), 1.f) * servo_range);
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

void beep_ms(uint16_t freq, float volume, uint16_t ms) {
    uint16_t period = 1250000 / freq;
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, period);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, period * volume / 2);
    wait_ms(ms);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, 1);
}

void beep_beep(void) {
    beep_ms(880, 0.75, 100);
    wait_ms(50);
    beep_ms(880, 0.75, 100);
}

void boop_boop(void) {
    beep_ms(320, 0.75, 100);
    wait_ms(50);
    beep_ms(320, 0.75, 100);
}

void startup_beeps(void) {
    beep_ms(440, .75f, 100);
    beep_ms(494, .75f, 100);
    beep_ms(523, .75f, 100);
    beep_ms(587, .75f, 100);
    beep_ms(659, .75f, 100);
    beep_ms(698, .75f, 100);
    beep_ms(784, .75f, 100);
    beep_ms(880, .75f, 100);
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

volatile size_t idx = 0;
volatile uint16_t x_buf[bufsize];
volatile uint16_t y_buf[bufsize];

volatile size_t d_idx = 0;
volatile float dX_buf[d_bufsize];
volatile float dY_buf[d_bufsize];

volatile float setX;
volatile float X;
volatile float dX;
volatile float sumX;
volatile float setY;
volatile float Y;
volatile float dY;
volatile float sumY;

volatile uint16_t no_touch;

uint16_t median_filter(uint16_t* buf) {
    static uint16_t buf_sort[bufsize];
    memcpy(buf_sort, buf, sizeof(uint16_t) * bufsize);
    qsort(buf_sort, bufsize, sizeof(uint16_t), cmp);
    return buf_sort[bufsize / 2];
}

float avg_filter(float* buf, size_t len) {
    float sum = 0.f;
    for (int i = 0; i < len; ++i) { sum += buf[i]; }
    return sum / len;
}

void read_pos(void) {
    uint16_t x = read_adc('x');
    if (x < xmin || x > xmax) {
        dX = 0.f;
        dY = 0.f;
        ++no_touch;
        return;
    }

    x_buf[idx] = x;
    x = median_filter(x_buf);
    float new_x = ((x - xmin) / (float)(xmax - xmin)) * -2 + 1;
    dX_buf[d_idx] = (new_x - X) * adc_read_freq;
    X = new_x;
    dX = avg_filter(dX_buf, d_bufsize);
    sumX += signbit(X) == signbit(sumX) ? X / adc_read_freq : -sumX;

    uint16_t y = read_adc('y');
    y_buf[idx] = y;
    y = median_filter(y_buf);
    float new_y = ((y - ymin) / (float)(ymax - ymin)) * -2 + 1;
    dY_buf[d_idx] = (new_y - Y) * adc_read_freq;
    Y = new_y;
    dY = avg_filter(dY_buf, d_bufsize);
    sumY += signbit(Y) == signbit(sumY) ? Y / adc_read_freq : -sumY;

    no_touch = 0;
    idx = (idx + 1) % bufsize;
    d_idx = (d_idx + 1) % d_bufsize;
}

void timer1a_handler(void) {
    ROM_TimerIntClear(TIMER1_BASE, TIMER_A);
    read_pos();
}

void manual_servo_control(void) {
    float xpos = 0;
    float ypos = 0;
    const float delta = 0.01;
    while (true) {
        blink_red();
        char c = ROM_UARTCharGet(UART0_BASE);
        switch (c) {
        case 'w': set_y(ypos = fmin(1, ypos + delta)); break;
        case 'a': set_x(xpos = fmax(-1, xpos - delta)); break;
        case 's': set_y(ypos = fmax(-1, ypos - delta)); break;
        case 'd': set_x(xpos = fmin(1, xpos + delta)); break;
        }
        printf("%-6d\t%-6d\t\n\r",
               (int)(x_servo_zero - fmin(fmax(-1.f, xpos), 1.f) * servo_range),
               (int)(y_servo_zero - fmin(fmax(-1.f, ypos), 1.f) * servo_range));
        // printf("%.2f, %.2f\n\r", xpos, ypos);
    }
}

volatile float speed = 1.f;
volatile float time = 0.f;
const float tau = 6.2831853f;
const uint32_t counter_freq = 1000;

void timer2a_handler(void) {
    ROM_TimerIntClear(TIMER2_BASE, TIMER_A);
    time += speed / counter_freq;
    if (time >= tau) {
        time -= tau;
    }
}

volatile bool did_beep = false;
volatile bool pls_beep = false;

void timer0a_handler(void) {
    ROM_TimerIntClear(TIMER0_BASE, TIMER_A);
    if (no_touch > adc_read_freq) {
        set_x(0);
        set_y(0);
        pls_beep = true;
        return;
    }
    pls_beep = did_beep = false;
    float temp = time;
    switch (mode) {
    case NORMAL: break;
    case REVERSE_CIRCLE: temp = -time;
    case CIRCLE:
        setX = cosf(temp) * x_dist * .9f;
        setY = sinf(temp) * y_dist * 1.3f;
        break;
    case REVERSE_RECT: temp = -time + tau;
    case RECT:
        if (temp < tau / 4) {
            setX = -x_dist, setY = -y_dist;
        } else if (temp < tau / 2) {
            setX = x_dist, setY = -y_dist;
        } else if (temp < 3 * tau / 4) {
            setX = x_dist, setY = y_dist;
        } else {
            setX = -x_dist, setY = y_dist;
        }
        break;
    }
    float x = (setX - X) * xp - dX * xd - xi * sumX;
    set_x(x);
    set_blue(fabs(x) / 32);
    float y = (setY - Y) * yp - dY * yd - yi * sumY;
    set_y(y);
    set_red(fabs(y) / 32);
}

void control_loop(void) {
    timer1_init(adc_read_freq);
    timer2_init(counter_freq);
    wait_ms(1000 * bufsize / adc_read_freq);
    timer0_init(100);
    bool printing = false;
    while (true) {
        if (pls_beep && !did_beep) {
            beep_beep();
            did_beep = true;
        }
        wait_ms(10);
        if (printing) {
            printf("%-5d\t%-5d\t\n\r", (int)((setX - X) * 2048.f),
                   (int)((setY - Y) * 2048.f));
        }
        // printf("%.02f\t%.02f\t\n\r", setX - X, setY - Y);
        if (!ROM_UARTCharsAvail(UART0_BASE)) {
            continue;
        }
        int32_t c = ROM_UARTCharGetNonBlocking(UART0_BASE);
        char* setpoints = "qweasdzxc";
        char* modes = "1234";
        if (strchr(setpoints, c) != NULL) {
            mode = NORMAL;
            beep_ms(440, .75f, 100);
        } else if (strchr(modes, c) != NULL) {
            boop_boop();
        }
        switch (c) {
        case 'q': setX = -x_dist, setY = y_dist; break;
        case 'w': setX = 0.f, setY = y_dist; break;
        case 'e': setX = x_dist, setY = y_dist; break;
        case 'a': setX = -x_dist, setY = 0.0; break;
        case 's': setX = 0.f, setY = 0.0; break;
        case 'd': setX = x_dist, setY = 0.0; break;
        case 'z': setX = -x_dist, setY = -y_dist; break;
        case 'x': setX = 0.f, setY = -y_dist; break;
        case 'c': setX = x_dist, setY = -y_dist; break;
        case '1': mode = RECT; break;
        case '2': mode = REVERSE_RECT; break;
        case '3': mode = CIRCLE; break;
        case '4': mode = REVERSE_CIRCLE; break;
        case 'h': beep_ms(196, .75f, 250); break;
        case 'j': beep_ms(262, .75f, 250); break;
        case 'k': beep_ms(294, .75f, 250); break;
        case 'l': beep_ms(330, .75f, 250); break;
        case ';': beep_ms(392, .75f, 250); break;
        case '+': speed += 0.1f; break;
        case '-': speed -= 0.1f; break;
        case 'p': printing = !printing; break;
        }
        if (c == '+' || c == '-') {
            beep_ms(880 * speed, .75f, 150);
        }
    }
}

void print_pos(void) {
    timer1_init(adc_read_freq);
    wait_ms(100);
    while (true) {
        wait_ms(10);
        // printf("%d\n\r", read_adc('x'));
        // printf("%d\t%d\t\n\r", median_filter(x_buf), median_filter(y_buf));
        printf("%.2f\t%.2f\t\n\r", X, Y);
        // printf("%d\t%d\t\n\r", (int)(X * 4000), (int)(dX * 1000.f));
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
    for (int i = 0; i < d_bufsize; ++i) { dX_buf[i] = dY_buf[i] = 0.f; }
    setX = setY = X = Y = dX = dY = sumX = sumY = 0.f;
    startup_beeps();
    // manual_servo_control();
    // print_pos();
    control_loop();
}
