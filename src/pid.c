#include "pid.h"

pid pid_init(float p, float i, float d, float interval) {
    pid pp;

    pp.pconst = p;
    pp.iconst = i;
    pp.dconst = d;

    pp.sum = 0.0;
    pp.last = 0.0;

    pp.interval = interval;

    return pp;
}

float pid_update(pid* p, float error) {

    float d = ((float)(error - p->last)) / p->interval;

    p->sum += error * p->interval;
    p->sum = fmin(p->sum, 2.5);
    p->sum = fmax(p->sum, -2.5);

    p->last = error;
    //printf("%.1f %.2f %.2f %.2f \n\r", error, p->pconst * error, p->iconst * p->sum, p->dconst * d);
    //printf("%.2f\n\r", d);

    return p->pconst * error + p->iconst * p->sum + p->dconst * d;
}
