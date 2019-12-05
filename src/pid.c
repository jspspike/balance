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

    float d = (error - p->last) / p->interval;

    p->sum += error * p->interval;

    p->last = error;

    return p->pconst * error + p->iconst * p->sum + p->dconst * d;
}
