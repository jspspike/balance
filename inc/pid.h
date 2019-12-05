struct pid {
    float pconst;
    float iconst;
    float dconst;
    float sum;
    float last;
    float interval;
} typedef pid;

pid pid_init(float p, float i, float d, float interval);

float pid_update(pid* p, float error);
