#ifndef PID_H
#define PID_H

// PID控制器结构体
typedef struct
{
    float Output;       // 控制输出
    float Kp;           // 比例系数
    float Kp2;
    float Kh1;
    float Khd1;
    float Kh12;
    float Khd12;
    float Kh2;
    float Khd2;
    float Kh22;
    float Khd22;
    float Kd;           // 微分系数
    float LastError;     // 上一次的误差
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float kp2, float kh1, float kh2, float kh12, float kh22, float kd, float khd1, float khd2, float khd12, float khd22);
float PID_Compute(PID_Controller *pid);

#endif
