#include "Pid.h"
#include "Control.h"
#include "ImagePrc.h"

extern PID_Controller steer;

// PID控制器初始化函数
void PID_Init(PID_Controller *pid, float kp, float kp2, float kh1, float kh2, float kh12, float kh22, float kd, float khd1, float khd2, float khd12, float khd22)
{
    pid->Kp = kp;
    pid->Kp2 = kp2;
    pid->Kh1 = kh1;
    pid->Kh2 = kh2;
    pid->Kh12 = kh12;
    pid->Kh22 = kh22;
    pid->Kd = kd;
    pid->Khd1 = khd1;
    pid->Khd2 = khd2;
    pid->Khd12 = khd12;
    pid->Khd22 = khd22;
    pid->LastError = 0.0f;
}
float PID_Compute(PID_Controller *pid)
{
    float Kp = pid->Kp;
    float Kp2 = pid->Kp2;
    float Kd = pid->Kd;

    if((ImageFlag.image_element_rings_flag == 5
    ||ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag == 7 || ImageFlag.image_element_rings_flag == 8 ) && ImageFlag.image_element_rings==1 && firstringFlag == true)
    {
        Kp = pid->Kh1;
        Kd = pid->Khd1;
    }

    else if((ImageFlag.image_element_rings_flag == 5
    ||ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag == 7 || ImageFlag.image_element_rings_flag == 8 ) && ImageFlag.image_element_rings==1 && firstringFlag == false)
    {
        Kp = pid->Kh12;
        Kd = pid->Khd12;
    }

    else if((ImageFlag.image_element_rings_flag == 5
    ||ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag == 7 || ImageFlag.image_element_rings_flag == 8 ) && ImageFlag.image_element_rings==2 && firstringFlag == true)
    {
        Kp = pid->Kh2;
        Kd = pid->Khd2;
    }

    else if((ImageFlag.image_element_rings_flag == 5
    ||ImageFlag.image_element_rings_flag == 6 || ImageFlag.image_element_rings_flag == 7 || ImageFlag.image_element_rings_flag == 8 ) && ImageFlag.image_element_rings==2 && firstringFlag == false)
    {
        Kp = pid->Kh22;
        Kd = pid->Khd22;
    }

    float error = ImageStatus.Det - (56);
    // std::cout << "error:" << error << std::endl;
    float derivative = error - pid->LastError;
    pid->Output = Kp * error + Kp2 * (error * my_abs(error)) + Kd * derivative;
    pid->LastError = error;
    return pid->Output;
}

