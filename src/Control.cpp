#include "Control.h"
#include "ImagePrc.h"

float my_abs(float err)
{
    if(err >= 0)
        err = err;
    else
        err = -err;
    return err;
}

//误差计算函数
//不用了
float errSum(void)
{
    int i;
    float err = 0;
    static float last_err = 0;

    if(ImageStatus.OFFLine < (LCDH - 5) )
    {
        for(i = (ImageStatus.OFFLine); i <= LCDH -1; i++)
        {
           err += ((LCDW / 2) - (ImageDeal[i].RightBoundary + ImageDeal[i].LeftBoundary) / 2);//ImageDeal[i].Center
        }
        err = err / (LCDH -1 - (ImageStatus.OFFLine));
        last_err = err;
        // std::cout << "last_err:" << last_err << std::endl;
        // std::cout << "err:" << err << std::endl;
    }
    else
    {
        return last_err;
    }

    return err;
}

float speedControl(float speedHigh, float speedLow)
{
    if(ImageStatus.Road_type == Straight)
        return speedHigh;
    else
        return speedLow;
}
