#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../../include/ImagePrc.h"

using namespace cv;
using namespace std;

#define DELAY_STOP   300 //ms turing

#define stopMOD 2

//改识别true或false固定进左右

class Layby
{
public:
    bool stopEnable = false;
    bool afterStop = false;

    bool process(vector<PredictResult> predict)
    {
        if (laybyEnable) // 进入临时停车状态
        {   
            if(pointEnable)
            {
                int Y = 0;
                int NUM = 0;

                if(leftEnable)
                {
                    curtailTracking(true);  //向左缩进
                    for (int Ysite = 58; Ysite > ImageStatus.OFFLine + 1; Ysite --)
                    {
                        if((ImageDeal[Ysite].LeftBoundary - ImageDeal[Ysite - 1].LeftBoundary) >= 3)
                        {
                            Y = Ysite;
                            break;
                        }    
                    }
                }
                else
                {
                    curtailTracking(false); //向右缩进
                    for (int Ysite = 58; Ysite > ImageStatus.OFFLine + 1; Ysite --)
                    {
                        if((ImageDeal[Ysite - 1].RightBoundary - ImageDeal[Ysite].RightBoundary) >= 3)
                        {
                            Y = Ysite;
                            break;
                        }    
                    }
                }
                
                if(Y >= 33) //38
                {
                    for (int Ysite = Y - 2; Ysite > Y - 20; Ysite --)
                    {
                        for (int Xsite = ImageDeal[Ysite].LeftBoundary + 2; Xsite < ImageDeal[Ysite].RightBoundary - 2; Xsite ++)
                        {
                            if (Pixle[Ysite][Xsite] == 0)
                            {
                                NUM ++;
                            }
                        }
                        if(NUM >= 10)
                            break;
                    }

                    // std::cout << "NUM:" << NUM << std::endl;
                    
                    if(NUM <= 1)
                        count ++;

                    if(count >= 1)
                    {
                        count = 0;
                        std::cout << "停车" << std::endl;
                        startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                        pointEnable = false;
                        stopEnable = true;
                    }
                }
            }
            else if(stopEnable)
            {
                endTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                if((endTime-startTime) >= DELAY_STOP)
                {
                    endEnable = true;
                    stopEnable = false;
                    std::cout << "停车结束" << std::endl;
                    return true;
                }
            }
            else if(endEnable)
            {
                int NUM;

                for (int Ysite = 58; Ysite >= 53; Ysite --)
                {
                    for (int Xsite = ImageDeal[Ysite].LeftBoundary + 6; Xsite < ImageDeal[Ysite].RightBoundary - 6; Xsite ++)
                    {
                        if (Pixle[Ysite][Xsite] == 0)
                        {
                            NUM ++;
                        }
                    }
                    if(NUM >= 10)
                        break;
                }
                if(NUM <= 1)
                {
                    endEnable = false;
                    stopEnable = false;
                    laybyEnable = false;
                    pointEnable = true;
                    afterStop = true;
                    std::cout << "出发" << std::endl;
                    return false;
                }
            }
            return true;
        }
            
        else// 检测标志
        {
            #if stopMOD == 1
            if(ImageStatus.OFFLine >= 5 || ImageStatus.RWLine >= 2 || ImageStatus.LWLine >= 2 || straightJudge(2, ImageStatus.OFFLine + 2, 45) > 1 || straightJudge(1, ImageStatus.OFFLine + 2, 45) > 1
            ||ImageDeal[55].Wide < 70 || ImageDeal[50].Wide < 65 || ImageDeal[45].Wide < 60 || ImageDeal[40].Wide < 55 || ImageDeal[35].Wide < 50)
                return false;

            for (size_t i = 0; i < predict.size(); i++)
            {
                if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY)  && predict[i].score > 0.7)  && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.07 && ImageFlag.image_element_cross == 0 && ImageFlag.image_element_rings == 0)
                {
                    if (ImageDeal[(predict[i].y + predict[i].height) / 5].Center > predict[i].x / 5)
                        leftEnable = true;              // 标识牌在左侧 true
                    else
                        leftEnable = false;
                    break;
                }
            }

            int num = 0;

            for (int Ysite = ImageStatus.OFFLine + 5; Ysite < 35; Ysite ++)
            {
                for (int Xsite =ImageDeal[Ysite].LeftBoundary + 2; Xsite < ImageDeal[Ysite].RightBoundary - 2; Xsite ++)
                {
                    if (Pixle[Ysite][Xsite] == 0 && Pixle[Ysite][Xsite + 1] == 1)
                    {
                        num ++;
                    }
                }

                if(num >= 9)
                {
                    laybyEnable = true; // 检测到标识牌子
                    std::cout << "识别到标识牌" << std::endl;
                    return true;
                }
            }

            #elif stopMOD == 2

            for (size_t i = 0; i < predict.size(); i++)
            {
                if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY)  && predict[i].score > 0.75)  && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.07 && ImageFlag.image_element_cross == 0 && ImageFlag.image_element_rings == 0)
                {
                    counterRec++;
                    if (ImageDeal[(predict[i].y + predict[i].height) / 5].Center > predict[i].x / 5)
                        leftEnable = true;              // 标识牌在左侧 true
                    else
                        leftEnable = false;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession ++;
                int num;

                for (int Ysite = ImageStatus.OFFLine; Ysite < 35; Ysite ++)
                {
                    for (int Xsite =ImageDeal[Ysite].LeftBoundary + 2; Xsite < ImageDeal[Ysite].RightBoundary - 2; Xsite ++)
                    {
                        if (Pixle[Ysite][Xsite] == 0)
                        {
                            num ++;
                        }
                    }

                    if(num >= 7)
                        break;
                }

                if(num >= 5)
                    kanFlag = true;
                else
                    kanFlag = false;

                if ((counterRec >= 1 && counterSession < 8) && kanFlag == true)
                {
                    counterRec = 0;
                    counterSession = 0;
                    laybyEnable = true; // 检测到标识牌子
                    kanFlag = false;
                    std::cout << "识别到标识牌" << std::endl;
                    return true;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    kanFlag = false;
                }
            }
            #endif
            return false;
        }
    }

    void curtailTracking(bool left)
    {
        if (left) // 向左侧缩进
        {
            for (size_t i = 59; i > ImageStatus.OFFLine; i--)
            {
                ImageDeal[i].RightBorder = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder) / 2;
                ImageDeal[i].Center = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder) / 2;
            }
        }
        else // 向右侧缩进
        {
            for (size_t i = 59; i > ImageStatus.OFFLine; i--)
            {
                ImageDeal[i].LeftBorder = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder) / 2;
                ImageDeal[i].Center = (ImageDeal[i].RightBorder + ImageDeal[i].LeftBorder) / 2;
            }
        }
    }

private:
    uint16_t counterSession = 0;    // 图像场次计数器
    uint16_t counterRec = 0;        // 标识牌检测计数器
    bool laybyEnable = false;       // 临时停车区域使能标志
    bool leftEnable = false;         // 标识牌在左侧
    bool pointEnable = true;
    bool kanFlag = false;
    bool endEnable = false;
    uint8_t count = 0;

    long long startTime;
    long long endTime;

};