#pragma once
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../../include/ImagePrc.h"
#include "../../include/Uart.hpp"
#include "../../include/json.hpp"

using namespace cv;
using namespace std;

class Bridge
{
public:
    bool process(vector<PredictResult> predict)
    {
        if (bridgeEnable) // 进入坡道
        {
            DetL =((float)(ImageDeal[50].LeftBorder - ImageDeal[55].LeftBorder)) / ((float)(50 - 55));                               //左边界的斜率：列的坐标差/行的坐标差
            DetR =((float)(ImageDeal[50].RightBorder - ImageDeal[55].RightBorder)) / ((float)(50 - 55));         //右边界的斜率：列的坐标差/行的坐标差

            for (int ytemp = 55; ytemp >= ImageStatus.OFFLine; ytemp--)               //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
            {
                ImageDeal[ytemp].LeftBorder = (int)(DetL * ((float)(ytemp - 55))) + ImageDeal[55].LeftBorder;                                      //将这期间的空白处补线（补斜线），目的是方便图像处理
                ImageDeal[ytemp].RightBorder =(int)(DetR * ((float)(ytemp - 55))) +ImageDeal[55].RightBorder;          //将这期间的空白处补线（补斜线），目的是方便图像处理
                ImageDeal[ytemp].Center =(ImageDeal[ytemp].RightBorder + ImageDeal[ytemp].LeftBorder) / 2;
            }            

            // for (int ytemp = 59; ytemp >= ImageStatus.OFFLine; ytemp--)               //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
            // {
            //     ImageDeal[ytemp].LeftBorder = (int)(-1 * ((float)(ytemp - 55))) + 0;                                      //将这期间的空白处补线（补斜线），目的是方便图像处理
            // }

            // for (int ytemp = 59; ytemp >= ImageStatus.OFFLine; ytemp--)               //从第一次扫到的右边界的下面第二行的坐标开始往上扫直到空白上方的右边界的行坐标值
            // {
            //     ImageDeal[ytemp].RightBorder =(int)(1 * ((float)(ytemp - 55))) + 119;          //将这期间的空白处补线（补斜线），目的是方便图像处理
            // }
            
            counterSession++;
            if (counterSession > 56) // 上桥40场图像后失效
            {
                counterRec = 0;
                counterSession = 0;
                bridgeEnable = false;
                std::cout << "结束坡道" << std::endl;
            }

            return true;
        }
        else // 检测坡道
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BRIDGE && predict[i].score > 0.75 && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.25)
                {
                    counterRec++;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 2 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    bridgeEnable = true; // 检测到桥标志
                    std::cout << "检测到坡道" << std::endl;
                    return true;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }

            return false;
        }
    }

private:
    uint16_t counterSession = 0; // 图像场次计数器
    uint16_t counterRec = 0;     // 加油站标志检测计数器
    bool bridgeEnable = false;   // 桥区域使能标志
    float DetL = 0;
    float DetR = 0;
};