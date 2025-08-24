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

#define DELAY_STOP   300 //ms 停车时间

class Catering
{
    public:

        bool stopEnable = false;              // 停车使能标志
        uint16_t counterSession = 0;          // 图像场次计数器
        uint16_t counterRec = 0;              // 汉堡标志检测计数器
        bool cateringEnable = false;          // 岔路区域使能标志
        bool burgerLeft = false;              // 汉堡在左侧
        bool Point_guai = false;              // 拐点标志位

        bool process(vector<PredictResult> predict)
        {
            if(!cateringEnable) // 检测汉堡标志
            {
                if(ImageStatus.OFFLine >= 5)
                    return false;
                 //检测餐饮区标志，并返回检测到的结果
                for (size_t i = 0; i < predict.size(); i++)
                {
                    //这里面检查的时候就已经准确确定汉堡的位置和准确率
                    if (predict[i].type == LABEL_BURGER && predict[i].score >= 0.7 && (predict[i].y + predict[i].height) > ROWSIMAGE * 0.1 && ImageFlag.image_element_cross == 0 && ImageFlag.image_element_rings == 0)//0.3太早就检测到了
                    {
                        counterRec++;
                        if (
                            // predict[i].x < COLSIMAGE / 2 && 
                            ImageDeal[(predict[i].y + predict[i].height)/5].Center > predict[i].x / 5){
                                std::cout << "左汉堡" << std::endl;
                                burgerLeft = true;              // 确定汉堡的左右位置
                            }
                        else if (
                            // predict[i].x > COLSIMAGE / 2 && 
                            ImageDeal[(predict[i].y + predict[i].height)/5].Center < predict[i].x / 5){
                                burgerLeft = false;
                                std::cout << "右汉堡" << std::endl;
                            }
                        break;
                        }
                    }
                    
                    for(int i = 55; i >= ImageStatus.OFFLine + 2; i--)
                {
                    if(burgerLeft && ImageDeal[i].RightBoundary - ImageDeal[i].RightBoundary_First >= 4 && ImageDeal[i - 1].RightBoundary - ImageDeal[i - 1].RightBoundary_First >= 4)
                    {
                        Point_guai = true;
                        break;
                    }

                    if(!burgerLeft && ImageDeal[i].LeftBoundary_First - ImageDeal[i].LeftBoundary >= 4 && ImageDeal[i - 1].LeftBoundary_First - ImageDeal[i - 1].LeftBoundary >= 4)
                    {
                        Point_guai = true;
                        break;
                    }
                }
        
                if (counterRec)
                {
                    counterSession++;
                    if (counterRec >= 1 && counterSession < 10 
                        && Point_guai
                        )//八帧图像里面有三次检测到了圆环
                    {
                        counterRec = 0;
                        counterSession = 0;
                        Point_guai = false;
                        cateringEnable = true;                // 检测到汉堡标志
                        std::cout << "识别到汉堡" << std::endl;

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
            else
            {   
                // int stopCounter = 0;
                //左边汉堡
                if(burgerLeft == true && stopEnable == false)
                {
                    for(int i = LCDH - 4 ; i > ImageStatus.OFFLine + 1 ; i --)
                    {
                        if (ImageDeal[i].LeftBoundary_First - ImageDeal[i - 1].LeftBoundary_First >= 8)
                        {
                            burgerFirst_Y = i;
                            if(stopCounter == 0)
                            {
                                for(int j = i ; j <= LCDH - 4 ; j ++)
                                {
                                    if (ImageDeal[j + 1].LeftBoundary - ImageDeal[j].LeftBoundary >= 4 && burgerFirst_Y >= 4)//停车时机！！！！！！！！！！！！！！！！！！
                                    {
                                        burgerPoint_X = ImageDeal[i].LeftBoundary_First;
                                        // burgerPoint_Y = i;
                                        stopEnable = true;
                                        stopCounter ++;
                                        start_time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                                        break;
                                    }
                                }
                            }
                        }
                        if(stopEnable)break;
                    }
                    if(afterStop == true && burgerFirst_Y >= 45)
                        burgerPointflag = true;

                    int leftnum = 0;

                    for(int Y = ImageStatus.OFFLine + 1; Y <= 45; Y ++)
                    {
                        if(ImageDeal[Y].IsLeftFind == 'W')
                        {
                            leftnum ++;
                        }
                    }

                    if (burgerPointflag == true && leftnum <= 15)    //右边为直线且截止行（前瞻值）很小
                    {
                        cateringEnable = false;  // 结束餐饮区标志位
                        burgerLeft = false;      // 汉堡在左侧
                        stopEnable = false;      // 停车失能
                        afterStop = false ;      //停车之后标志位
                        stopCounter = 0;
                        burgerFirst_Y = 0;
                        burgerPoint_Y = 0;
                        burgerPointflag = false;
                        leftnum = 0 ;
                        std::cout << "退出餐饮区" << std::endl;
                        // return false;
                    }
                    if(afterStop == false)
                    {
                        //重新规划赛道中线
                        for(int Ysite = LCDH - 2; Ysite >= 5; Ysite --)//从右上拐点开始向汉堡坐标进行补线
                        {
                            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite].RightBoundary ;
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBoundary - Half_Road_Wide[Ysite] ;                            
                            if(ImageDeal[Ysite].Center < 0)ImageDeal[Ysite].Center = 0;
                        }
                    }
                    else
                    {
                        //重新规划赛道中线
                        for(int Ysite = LCDH - 2; Ysite >= 5; Ysite --)//从右上拐点开始向汉堡坐标进行补线
                        {
                            ImageDeal[Ysite].RightBorder = ImageDeal[Ysite].RightBoundary_First ;
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBoundary_First - Half_Road_Wide[Ysite];                            
                            if(ImageDeal[Ysite].Center < 0)ImageDeal[Ysite].Center = 0;
                        }
                    }
                }

                //右边汉堡
                else if(burgerLeft == false && stopEnable == false)
                {
                    // int stopCounter = 0;
                    for(int i = LCDH - 4 ; i >ImageStatus.OFFLine + 1; i --)
                    {
                        if (ImageDeal[i - 1].RightBoundary_First - ImageDeal[i].RightBoundary_First >= 8 )
                        {
                            burgerFirst_Y = i;
                            if(stopCounter == 0)
                            {
                                for(int j = i ; j <= LCDH - 4 ; j ++)
                                {
                                    if (ImageDeal[j - 1].RightBoundary - ImageDeal[j].RightBoundary >= 4 && burgerFirst_Y >= 4)//停车时机！！！！！！！！！！
                                    {
                                        burgerPoint_X = ImageDeal[i].RightBoundary_First;
                                        // burgerPoint_Y = i;
                                        stopEnable = true;
                                        stopCounter ++;
                                        start_time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                                        break;
                                    }
                                }
                            }
                        }
                        if(stopEnable)break;
                    }
                    if(afterStop == true && burgerFirst_Y >= 42)//53
                        burgerPointflag = true;

                    int rightnum = 0;

                    for(int Y = ImageStatus.OFFLine + 1; Y < 45; Y ++)
                    {
                        if(ImageDeal[Y].IsRightFind == 'W')
                        {
                            rightnum ++;
                        }
                    }

                    if (burgerPointflag == true && rightnum <= 15)//3 5   //右边为直线且截止行（前瞻值）很小
                    {
                        cateringEnable = false;  // 结束餐饮区标志位
                        burgerLeft = false;      // 汉堡在左侧
                        stopEnable = false;      // 停车失能
                        afterStop = false ;      //停车之后标志位
                        stopCounter = 0;
                        burgerFirst_Y = 0;
                        burgerPoint_Y = 0;
                        burgerPointflag = false;
                        rightnum = 0 ;
                        std::cout << "退出餐饮区" << std::endl;
                        // return false;
                    }
                
                    if(afterStop == false)
                    {
                        //重新规划赛道中线
                        for(int Ysite = LCDH - 3; Ysite >= 5; Ysite --)//从右上拐点开始向汉堡坐标进行补线
                        {
                            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite].LeftBoundary ;     //y=y0-K(x-x0);从左上拐点补到图像的面包坐标点
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBoundary + Half_Road_Wide[Ysite];                            
                            if(ImageDeal[Ysite].Center > 120)ImageDeal[Ysite].Center = 120;
                        }
                    }
                    else
                    {
                        //重新规划赛道中线
                        for(int Ysite = LCDH - 3; Ysite >= 5; Ysite --)//从右上拐点开始向汉堡坐标进行补线
                        {
                            ImageDeal[Ysite].LeftBorder = ImageDeal[Ysite].LeftBoundary_First ;     //y=y0-K(x-x0);从左上拐点补到图像的面包坐标点
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBoundary_First + Half_Road_Wide[Ysite];                            
                            if(ImageDeal[Ysite].Center > 118)ImageDeal[Ysite].Center = 119;
                        }
                    }
                }
                else if(stopEnable == true)
                {
                    // std::cout << "开始停车" << std::endl;
                    end_time =chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                    if(end_time - start_time > DELAY_STOP)
                    {
                        afterStop = true;
                        stopEnable = false;
                        std::cout << "结束停车" << std::endl;
                    }
                }
                return true;
            }
        }
    private:
        //delay计时开始结束时间
        long long start_time;
        long long end_time;

        bool afterStop = false;
        // bool burgerPointflag = false; 
        int burgerY = 0;
        int burgerPoint_X = 0;
        int burgerPoint_Y = 0;
        int burgerFirst_Y = 0;
        int stopCounter = 0;
        bool burgerPointflag = false;

        uint8_t Half_Road_Wide[60]=                      //赛道半宽
        {      
        10,10,10,11,12,12,13,13,14,15,
        15,16,16,17,17,18,18,19,20,20,
        21,21,22,23,23,24,24,25,25,26,
        26,27,28,28,29,29,30,31,31,32,
        32,33,34,34,35,35,36,36,37,37,
        38,39,39,40,40,41,42,43,44,45,
        };
};

