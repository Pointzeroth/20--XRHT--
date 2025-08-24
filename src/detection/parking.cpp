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

#define CAR 1

extern uint8_t Pixle[LCDH][LCDW];

class Parking
{
public:

    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        enable,   // 停车场使能
        turning,  // 入库转向
        stop,     // 停车
        trackout, // 出库
        gostr     //
    };
    
    ParkStep step = ParkStep::none; // 停车步骤
    bool turningEable = false;  //转弯使能
    bool turningBackEable = false;  //转弯后退使能
    bool renFlag = false;
    bool afterStop = false;

    bool process(vector<PredictResult> predict)
    {
        switch (step)
        {
            case ParkStep::none: // AI未识别
            {
                if(ImageStatus.OFFLine >= 5)
                    return false;

                for (size_t i = 0; i < predict.size(); i++)
                {
                    if ((predict[i].type == LABEL_BATTERY) && predict[i].score >= 0.55 && ImageFlag.image_element_cross == 0 && ImageFlag.image_element_rings == 0)
                    {
                        counterRec++;
                        batteryX = predict[i].x / 5;
                        batteryY = ( predict[i].y + predict[i].height ) / 5;

                        if ( predict[i].x > COLSIMAGE / 2 && ImageDeal[(predict[i].y + predict[i].height) / 5].Center < predict[i].x / 5)   // 标识牌在右侧
                            rightEnable = true; // 直接改左右
                        else if( predict[i].x < COLSIMAGE / 2 && ImageDeal[(predict[i].y + predict[i].height) / 5].Center > predict[i].x / 5)
                            rightEnable = false;   // 直接改左右
                        break;
                    }
                }
                if (counterRec) // 检测到一帧后开始连续监测AI标志是否满足条件
                {
                    counterSession++;

                    if(rightEnable)
                    {
                        if(straightJudge(1, ImageStatus.OFFLine + 5, 45) > 1)
                            return false;
                    }
                    else
                    {
                        if(straightJudge(2, ImageStatus.OFFLine + 5, 45) > 1)
                            return false;
                    }

                    for (Ysite = 40; Ysite > ImageStatus.OFFLine + 1; Ysite--)
                    {
                        if ((ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary_First > 4) &&
                            (ImageDeal[Ysite - 1].RightBoundary - ImageDeal[Ysite + 1].RightBoundary_First > 4) && (rightEnable == true))
                        {
                            renFlag = true;
                            break;
                        }

                        if((ImageDeal[Ysite + 1].LeftBoundary_First - ImageDeal[Ysite].LeftBoundary > 4) &&
                            (ImageDeal[Ysite + 1].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary > 4) && (rightEnable == false))
                        {
                            renFlag = true;
                            break;
                        }
                    }
                    
                    if ((counterRec >= 1 && counterSession < 8) && renFlag)
                    {
                        counterRec = 0;
                        counterSession = 0;
                        step = ParkStep::enable; // 检测到停车场标志
                        renFlag = false;
                        std::cout << "识别牌子进入充电区" << std::endl;
                        return true;
                    }
                    else if (counterSession >= 8)
                    {
                        counterRec = 0;
                        counterSession = 0;
                        renFlag = false;
                    }
                }
                return false;
                break;
            }
            case ParkStep::enable: // 停车场使能
            {   
                carY = 0;
                int leftnum = 0;
                int rightnum = 0;
                int pointguai = 0;
                int whiteLine1 = 0;
                int whiteLine2 = 0;
                int whiteLine3 = 0;

                #if CAR == 4

                
                for (Ysite = 35; Ysite > ImageStatus.OFFLine + 1; Ysite--)
                {
                    if ((ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4) && (rightEnable == true))
                    {
                        pointguai = Ysite;
                        break;
                    }
                    
                    if((ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4) && (rightEnable == false))
                    {
                        pointguai = Ysite;
                        break;
                    }
                }
                
                for(int Y = pointguai; Y >= ImageStatus.OFFLine; Y --)
                {
                    if(ImageDeal[Y].IsLeftFind == 'W')
                    {
                        leftnum ++;
                    }
                    if(ImageDeal[Y].IsRightFind == 'W')
                    {
                        rightnum ++;
                    }
                }

                // std::cout << "left:" << leftnum << std::endl;
                // std::cout << "right:" << rightnum << std::endl;

                if(pointguai >= 10 && rightEnable)
                {
                    for(int x = ImageDeal[pointguai - 1].LeftBorder + 1; x < LCDW - 2; x ++)
                    {
                        if(Pixle[pointguai - 1][x] == 0 && Pixle[pointguai - 1][x + 1] == 0)
                            break;
                        else
                            whiteLine1 ++;
                    }

                    for(int x = ImageDeal[pointguai - 3].LeftBorder + 1; x < LCDW - 1; x ++)
                    {
                        if(Pixle[pointguai - 3][x] == 0 && Pixle[pointguai - 3][x + 1] == 0)
                            break;
                        else
                            whiteLine2 ++;
                    }

                    for(int x = ImageDeal[pointguai - 5].LeftBorder + 1; x < LCDW - 1; x ++)
                    {
                        if(Pixle[pointguai - 5][x] == 0 && Pixle[pointguai - 5][x + 1] == 0)
                            break;
                        else
                            whiteLine3 ++;
                    }
                }
                else if(pointguai >= 10 && !rightEnable)
                {
                    for(int x = ImageDeal[pointguai - 1].RightBorder - 1; x > 1; x --)
                    {
                        if(Pixle[pointguai - 1][x] == 0 && Pixle[pointguai - 1][x - 1] == 0)
                            break;
                        else
                            whiteLine1 ++;
                    }

                    for(int x = ImageDeal[pointguai - 3].RightBorder - 1; x > 1; x --)
                    {
                        if(Pixle[pointguai - 3][x] == 0 && Pixle[pointguai - 3][x - 1] == 0)
                            break;
                        else
                            whiteLine2 ++;
                    }

                    for(int x = ImageDeal[pointguai - 5].RightBorder - 1; x > 1; x --)
                    {
                        if(Pixle[pointguai - 5][x] == 0 && Pixle[pointguai - 5][x - 1] == 0)
                            break;
                        else
                            whiteLine3 ++;
                    }
                }

                // std::cout << "whiteLine1:" << whiteLine1 << std::endl;
                // std::cout << "whiteLine2:" << whiteLine2 << std::endl;
                // std::cout << "whiteLine3:" << whiteLine3 << std::endl;

                #endif

                for(size_t i = 0; i < predict.size(); i++)
                {
                    if (predict[i].type == LABEL_CAR)
                    {  
                        
                        carX = predict[i].x / 5;
                        carY = ( predict[i].y + predict[i].height ) / 5;
                    }
                    if (predict[i].type == LABEL_BATTERY)
                    {
                        batteryX = predict[i].x / 5;
                        batteryY = ( predict[i].y + predict[i].height ) / 5;
                    }
                }

                if(batteryY >= 12 && batteryY <= 58)
                {
                    #if CAR == 4

                    if((rightnum >= 4) && (abs(whiteLine1 - whiteLine2) <= 5) && (abs(whiteLine1 - whiteLine3) <= 5) && rightEnable)
                    {
                        garageFirst = true;       // 进入二号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "1号车库" << std::endl;
                    }
                    else if((rightnum <= 8) && ((abs(whiteLine1 - whiteLine2) >= 6) || (abs(whiteLine1 - whiteLine3) >= 6)) && rightEnable)
                    {
                        garageFirst = false;        // 进入一号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "2号车库" << std::endl;
                    }
                    else if((leftnum >= 4) && (abs(whiteLine1 - whiteLine2) <= 5) && (abs(whiteLine1 - whiteLine3) <= 5) && !rightEnable)
                    {
                        garageFirst = true;       // 进入二号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "1号车库" << std::endl;
                    }
                    else if((leftnum <= 8) && ((abs(whiteLine1 - whiteLine2) >= 6) || (abs(whiteLine1 - whiteLine3) >= 6)) && !rightEnable)
                    {
                        garageFirst = false;        // 进入一号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "2号车库" << std::endl;
                    }

                    #elif CAR == 3
                    carDis = batteryY - carY;
                    std::cout << "batteryY:" << batteryY << std::endl;
                    std::cout << "carY:" << carY << std::endl;
                    std::cout << "carDis:" << carDis << std::endl;
                    if(carDis <= 17 && carY != 0)
                    {
                        garageFirst = false;       // 进入二号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "2号车库" << std::endl;
                    }
                    else
                    {
                        garageFirst = true;        // 进入一号车库
                        step =  ParkStep::turning; // 开始入库
                        counterSession = 0;
                        std::cout << "1号车库" << std::endl;
                    }
                    #elif CAR == 1
                    garageFirst = true;        // 进入一号车库
                    step =  ParkStep::turning; // 开始入库
                    counterSession = 0;
                    std::cout << "1号车库" << std::endl;
                    #elif CAR == 2
                    garageFirst = false;       // 进入二号车库
                    step =  ParkStep::turning; // 开始入库
                    counterSession = 0;
                    std::cout << "2号车库" << std::endl;
                    #endif

                }

                return true;
                break;
            }

            case ParkStep::turning: // 入库转向
            {
                // 图像处理
                int turnX = 0;
                int turnY = 0;
                whiteColumnRight[0] = 0;
                whiteColumnRight[1] = 0;
                whiteRowLeft[0] = 0;
                whiteRowLeft[1] = 0;

                for(int i = 0; i < LCDW; i ++)
                {
                    whiteColumn[i] = 0;
                }

                for(int i = 0; i < LCDH; i ++)
                {
                    whiteRow[i] = 0;
                }

                if(garageFirst && rightEnable)
                {
                    if(ImageDeal[57].IsRightFind == 'W' &&
                       ImageDeal[56].IsRightFind == 'W' && whiteflag == 0)
                    {
                        turningEable = true;
                        std::cout << "开始右打角1库" << std::endl;
                        whiteflag = 1;
                    }
                    else if(whiteflag == 0)     //补半宽
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + halfRoadWide[Ysite];
                        }
                    }
                    else if(whiteflag == 1)
                    {
                        //遍历全图记录范围内的每一列白点数量
                        for (int j = LCDW - 1; j > 0; j --)
                        {
                            for (int i = LCDH - 2; i > 0; i--)
                            {
                                if(Pixle[i][j] == 0)
                                    break;
                                else
                                    whiteColumn[j]++;
                            }
                        }
    
                        // 从右到左找最长白列
                        for(int i = LCDW - 5; i > 5; i --)//从右往左，注意条件，找到左边最长白列位置就可以停了
                        {
                            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
                            {
                                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                                whiteColumnRight[1] = i;              //【1】是下标，第j列
                            }
        
                            if(whiteColumn[i] < whiteColumnRight[0] && whiteColumn[i - 1] < whiteColumnRight[0] && whiteColumn[i - 2] < whiteColumnRight[0])
                            {
                                center = i;
                                // std::cout << "center:" << i << std::endl;
                                break;
                            }
                        }
    
                        if(center <= 68 && ImageStatus.OFFLine >= 20)
                        {
                            step = ParkStep::stop;
                            turnY = 0;
                            turnX = 0;
                            whiteflag = 0;
                            startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                        }
                    }
                }
                else if(!garageFirst && rightEnable)
                {
                    if(carFlag == false)
                    {
                        for(int Y = LCDH - 2; Y > 0; Y --)
                        {
                            for(int X = ImageDeal[Y].LeftBorder + 1 ; X < LCDW - 2; X ++)
                            {
                                if(Pixle[Y][X] == 0 && Pixle[Y][X + 1] == 0)
                                    break;
                                else
                                    whiteRow[Y]++;
                            }
                            if(whiteRow[Y] - whiteRow[Y + 1] >= 18 && whiteRow[Y] - whiteRow[Y + 2] >= 18 && Y < 55 && Y > 5)
                            {
                                whiteRowLeft[0] = whiteRow[Y];
                                whiteRowLeft[1] = Y;
                                break;
                            }
                        }

                        if(whiteRowLeft[1] >= 40)
                            carFlag = true;
                    }

                    if(carFlag == true && whiteflag == 0)
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + halfRoadWide[Ysite];
                        }

                        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 35; Ysite ++)
                        {
                            if ((ImageDeal[Ysite + 1].RightBoundary - ImageDeal[Ysite].RightBoundary >= 12) &&
                                (ImageDeal[Ysite + 2].RightBoundary - ImageDeal[Ysite].RightBoundary >= 12))
                            {
                                turnX = ImageDeal[Ysite].RightBoundary;
                                turnY = Ysite;
                                // std::cout << "turnY:" << turnY << std::endl;
                                break;
                            }
                        }

                        if(turnY >= 27) //22
                        {
                            turningEable = true;
                            std::cout << "开始右打角" << std::endl;
                            whiteflag = 1;
                            carFlag = false;
                        }
                    }
                    else if(whiteflag == 0)     //补半宽
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + halfRoadWide[Ysite];
                        }
                    }
                    else if(whiteflag == 1)
                    {
                        //遍历全图记录范围内的每一列白点数量
                        for (int j = LCDW - 1; j > 0; j --)
                        {
                            for (int i = LCDH - 2; i > 0; i--)
                            {
                                if(Pixle[i][j] == 0)
                                    break;
                                else
                                    whiteColumn[j]++;
                            }
                        }
    
                        // 从右到左找最长白列
                        for(int i = LCDW - 5; i > 5; i --)//从右往左，注意条件，找到左边最长白列位置就可以停了
                        {
                            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
                            {
                                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                                whiteColumnRight[1] = i;              //【1】是下标，第j列
                            }
        
                            if(whiteColumn[i] < whiteColumnRight[0] && whiteColumn[i - 1] < whiteColumnRight[0] && whiteColumn[i - 2] < whiteColumnRight[0])
                            {
                                center = i;
                                // std::cout << "center:" << i << std::endl;
                                break;
                            }
                        }
    
                        if(center <= 60 && ImageStatus.OFFLine >= 20)
                        {
                            step = ParkStep::stop;
                            turnY = 0;
                            turnX = 0;
                            whiteflag = 0;
                            startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                        }
                    }
                }
                else if(garageFirst && !rightEnable)
                {
                    if(ImageDeal[53].IsLeftFind == 'W' && ImageDeal[54].IsLeftFind == 'W' && whiteflag == 0)
                    {
                        turningBackEable = true;
                        std::cout << "开始左打角" << std::endl;
                        whiteflag = 1;
                    }
                    else if(whiteflag == 0)     //补半宽
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - halfRoadWide[Ysite];
                        }
                    }
                    else if(whiteflag == 1)
                    {
                        //遍历全图记录范围内的每一列白点数量
                        for (int j = LCDW - 1; j > 0; j --)
                        {
                            for (int i = LCDH - 2; i > 0; i--)
                            {
                                if(Pixle[i][j] == 0)
                                    break;
                                else
                                    whiteColumn[j]++;
                            }
                        }
    
                        // 从右到左找最长白列
                        for(int i = 5; i < LCDW - 5; i ++)
                        {
                            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
                            {
                                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                                whiteColumnRight[1] = i;              //【1】是下标，第j列
                            }
        
                            if(whiteColumn[i] < whiteColumnRight[0] && whiteColumn[i + 1] < whiteColumnRight[0] && whiteColumn[i + 2] < whiteColumnRight[0])
                            {
                                center = i;
                                // std::cout << "center:" << i << std::endl;
                                break;
                            }
                        }
    
                        if(center >= 50 && ImageStatus.OFFLine >= 20)
                        {
                            step = ParkStep::stop;
                            turnY = 0;
                            turnX = 0;
                            whiteflag = 0;
                            startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                        }
                    }
                }
                else if(!garageFirst && !rightEnable)
                {
                    if(carFlag == false)
                    {
                        for(int Y = LCDH - 2; Y > 0; Y --)
                        {
                            for(int X = ImageDeal[Y].RightBorder - 1 ; X > 1; X --)
                            {
                                if(Pixle[Y][X] == 0 && Pixle[Y][X - 1] == 0)
                                    break;
                                else
                                    whiteRow[Y]++;
                            }
                            if(whiteRow[Y] - whiteRow[Y + 1] >= 18 && whiteRow[Y] - whiteRow[Y + 2] >= 18 && Y < 55 && Y > 5)
                            {
                                whiteRowLeft[0] = whiteRow[Y];
                                whiteRowLeft[1] = Y;
                                break;
                            }
                        }

                        if(whiteRowLeft[1] > 40)
                            carFlag = true;
                    }

                    if(carFlag == true && whiteflag == 0)
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - halfRoadWide[Ysite];
                        }

                        for (Ysite = ImageStatus.OFFLine + 1; Ysite < 35; Ysite ++)
                        {
                            if ((ImageDeal[Ysite].LeftBoundary - ImageDeal[Ysite + 1].LeftBoundary > 12) &&
                                (ImageDeal[Ysite].LeftBoundary - ImageDeal[Ysite + 2].LeftBoundary > 12))
                            {
                                turnX = ImageDeal[Ysite].LeftBoundary;
                                turnY = Ysite;
                                // std::cout << "turnY:" << turnY << std::endl;
                                break;
                            }
                        }

                        if(turnY >= 30) //22
                        {
                            turningBackEable = true;
                            std::cout << "开始左打角" << std::endl;
                            whiteflag = 1;
                            carFlag = false;
                        }
                    }
                    else if(whiteflag == 0)     //补半宽
                    {
                        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                        {
                            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - halfRoadWide[Ysite];
                        }
                    }
                    else if(whiteflag == 1)
                    {
                        //遍历全图记录范围内的每一列白点数量
                        for (int j = LCDW - 1; j > 0; j --)
                        {
                            for (int i = LCDH - 2; i > 0; i--)
                            {
                                if(Pixle[i][j] == 0)
                                    break;
                                else
                                    whiteColumn[j]++;
                            }
                        }
    
                        // 从右到左找最长白列
                        for(int i = 5; i < LCDW - 5; i ++)
                        {
                            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
                            {
                                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                                whiteColumnRight[1] = i;              //【1】是下标，第j列
                            }
        
                            if(whiteColumn[i] < whiteColumnRight[0] && whiteColumn[i + 1] < whiteColumnRight[0] && whiteColumn[i + 2] < whiteColumnRight[0])
                            {
                                center = i;
                                // std::cout << "center:" << i << std::endl;
                                break;
                            }
                        }
    
                        if(center >= 60 && ImageStatus.OFFLine >= 20)
                        {
                            step = ParkStep::stop;
                            turnY = 0;
                            turnX = 0;
                            whiteflag = 0;
                            startTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();//start
                        }
                    }
                }

                return true;
                break;
            }
            case ParkStep::stop: // 停车
            {
                endTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                if((endTime-startTime) >= 300)
                {
                    step =  ParkStep::trackout; // 开始倒车
                    std::cout << "开始倒车" << std::endl;
                }
                return true;
                break;
            }

            case ParkStep:: trackout: // 出库这段标志位给－速度
            {
                if ((straightJudge(1, 25, 45) < 1) && (ImageStatus.OFFLine < 10) && rightEnable && ImageStatus.LWLine <= 4)    //右边为直线且截止行（前瞻值）很小
                {
                    turningEable = false;
                    step = ParkStep::gostr;
                    goTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                }
                else if((straightJudge(2, 25, 45) < 1) && (ImageStatus.OFFLine < 10) && (!rightEnable) && ImageStatus.RWLine <= 4)
                {
                    turningBackEable = false;
                    step = ParkStep::gostr;
                    goTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
                }
                return true;
                break;
            }

            case ParkStep::gostr:
            {
                turningEable = false;
                turningBackEable = false;

                if(rightEnable)
                {
                    for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + halfRoadWide[Ysite];
                    }
    
                    // if(ImageStatus.RWLine <= 8 && ImageStatus.OFFLine <= 15 && (straightJudge(2, ImageStatus.OFFLine + 8, 28) < 1))
                    // {
                    //     step = ParkStep::none;
                    //     batteryX = 0;
                    //     batteryY = 0;
                    //     carY = 0;
                    //     carX = 0;
                    //     afterStop = true;
                    //     std::cout << "退出停车场" << std::endl;
                    //     return false;
                    // }

                    goendTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

                    if(goendTime - goTime >= 500)
                    {
                        step = ParkStep::none;
                        batteryX = 0;
                        batteryY = 0;
                        carY = 0;
                        carX = 0;
                        afterStop = true;
                        std::cout << "退出停车场" << std::endl;
                        return false;
                    }
                }
                else
                {
                    for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - halfRoadWide[Ysite];
                    }

                    goendTime = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

                   if(goendTime - goTime >= 500)
                    {
                        step = ParkStep::none;
                        batteryX = 0;
                        batteryY = 0;
                        carY = 0;
                        carX = 0;
                        afterStop = true;
                        std::cout << "退出停车场" << std::endl;
                        return false;
                    }                    
                    // if(ImageStatus.LWLine <= 8 && ImageStatus.OFFLine <= 15 && (straightJudge(1, ImageStatus.OFFLine + 8, 30) < 1))
                    // {
                    //     step = ParkStep::none;
                    //     batteryX = 0;
                    //     batteryY = 0;
                    //     carY = 0;
                    //     carX = 0;
                    //     afterStop = true;
                    //     std::cout << "退出停车场" << std::endl;
                    //     return false;
                    // }
                }

                return true;
                break;
            }
        }
        return true;
    }



private:
    //图像处理
    uint8_t halfRoadWide[60]=
    {      
    10,10,10,11,12,12,13,13,14,15,
    15,16,16,17,17,18,18,19,20,20,
    21,21,22,23,23,24,24,25,25,26,
    26,27,28,28,29,29,30,31,31,32,
    32,33,34,34,35,35,36,36,37,37,
    38,39,39,40,40,41,42,43,44,45,
    };

    int carY = 0;
    int carX = 0;
    int batteryY = 0;
    int batteryX = 0;
    int carDis = 0;

    int whiteColumn[LCDW];//每列白列长度
    uint8_t whiteColumnRight[2];
    uint8_t whiteflag = 0;

    int whiteRow[LCDH];//每行白列长度
    uint8_t whiteRowLeft[2];
    
    bool carFlag = false;
    int Ysite = 0;
    int center = 0;

    long long startTime;
    long long endTime;

    long long goTime;
    long long goendTime;

    uint16_t counterSession = 0;  // 图像场次计数器
    uint16_t counterRec = 0;      // 加油站标志检测计数器
    bool garageFirst = true;      // 进入一号车库
    bool rightEnable = true;
};