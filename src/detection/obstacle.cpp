#pragma once

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测
#include "../../include/ImagePrc.h"

using namespace std;
using namespace cv;

//extern ImageStatustypedef ImageStatus;

/**
 * @brief 障碍区AI识别与路径规划类
 *
 */
class Obstacle
{

public:
    /**
     * @brief 障碍区AI识别与路径规划处理
     *
     * @param track 赛道识别结果
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict)//block
    {
        vector<PredictResult> resultsObs; // 锥桶AI检测数据
        for (size_t i = 0; i < predict.size(); i++)
        {
            int xtemp1 = predict[i].x / 5;
            int ytemp1 = ( predict[i].y + predict[i].height )/ 5;
            int widthtemp1 = predict[i].width / 5;
            int heighttemp1 = predict[i].height / 5;
            // bool leftline_flag = calculate_woline(true);//从offline+1到56行得线左丢边大于2，返回true
            // bool rightline_flag = calculate_woline(false);//同上右丢边大于2，返回true

            if (  ImageStatus.OFFLine < 30 && (predict[i].type == LABEL_CONE || predict[i].type == LABEL_PEDESTRIAN || predict[i].type == LABEL_BLOCK )&& (predict[i].y + predict[i].height) > 0 && predict[i].score > 0.7 && ImageFlag.image_element_cross == 0 && ImageFlag.image_element_rings == 0) // 锥桶识别
            {
                if(ytemp1 > 56) ytemp1 = 56;
                if( xtemp1 > ImageDeal[ytemp1 + 2].LeftBorder - 10 &&
                    ImageDeal[ytemp1 + 2].RightBorder + 3 > xtemp1 + widthtemp1 
                    // ImageDeal[ytemp + 2].LeftBorder > 15
                  )//障碍物靠左
                {
                    if(predict[i].type == LABEL_CONE || predict[i].type == LABEL_PEDESTRIAN )resultsObs.push_back(predict[i]);
                    else if(predict[i].type == LABEL_BLOCK){
                        if(straightJudge(2,32,50) >= 1)return false;
                        else resultsObs.push_back(predict[i]);
                    }
                    
                }
                else if(xtemp1 + widthtemp < ImageDeal[ytemp1 + 2].RightBorder + 10 &&
                        ImageDeal[ytemp1 + 2].LeftBorder - 3 < xtemp1 
                        // ImageDeal[ytemp1 + 2].RightBorder < 105 
                       )//障碍物靠右
                {
                    if(predict[i].type == LABEL_CONE || predict[i].type == LABEL_PEDESTRIAN )resultsObs.push_back(predict[i]);
                    else if(predict[i].type == LABEL_BLOCK){
                        if(straightJudge(1,32,50) >= 1)return false;
                        else resultsObs.push_back(predict[i]);
                    }                
                }
            }
        }
        if (resultsObs.size() <= 0 && Obs_enable == false)
        {
            return false;
        }
        else if((resultsObs.size() > 0 && Obs_enable == false) || (resultsObs.size() > 0 && Obs_enable == true))
        {
            // 选取距离最近的锥桶
            for (size_t i = 0; i < resultsObs.size(); i++) {
                maxY = 0;
                int y_bottom = resultsObs[i].y ;
                if (y_bottom >= maxY) {
                    index = i;
                    maxY = y_bottom;
                }
            }
            resultObs = resultsObs[index];
            
            //模型的左下角
            xtemp = resultsObs[index].x / 5;
            ytemp = ( resultsObs[index].y + resultsObs[index].height )/ 5;
            widthtemp = resultsObs[index].width / 5;
            heighttemp = resultsObs[index].height / 5;

            if(ytemp > 56) ytemp = 56; 
            //左右距离
            disLeft = xtemp - ImageDeal[ytemp].LeftBorder;
            disRight = ImageDeal[ytemp].RightBorder - xtemp - widthtemp;
            
            if(ytemp <= 56 && ytemp > 18){
                // inside = false;      //赛道内满足条件的标志位
                update_flag = false;//防止近处没检测到障碍物，检测到远处障碍物不延时
                //判断障碍物左右方位
                if (xtemp > ImageDeal[ytemp + 2].LeftBorder - 10 &&
                    ImageDeal[ytemp + 2].RightBorder + 3 > xtemp + widthtemp &&
                    // ImageDeal[ytemp + 2].LeftBorder > 15 &&
                    disLeft < disRight 
                    ) //[1] 障碍物靠左
                {
                    // inside = true;
                    update_flag = true;
                    obstacle_left = true; //记录当前状态
                    Obs_enable = true;
                    countsession = 25;
                }
                if (xtemp + widthtemp < ImageDeal[ytemp + 2].RightBorder + 10 &&
                    ImageDeal[ytemp + 2].LeftBorder - 3 < xtemp &&
                    // ImageDeal[ytemp + 2].RightBorder < 105 &&
                    disLeft > disRight
                    ) //[2] 障碍物靠右
                {
                    // inside = true;
                    update_flag = true;
                    obstacle_left = false; //记录当前状态
                    Obs_enable = true;
                    countsession = 25;
                }
            }
        } 


        // 场景检测使能标志
        // Obs_enable = true; 

        // std::cout << "ytemp" << ytemp << std::endl;
        // std::cout << "Obs_enable" << Obs_enable << std::endl;
        // std::cout << "update_flag" << update_flag << std::endl;
        // std::cout << "inside" << inside << std::endl;
        if((resultsObs.size() <= 0 && Obs_enable == true) || update_flag)
        {
            //不同方向障碍的处理
            if( countsession > 0)
            {
                countsession--;
                // std::cout << "剩下的帧数" << countsession << std::endl;
                // Last_Mode = obstacle_left;
                if (obstacle_left == false)  curtailTracking(true, -0.12*disLeft); // 第二个数值越小，拐的力度越大       （行人,锥桶）
                if (obstacle_left == true )   curtailTracking(false, -0.12*disRight); // 第二个数值越小，拐的力度越大       （行人,锥桶）
                std::cout << "锥桶" << std::endl;
            }
            if(countsession == 0)
            {
                Obs_enable = false;
                obstacle_left = false;
                update_flag = false;
                // inside = false;      //赛道内满足条件的标志位
                count = 0;           // 检测一张图像里面障碍元素的个数，从而分出不同的方案
                maxY = 0;
                index = 0;
                xtemp = 0;
                ytemp = 0;
                widthtemp = 0;
                heighttemp = 0;
                countsession = 0;
                disLeft = 0;
                disRight = 0;
                return false;
            }
        }
        return true;
    }

private:
    bool Obs_enable = false; // 场景检测使能标志
    bool obstacle_left = false;
    bool update_flag = false;  //保持赛道标志位
    // bool inside = false;      //赛道内满足条件的标志位

    int count = 0;           // 检测一张图像里面障碍元素的个数，从而分出不同的方案
    int maxY = 0;
    int index = 0;
    int xtemp = 0;
    int ytemp = 0;
    int widthtemp = 0;
    int heighttemp = 0;
    int countsession =0;
    int disLeft = 0;
    int disRight = 0;
    PredictResult resultObs; // 避障目标锥桶

    /**
     * @brief 缩减优化车道线（双车道→单车道）
     *
     * @param track
     * @param left
     */
    void curtailTracking(bool left , float distance)
    {
        if (left) // 向左侧缩进
        {
            for (size_t i = 57; i > ImageStatus.OFFLine; i--)
            {
                ImageDeal[i].Center = ImageDeal[i].LeftBorder + distance;
                if(ImageDeal[i].Center < 0)
                    ImageDeal[i].Center = 0;
                if(ImageDeal[i].Center > LCDW - 1)
                    ImageDeal[i].Center = LCDW - 1;
                ImageDeal[i].RightBorder = 2*ImageDeal[i].Center - ImageDeal[i].LeftBorder;
            }
        }
        else // 向右侧缩进
        {
            for (size_t i = 57; i > ImageStatus.OFFLine; i--)
            {
                ImageDeal[i].Center = ImageDeal[i].RightBorder - distance;
                if(ImageDeal[i].Center < 0)
                    ImageDeal[i].Center = 0;
                if(ImageDeal[i].Center > LCDW - 1)
                    ImageDeal[i].Center = LCDW - 1;
                ImageDeal[i].LeftBorder = 2*ImageDeal[i].Center - ImageDeal[i].RightBorder;
            }
        }
    }
    bool calculate_woline(bool flag)
    {
        int count = 0;
        if(flag == true){
            for(int i = ImageStatus.OFFLine + 5 ; i <= 56 ; i ++ ){
                if(ImageDeal[i].IsLeftFind == 'W'){
                    count ++;
                }
            }
        }
        else{
            for(int i = ImageStatus.OFFLine + 5 ; i <= 56 ; i ++ ){
                if(ImageDeal[i].IsRightFind == 'W'){
                    count ++;
                }
            }
        }
        if(count > 3)return true;
        else return false;
    }
};