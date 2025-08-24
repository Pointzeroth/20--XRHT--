
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测

using namespace std;
using namespace cv;

/**
 * @brief 停车区AI识别与路径规划类
 *
 */
class StopArea
{
private:
    /**
     * @brief 场景状态
     *
     */
    enum Step
    {
        init = 0, // 初始化屏蔽
        det,      // AI标识检测
        enable,   // 场景使能
        stop      // 准备停车
    };

    long long startTime;
    long long endTime;
    Step step = Step::init; // 场景状态
    uint16_t countRec = 0;  // AI场景识别计数器
    uint16_t countSes = 0;  // 场次计数器

public:
    uint16_t countExit = 0; // 程序退出计数器
    bool park = false;      // 停车标志
    /**
     * @brief 停车区AI识别与路径规划处理
     *
     * @param predict AI检测结果
     * @return true
     * @return false
     */
    bool process(vector<PredictResult> predict)
    {
        switch (step)
        {
        case Step::init: // 初始化：起点斑马线屏蔽
            countSes++;
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if ((predict[i].y + predict[i].height) > ROWSIMAGE * 0.2) // 标志距离计算 出发改这个
                    {
                        countSes = 0;
                        break;
                    }
                }
            }
            if (countSes > 50)
            {
                countSes = 0;
                step = Step::det;
            }
            break;

        case Step::det: // AI未识别
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if ((predict[i].y + predict[i].height) > ROWSIMAGE * 0.6) // 标志距离计算
                    {
                        countRec++;
                        break;
                    }
                }
            }
            if (countRec) // 识别AI标志后开始场次计数
                countSes++;

            if (countSes >= 8)
            {
                if (countRec >= 2)
                {
                    step = Step::enable;
                }

                countRec = 0;
                countSes = 0;
            }
            break;

        case Step::enable: // 场景使能: 检测斑马线标识丢失
            countSes++;
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
                {
                    if (predict[i].y > ROWSIMAGE * 0.2) // 标志距离计算 停车改这个
                    {
                        countSes = 0;
                        break;
                    }
                }
            }
            if (countSes > 2)
            {
                countExit = 0;
                step = Step::stop;
            }
            break;

        case Step::stop: // 准备停车
            park = true;
            countExit++; // 停车倒计时
            break;
        }

        // 输出场景状态结果
        if (step == Step::init || step == Step::det)
            return false;
        else
            return true;
    }

    /**
     * @brief 图像绘制禁行区识别结果
     *
     * @param img 需要叠加显示的图像
     */
    void drawImage(Mat &img)
    {
        if (step == Step::enable)
            putText(img, "[1] PARK - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }
};