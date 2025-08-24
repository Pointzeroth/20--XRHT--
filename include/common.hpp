#pragma once

#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署

using namespace std;
using namespace cv;

#define COLSIMAGE 640    // 图像的列数
#define ROWSIMAGE 320    // 图像的行数

#define LABEL_BATTERY 0    // AI标签：充电站
#define LABEL_BLOCK 1      // AI标签：障碍物
#define LABEL_BRIDGE 2     // AI标签：坡道
#define LABEL_BURGER 3     // AI标签：汉堡
#define LABEL_CAR 4        // AI标签：道具车
#define LABEL_COMPANY 5    // AI标签：公司
#define LABEL_CONE 6       // AI标签：锥桶
#define LABEL_CROSSWALK 7  // AI标签：斑马线
#define LABEL_PEDESTRIAN 8 // AI标签：行人
#define LABEL_SCHOOL 9     // AI标签：学校

/**
 * @brief 场景类型（路况）
 *
 */
enum Scene
{
    NormalScene = 0, // 基础赛道
    CrossScene,      // 十字道路
    RingScene,       // 环岛道路
    BridgeScene,     // 坡道区
    ObstacleScene,   // 障碍区
    CateringScene,   // 快餐店
    LaybyScene,      // 临时停车区
    ParkingScene,    // 停车区
    StopScene        // 停车（结束）
};

/**
 * @brief Get the Scene object
 *
 * @param scene
 * @return string
 */
string getScene(Scene scene)
{
    switch (scene)
    {
    case Scene::NormalScene:
        return "Normal";
    case Scene::CrossScene:
        return "Crossroad";
    case Scene::RingScene:
        return "Ring";
    case Scene::BridgeScene:
        return "Bridge";
    case Scene::ObstacleScene:
        return "Obstacle";
    case Scene::CateringScene:
        return "Catering";
    case Scene::LaybyScene:
        return "Layby";
    case Scene::ParkingScene:
        return "Parking";
    case Scene::StopScene:
        return "Stop";
    default:
        return "Error";
    }
}
