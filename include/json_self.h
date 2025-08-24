#ifndef JSON_H
#define JSON_H
#include "ImagePrc.h"
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include "../include/json.hpp"
#include <json_self.h>

using namespace std;
    
    
class json_self
{
public:
    int countShift = 0; // 变速计数器

    /**
     * @brief 初始化：加载配置文件
     *
     */
    void json_make()
    {
        string jsonPath = "../config/config.json";
        std::ifstream config_is(jsonPath);
        if (!config_is.good())
        {
            std::cout << "Error: Params file path:[" << jsonPath<< "] not find .\n";
            exit(-1);
        }

        nlohmann::json js_value;
        config_is >> js_value;

        try
        {
            params = js_value.get<Params>();
        }
        catch (const nlohmann::detail::exception &e)
        {
            std::cerr << "Json Params Parse failed :" << e.what() << '\n';
            exit(-1);
        }

        speed = params.speedLow;
        cout << "--- speedLow:" << params.speedLow << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
    };

    /**
     * @brief 控制器核心参数
     *
     */
    struct Params
    {
        float servo_p = 0.0;
        float servo_p2 = 0.0;
        float servo_ring_p1 = 0.0;
        float servo_ring_p2 = 0.0;
        float servo_ring_p12 = 0.0;
        float servo_ring_p22 = 0.0;

        float servo_d = 0.0;
        float servo_ring_d1 = 0.0;
        float servo_ring_d2 = 0.0;
        float servo_ring_d12 = 0.0;
        float servo_ring_d22 = 0.0;

        int   towNormal = 0;
        int   threValue = 0;
        float speedLow = 0.0;                              // 智能车最低速
        float speedHigh = 0.0;                             // 智能车最高速
        
        float speedNormal = 0.0;
        float speedBridge = 0.6;    // 坡道速度
        float speedCatering = 0.6;  // 快餐店速度0.6
        float speedLayby = 0.6;     // 临时停车区速度
        float speedObstacle = 0.6;  // 障碍区速度
        float speedParking = 0.6;   // 停车场速度
        float speedRing = 0.6;      // 环岛速度
        float speedRing2 = 0.6;      // 环岛速度
        float speedDown = 0.5;      // 特殊区域降速速度

        bool debug = false;                                // 调试模式使能
        bool saveImg = false;                              // 存图使能

        bool bridge = true;         // 坡道区使能
        bool catering = true;       // 快餐店使能
        bool layby = true;          // 临时停车区使能
        bool obstacle = true;       // 障碍区使能
        bool parking = true;        // 停车场使能
        bool ring = true;           // 环岛使能
        bool cross = true;          // 十字道路使能
        bool stop = true;           // 停车区使能

        float score = 0.5;                                 // AI检测置信度
        string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
        string video = "../res/samples/demo.mp4";          // 视频路径
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, servo_p, servo_p2, servo_ring_p1, servo_ring_p2, servo_ring_p12, servo_ring_p22, servo_d, servo_ring_d1, servo_ring_d12, servo_ring_d2, servo_ring_d22, towNormal, threValue, speedLow, speedHigh, speedNormal, speedBridge, speedCatering, speedLayby, 
            speedObstacle, speedParking, speedRing, speedRing2, speedDown, debug, saveImg, bridge, catering,
            layby, obstacle, parking, ring, cross, stop, score, model, video); // 添加构造函数
    };

    Params params;                  // 读取控制参数
    float speed = 40;               // 发送给电机的速度
};
    
#endif
