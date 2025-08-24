#include <iostream>             //输入输出流
#include <opencv2/highgui.hpp>  //OpenCV终端部署:gui人机交互界面
#include <opencv2/opencv.hpp>   //OpenCV终端部署:cv处理
#include <ImagePrc.h>
#include <Uart.hpp>
#include <Control.h>
#include <Pid.h>
#include <json_self.h>
#include "SpeedCommand2.cpp"
#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "detection/crosswalk.cpp"   //AI检测：停车区
#include "detection/obstacle.cpp"
#include "detection/catering.cpp"
#include "detection/parking.cpp"
#include "detection/layby.cpp"
#include "detection/bridge.cpp"
#include "./secondCapture.cpp"
#include <signal.h>
#include <unistd.h>
#include <future>
#include <chrono>

using namespace std;
using namespace cv;

const string RECORD_DIR = "/root/code/records/";

// 1 2 
#define MOD 1
#define CAP 0
#define AI  1

int main(void)
{   
    Obstacle obstacle;        // 障碍区检测类
    Parking park;
    Catering cater;
    Layby layby;
    StopArea crosswalk;        // 停车区识别与路径规划类
    Bridge bridge;

    Rect roi(0, 145, 640, 320);
    #if MOD == 1
    // 实例化摄像头
    VideoCapture cap1("/dev/video0");
    #else
    VideoCapture cap1("/root/code/records/1ku.mp4");
    #endif

    #if MOD == 1
    if (!cap1.isOpened())  // 检测摄像头是否正常获取图像
    {
        cout << "video failed" << endl;
        return 0;
    }
    
    // 设置图像分辨率
    cap1.set(CAP_PROP_FRAME_WIDTH, 640);
    cap1.set(CAP_PROP_FRAME_HEIGHT, 480);
    // 显式设置像素格式为 MJPG
    cap1.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap1.set(CAP_PROP_FPS, 120);
          
    
    
    json_self json_;
    json_.json_make();
    
    #if AI == 1
    // 目标检测类(AI模型文件)
    shared_ptr<Detection> detection = make_shared<Detection>(json_.params.model);
    detection->score = json_.params.score; // AI检测置信度
    std::mutex detection_mutex;
    std::future<std::vector<PredictResult>> future_result;
    std::vector<PredictResult> current_results;  // 存储最近的检测结果
    Mat prev_frame;
    std::atomic<bool> processing(false);
    #endif
    
    // USB转串口初始化： /dev/ttyUSB0
    shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
    int ret = uart->open();
    if (ret != 0) {
        printf("[Error] Uart Open failed!\n");
        return -1;
    }
    // uart->startReceive(); // 启动数据接收子线程
    
    // 定义图像保存的矩阵
    Mat current_frame, use_img;
    Mat AIimg;
    long preTime;
    Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
    
    PID_Controller steer;

    bool speedupFlag = false;
    
    // // 舵机PID初始化       
    PID_Init(&steer, json_.params.servo_p, json_.params.servo_p2, 
        json_.params.servo_ring_p1, json_.params.servo_ring_p2, json_.params.servo_ring_p12, json_.params.servo_ring_p22,
        json_.params.servo_d, json_.params.servo_ring_d1, json_.params.servo_ring_d2, json_.params.servo_ring_d12, json_.params.servo_ring_d22);//-7.1 -10.4 -8.5 -21.25
        
        #if AI == 1
        // 首次读取一帧
        Mat initial_frame;
        if (!cap1.read(initial_frame)) return 0;
        initial_frame = initial_frame(roi);
        prev_frame = initial_frame.clone();
        
        // 启动首次异步任务
        future_result = std::async(std::launch::async, [&]() {
            std::lock_guard<std::mutex> lock(detection_mutex);
            detection->inference(prev_frame);
            return detection->results;
        });
        processing = true;
        #endif
        
        #elif MOD == 2
        
        shared_ptr<Detection> detection = make_shared<Detection>("../res/model/yolov3_mobilenet_v1");
        detection->score = 0.5; // AI检测置信度
        
        Mat frame, useFrame;
        Mat Aiimg;
        Mat jpgimg;
        bool pause = false; // 暂停/播放状态
        int frame_count = 0; // 当前帧号
        namedWindow("Video Frame", (640 * 2, 480 * 2));
        
        PID_Controller steer;
        
        SpeedController speedCtrl;
        Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
        
        #endif
        
        #if CAP == 1
        // 录像控制变量
        bool is_recording = false;
        unique_ptr<VideoWriter> videoWriter;
        string current_video_path;
        #endif
        
        #if MOD == 1
        if(json_.params.debug)
        {
            // 创建窗口显示图像
            namedWindow("img", (640, 480));
        }
        #endif
   
            // namedWindow("img", (640, 480));
        while(1)
        {   
            #if MOD == 1 
        // 帧率计算函数
        if(json_.params.debug)
        {
            // preTime = chrono::duration_cast<chrono::milliseconds>(
            //             chrono::system_clock::now().time_since_epoch())
            //             .count();
        }

        // 视频源读取
        if (!cap1.read(current_frame)) continue;
        // second_capture.cv.notify_one();  // 通知等待的线程有新帧到来


        #if AI == 1
        // 检查前一次异步任务是否完成

        if (processing && future_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) 
        {
            current_results = future_result.get();
            processing = false;
        }

        // 若当前无任务运行，启动新异步任务处理上一帧
        if (!processing) 
        {
            prev_frame = current_frame.clone(); // 保存当前帧供异步使用
            prev_frame = prev_frame(roi);
            future_result = std::async(std::launch::async, [&]() 
            {
                std::lock_guard<std::mutex> lock(detection_mutex);
                detection->inference(prev_frame);
                return detection->results;
            });
            processing = true;
        }

        #endif
        
        // 图像预处理
        firstPrcImage(current_frame, use_img, json_.params.threValue);
        imageProcess(0);

        // 停车区检测
        if (json_.params.stop) 
        {
            if (crosswalk.process(current_results)) 
            {
                scene = Scene::StopScene;
                if (crosswalk.countExit > 40) 
                {
                    uart->carControl(0, 0);
                    sleep(1);
                    printf("-----> System Exit!!! <-----\n");
                    exit(0); // 程序退出
                }
            }
        }

        if(stop_flag == true && scene == Scene::NormalScene)
        {
            uart->carControl(0, 0);
            sleep(1);
            printf("-----> System Exit!!! <-----\n");
            exit(0); // 程序退出
        }
        
        // 快餐店检测
        if ((scene == Scene::NormalScene || scene == Scene::CateringScene) && json_.params.catering) 
        {
            if (cater.process(current_results))
                scene = Scene::CateringScene;
            else
                scene = Scene::NormalScene;
        }

        // 充电停车场检测
        if ((scene == Scene::NormalScene || scene == Scene::ParkingScene) && json_.params.parking) 
        {
            if (park.process(current_results))
                scene = Scene::ParkingScene;
            else
                scene = Scene::NormalScene;
        }

        // 临时停车区检测
        if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) && json_.params.layby) 
        {
            if (layby.process(current_results))  // 传入二值化图像进行再处理
                scene = Scene::LaybyScene;
            else
                scene = Scene::NormalScene;
        }

        // 障碍区检测
        if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene) && json_.params.obstacle) 
        {
            if (obstacle.process(current_results)) 
                scene = Scene::ObstacleScene;
            else
                scene = Scene::NormalScene;
        }
        
        // 坡道检测
        if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) && json_.params.bridge) 
        {
            if (bridge.process(current_results)) 
                scene = Scene::BridgeScene;
            else
                scene = Scene::NormalScene;
        }
        
        // 环岛元素识别与路径规划
        if (scene == Scene::NormalScene) 
        {
            
            if(layby.afterStop == true)
            {
                speedupFlag = true;
                // std::cout << "start speed" << std::endl;
            }
            imageProcess2(speedupFlag);
        }

        // 获取偏差值与速度计算
        trackImg(use_img);
        getDet(json_.params.towNormal, use_img);

        // 触发停车
        if ((cater.stopEnable) || (park.step == park.ParkStep::stop) || (layby.stopEnable))
        {
            json_.params.speedNormal = 0;
        }
        else if(park.step == park.ParkStep::gostr)
            json_.params.speedNormal = json_.params.speedLow - 35;
        else if (scene == Scene::LaybyScene)
            json_.params.speedNormal = json_.params.speedLayby;
        else if (scene == Scene::CateringScene)
            json_.params.speedNormal = json_.params.speedCatering;
        else if (scene == Scene::ParkingScene && park.step == park.ParkStep::trackout) // 停车区倒车出库
            json_.params.speedNormal = -json_.params.speedDown;
        else if (scene == Scene::ParkingScene) // 停车区减速
            json_.params.speedNormal = json_.params.speedParking;
        else if (scene == Scene::BridgeScene) // 坡道速度
            json_.params.speedNormal = json_.params.speedBridge;
        else if (scene == Scene::ObstacleScene) // 避障速度
            json_.params.speedNormal = json_.params.speedObstacle;
        else if (scene == Scene::StopScene)
            json_.params.speedNormal = json_.params.speedLow;
        else if (ImageStatus.Road_type == Straight)//正常道路的速度决策放到图像第二次处理里面
            json_.params.speedNormal = json_.params.speedHigh;
        else if((ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque) && firstringFlag == true)//圆环速度
            json_.params.speedNormal = json_.params.speedRing;
        else if((ImageStatus.Road_type == LeftCirque || ImageStatus.Road_type == RightCirque) && firstringFlag == false)//圆环速度
            json_.params.speedNormal = json_.params.speedRing2;
        else if(ImageStatus.Road_type == Normol)//普通速度
            json_.params.speedNormal = json_.params.speedLow;
            
        if(park.turningEable)   // 停车区进库右打角
            uart->carControl(-310, json_.params.speedNormal);
        else if(park.turningBackEable)      // 停车区进库左打角
            uart->carControl(310, json_.params.speedNormal);
        else
            uart->carControl(PID_Compute(&steer), json_.params.speedNormal);

        //[17] 按键退出程序
        // if (uart->keypress) 
        // {
        //     uart->carControl(0, 0); // 控制车辆停止运动
        //     sleep(1);
        //     printf("-----> System Exit!!! <-----\n");
        //     exit(0); // 程序退出
        // }

        // 帧率计算
        if(json_.params.debug)
        {
            imshow("img", use_img);
            waitKey(1);     //1ms
            // auto startTime = chrono::duration_cast<chrono::milliseconds>(
            //     chrono::system_clock::now().time_since_epoch())
            //     .count();
            // printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
            // 1000.0 / (startTime - preTime));
        }
        // 显示必要延时
        
        // 录像功能
        #if CAP == 1

        int key = waitKey(1);
        if(key == 32) // 空格键
        {
            is_recording = !is_recording;
            
            if(is_recording) 
            {
                // 生成带时间戳的文件名
                auto now = chrono::system_clock::now();
                auto timestamp = chrono::duration_cast<chrono::seconds>(
                    now.time_since_epoch()).count();
                current_video_path = RECORD_DIR + "record_" + 
                                    to_string(timestamp) + ".mp4";


                // 获取摄像头参数
                int frame_width = cap1.get(CAP_PROP_FRAME_WIDTH);
                int frame_height = cap1.get(CAP_PROP_FRAME_HEIGHT);
                double fps = cap1.get(CAP_PROP_FPS);
                
                // 初始化视频写入器（使用H264编码）
                videoWriter.reset(new VideoWriter(
                    current_video_path, 
                    VideoWriter::fourcc('a','v','c','1'), // H264编码
                    fps, 
                    Size(frame_width, frame_height)
                ));

                if(!videoWriter->isOpened()) {
                    cerr << "ERROR: 无法创建视频文件" << endl;
                    is_recording = false;
                }
                else {
                    cout << "开始录制: " << current_video_path << endl;
                }
            }
            else 
            {
                if(videoWriter && videoWriter->isOpened()) {
                    videoWriter->release();
                    cout << "视频已保存: " << current_video_path << endl;
                }
            }
        }

        // 录制原始帧
        if(is_recording && videoWriter && videoWriter->isOpened()) {
            videoWriter->write(current_frame);  // 写入原始img
        }

        #endif

        #elif MOD == 2

        if (!pause) {
            cap1 >> frame;       // 读取一帧
            if (frame.empty()) break;
            frame_count++;
            pause = true;       // 自动暂停
        }

        Aiimg = frame.clone();
        Aiimg = Aiimg(roi);

        resize(Aiimg, jpgimg, Size(320, 240), 0, 0, INTER_LINEAR); //压缩图像

        detection->inference(Aiimg);
        detection->drawBox(Aiimg); // 图像绘制AI结果
        imshow("detection", Aiimg);
        
        firstPrcImage(frame, useFrame, 0);
        imageProcess(scene);

        // 快餐店检测
        if ((scene == Scene::NormalScene || scene == Scene::CateringScene)) 
        {
            if (cater.process(detection->results))
                scene = Scene::CateringScene;
            else
                scene = Scene::NormalScene;
        }

        // 充电停车场检测
        if ((scene == Scene::NormalScene || scene == Scene::ParkingScene)) 
        {
            if (park.process(detection->results))
                scene = Scene::ParkingScene;
            else
                scene = Scene::NormalScene;
        }

        // 临时停车区检测
        if ((scene == Scene::NormalScene || scene == Scene::LaybyScene)) 
        {
            if (layby.process(detection->results))  // 传入二值化图像进行再处理
                scene = Scene::LaybyScene;
            else
                scene = Scene::NormalScene;
        }

        // 障碍区检测
        if ((scene == Scene::NormalScene || scene == Scene::ObstacleScene)) 
        {
            if (obstacle.process(detection->results)) 
                scene = Scene::ObstacleScene;
            else
                scene = Scene::NormalScene;
        }

        // 环岛元素识别与路径规划
        if (scene == Scene::NormalScene) 
        {
            imageProcess2(false);
        }

        // std::cout << "scene:" << scene <<std::endl;
        // std::cout << "55" << ImageDeal[55].IsLeftFind << std::endl;
        // std::cout << "56" << ImageDeal[56].IsLeftFind << std::endl;
        // std::cout << "57" << ImageDeal[57].IsLeftFind << std::endl;
        // std::cout << "58" << ImageDeal[58].IsLeftFind << std::endl;
        getDet(30, useFrame);
        trackImg(useFrame);

    
        cv::imshow("Video Frame", useFrame);
        
        int key = cv::waitKey(0); // 无限等待按键
        if (key == 32) {        // 空格键触发下一帧
            pause = false;      // 允许读取下一帧
        } else if (key == 27) { // ESC退出
            break;
        } else if (key == 's' || key == 'S') { // 添加S键保存功能
            if (!frame.empty()) {
                // 生成带毫秒时间戳的文件名
                auto now = chrono::system_clock::now();
                auto timestamp = chrono::duration_cast<chrono::milliseconds>(
                    now.time_since_epoch()).count();
                string filename = RECORD_DIR + "img_" + 
                                 to_string(timestamp) + ".jpg";
                
                // 保存原始帧
                if (imwrite(filename, jpgimg)) {
                    cout << "图像已保存: " << filename << endl;
                } else {
                    cerr << "错误: 图像保存失败" << endl;
                }
            } else {
                cerr << "警告: 当前帧为空" << endl;
            }
        }
        
        #endif
    }
    
    #if CAP == 1
    // 程序退出时确保释放资源
    if(videoWriter && videoWriter->isOpened()) {
        videoWriter->release();
    }
    #endif
    #if MOD == 1
    uart->close(); // 串口通信关闭
    #elif MOD == 2
    destroyAllWindows();
    #endif
    cap1.release();
    return 0;
}

