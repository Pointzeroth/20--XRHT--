#pragma once
#include <iostream>             //输入输出流
#include <opencv2/highgui.hpp>  //OpenCV终端部署:gui人机交互界面
#include <opencv2/opencv.hpp>   //OpenCV终端部署:cv处理
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <chrono>    // 包含时间单位（如 milliseconds）

using namespace std;
using namespace cv;

class secondCapture
{
    public:
    ~secondCapture() {second_capture_main_end();}

    //第二个摄像头的capture 放到主循环函数中由系统释放
    // VideoCapture cap2;

    //第二个摄像头获取图像的主循环线程指针
    std::unique_ptr<std::thread> cap2_main_thread;  
    //主循环标志的原子变量
    std::atomic<bool> main_flag{false};  
    

    std::condition_variable cv;             // 条件变量用于线程同步
    std::mutex record_locker;

    //第二摄像头主循环,直接放在创建线程的函数中了
    void second_capture_main()
    {
        //初始化摄像头
        VideoCapture cap1("/dev/video0");//摄像头是1还是0要开图像看一眼确定
        int frame_width = cap1.get(CAP_PROP_FRAME_WIDTH);
        int frame_height = cap1.get(CAP_PROP_FRAME_HEIGHT);
        double fps = cap1.get(CAP_PROP_FPS);
        //设置摄图像分辨率
        cap1.set(CAP_PROP_FRAME_WIDTH, frame_width);
        cap1.set(CAP_PROP_FRAME_HEIGHT, frame_height);
        // 显式设置像素格式为 MJPG
        cap1.set(CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap1.set(CAP_PROP_FPS, CAP_PROP_FPS);

        //第二个摄像头获取到的图像
        cv::Mat second_capture_frame;
        while(main_flag)
        {

            std::unique_lock<std::mutex> lock(record_locker);
            cv.wait_for(lock, std::chrono::milliseconds(30));

            if (!cap1.read(second_capture_frame)) continue;

            // 检查窗口是否存在，如果被关闭则重新创建
            imshow("second_capture_img", second_capture_frame);
            // waitKey(30); 
            //不能在多个线程中同时调用waitKey，因为这样会导致竞争opencv的后台资源，摄像头卡住
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        return;
    }

    //开启第二摄像头主循环线程
    void second_capture_main_start()
    {
        // 如果线程已在运行则不重复创建
        if (cap2_main_thread && cap2_main_thread->joinable())
            return;
        main_flag = true;
        cap2_main_thread = make_unique<std::thread>(&secondCapture::second_capture_main,this);
        // 使用detach()保持与原代码一致
        cap2_main_thread->detach();
    }

        
    void second_capture_main_end() {
        if(main_flag) {
            main_flag = false;
            if(cap2_main_thread && cap2_main_thread->joinable()) {
                cap2_main_thread->join();  // 等待线程结束
                cap2_main_thread.reset();  // 释放线程对象
            }
        }
    }




};