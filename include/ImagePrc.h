#ifndef IMAGEPRC_H
#define IMAGEPRC_H

#include <iostream>
#include <opencv2/highgui.hpp>  //OpenCV终端部署:gui人机交互界面
#include <opencv2/opencv.hpp>   //OpenCV终端部署:cv处理
#include <cstdint>
#include <json_self.h>

#define     m_white     255
#define     m_black     0
#define     ImageSensorMid      LCDW / 2
#define     LimitL(L) (L = ((L < 1) ? 1 : L))    //限制幅度
#define     LimitH(H) (H = ((H > 118) ? 118 : H))  //限制幅度
#define     LCDH 60                              //用于图像处理图像的高度
#define     LCDW 120                              //用于图像处理图像的宽度

using namespace cv;

typedef struct 
{
  int point;
  uint8_t type;
} JumpPointtypedef;

typedef struct 
{
  /*左右边边界标志    T为正常跳变边    W为无边   P为障碍类多跳边的内边*/
  uint8_t IsRightFind;                          //右边有边标志
  uint8_t IsLeftFind;                           //左边有边标志
  int Wide;                                   //边界宽度
  int LeftBorder;                             //左边界
  int RightBorder;                            //右边界
  int Center;                                 //中线

  //左右手法则扫线数据
  int LeftBoundary_First;  //左边界 保存第一次数据
  int RightBoundary_First; //右边界 保存第一次数据
  int LeftBoundary;        //左边界 保存最后一次数据
  int RightBoundary;       //右边界 保存最后一次数据

} ImageDealDatatypedef;

//元素类型
typedef enum 
{
    Normol,       //无任何特征
    Straight,     ////直道
    Cross,        ////十字
    LeftCirque,   ////左圆环
    RightCirque,  ////右圆环
} RoadType_e;

typedef struct 
{
  /*以下关于全局图像正常参数*/
  int Det;             //由GetDet()函数解算出来的平均图像偏差
  int towPoint;

  //uint8_t 实际上是 unsigned char 的别名
  int16_t OFFLine;                              //图像顶边
  int16_t WhiteLine;                            //双边丢边数
  int16_t RWLine;                             //右丢线数量
  int16_t LWLine;                             //左丢线数量

  RoadType_e Road_type;  //元素类型

  //左右手法则扫线数据
  int16_t WhiteLine_L;        //左边丢线数
  int16_t WhiteLine_R;        //右边丢线数
  int16_t OFFLineBoundary;   //八领域截止行

} ImageStatustypedef;

typedef struct 
{
    int16_t image_element_rings;                  /*0 :无圆环          1 :左圆环       2 :右圆环*/
    int16_t ring_big_small;                       /*0:无                     1 :大圆环       2 :小圆环*/
    int16_t image_element_rings_flag;             /*圆环进程*/
    int16_t straight_long;                        /*长直道标志位*/
    int16_t image_element_cross;                  /*十字标志位          0  :无十字      1 :有十字*/      
    int16_t image_element_cross_flag;             /*十字进程*/     
} ImageFlagtypedef;

extern ImageDealDatatypedef ImageDeal[60];             //记录单行的信息
extern ImageStatustypedef ImageStatus;
extern ImageFlagtypedef ImageFlag;
extern bool stop_flag;
extern bool firstringFlag;

void stopProtect();
void firstPrcImage(Mat& img, Mat& re_img, int value);
void trackImg(Mat& img);
void imageProcess(int scene);
void imageProcess2(bool speedflag);
float straightJudge(uint8_t dir, uint8_t start, uint8_t end);
void getDet(int straighet_towpoint, Mat& img);

#endif
