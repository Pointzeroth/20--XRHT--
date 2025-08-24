#include "ImagePrc.h"
#include "Control.h"
#include "SpeedCommand2.cpp"

#define trackMod 1
#define binMOD 0
#define circleMod 0

SpeedController speedCtrl;
// 使用大津法自动计算阈值
double thresholdValue = 128; // 初始阈值（会被大津法覆盖）
double maxValue = 255;       // 二值化后的最大值

int ImageScanInterval = 6;                      //扫边范围    上一行的边界+-ImageScanInterval
int ImageScanInterval_Cross;                    //270°的弯道后十字的扫线范围


Rect useRoi(0, 27, 120, 60);
uint8_t Pixle[LCDH][LCDW];                        //用于处理的二值化图像

///////////////////////////////////////////
uint8_t Ring_Help_Flag = 0;                      //进环辅助标志
///////////////////////////////////////////

static int Ysite = 0, x = 0;                    //Y坐标=行，X是列
static uint8_t* pixtemp;                          //保存单行图像
static int IntervalLow = 6, IntervalHigh = 6;   //定义高低扫描区间
static int bottomright = LCDW - 1,              //89行右边界
bottomleft = 0,                                 //89行左边界
BottomCenter = 0;                               //89行中点

int Left_RingsFlag_Point1_Ysite, Left_RingsFlag_Point2_Ysite;   //左圆环判断的两点纵坐标
int Right_RingsFlag_Point1_Ysite, Right_RingsFlag_Point2_Ysite; //右圆环判断的两点纵坐标

ImageDealDatatypedef ImageDeal[LCDH];             //记录单行的信息
ImageStatustypedef ImageStatus;
ImageFlagtypedef ImageFlag;

bool stop_flag = false;
static int lastMidlle = 60;

uint8_t Half_Road_Wide[60]=                      //赛道半宽
{      
10,10,10,11,12,12,13,13,14,15,
15,16,16,17,17,18,18,19,20,20,
21,21,22,23,23,24,24,25,25,26,
26,27,28,28,29,29,30,31,31,32,
32,33,34,34,35,35,36,36,37,37,
38,39,39,40,40,41,42,43,44,45,
};

/*-------------------------------------------------------------------------------------------------------------------
   @brief     普通大津求阈值
   @param     image       图像数组
              col         列 ，宽度
              row         行，高度
   @return    threshold   返回int类型的的阈值
   Sample     threshold=My_Adapt_Threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//普通大津
   @note
 -------------------------------------------------------------------------------------------------------------------*/
int My_Adapt_Threshold(uint8_t* image, uint8_t width, uint8_t height)   //大津算法，注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height / 4;
    int  threshold = 0;
    uint8_t* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32_t gray_sum=0;
    for (i = 0; i < height; i+=2)//统计灰度级中每个像素在整幅图像中的个数想
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    for (i = 0; i < GrayScale; i++) //计算每个像素值的点在整幅图像中的比例
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)//遍历灰度级[0,255]
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);//平方
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;//最大类间方差法
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold>255)
        threshold=255;
    if(threshold<0)
        threshold=0;
return threshold;
}

 /*****************大津法参数*********************/
 float bin_float[256];     //灰度比例直方图
 int sizeimg = 60 * 118;
 float u = 0;                //全图平均灰度
 float w0 = 0;
 float u0 = 0;               //前景灰度
 uint8_t Bin_Array[256];
 float gray_hh = 0;//前景灰度和
 float var = 0;//方差
 float maxvar = 0;//最大方差
 float maxgray = 0;//最大灰度占比
 float maxbin = 0;
 uint8_t Thresholds[3] = {0};
 uint8_t threshold1 = 0;
 uint8_t Threshold = 0;//调试

 struct size_point
{
int x0;
int y0;
int x1;
int y1;
};

 struct size_point ostu_point[3]=
{
    {0,  0, 119, 19},  // 顶部区域: 0~19行 (高度20)
    {0, 20, 119, 39},  // 中部区域: 20~39行 (高度20)
    {0, 40, 119, 59}   // 底部区域: 40~59行 (高度20)
};
 /*****************大津法参数end*********************/

void Ostu(void)//大津
{
    int j, k;
    int i;
    uint8_t (*p_image)[120] = &Pixle[0];
    threshold1 = (uint8_t)My_Adapt_Threshold(Pixle[0],120,60);
    for (k = 0; k < 3; k++)
    {
        maxvar = 0;
        w0 = 0;
        u = 0;
        gray_hh = 0;
        var = 0;
        Thresholds[k] = 0;
        for (i = 0; i < 256; i++)
        { bin_float[i] = 0; }
        for (i = ostu_point[k].y0; i <= ostu_point[k].y1; i++)
        {
                for (j = ostu_point[k].x0; j <= ostu_point[k].x1; j++)
                {
                        ++bin_float[*(*(p_image + i) + j)];
                }
        }
        sizeimg = (ostu_point[k].y1 - ostu_point[k].y0 + 1) * (ostu_point[k].x1 - ostu_point[k].x0 + 1);
        for (i = 0; i < 256; i++)
        {
                bin_float[i] = bin_float[i] / sizeimg;
                u += i * bin_float[i];
        }
        //创建比例灰度直方图
        for (i = 0; i < 256; i++)
        {
                w0 += bin_float[i];
                gray_hh += i * bin_float[i];             //灰度和
                u0 = gray_hh / w0;
                var = (u0 - u) * (u0 - u) * w0 / (1 - w0);
                if (var > maxvar)
                {
                        maxgray = gray_hh;
                        maxbin = w0;
                        maxvar = var;
                        Thresholds[k] = (uint8_t)i;

                }
        }
        if (k == 0)
        {
                if (gray_hh > 15 && gray_hh <= 33)
                {
                        if (maxbin < 0.9f)
                        {
                                Thresholds[k] = (uint8_t)(Thresholds[1] - 3);
                        }
                }
                else if (gray_hh > 41 && gray_hh <= 47)
                {
                        if (maxbin < 0.64f || maxbin > 0.76f)
                        {
                                Thresholds[k] = (uint8_t)(Thresholds[1] - 3);
                        }
                }
                else if (gray_hh > 50 && gray_hh <= 60)
                {
                        if (maxbin < 0.42f || maxbin > 0.58f)
                        {
                                Thresholds[k] = (uint8_t)(Thresholds[1] - 3);
                        }
                }
                                                                if (abs(threshold1 - Thresholds[k]) >= 30)//相差太大直接用标准值
                                                                                                {
                                                                                                    Thresholds[k] = threshold1;
                                                                                                }
                                                                                                else
                                                                                                {
                                                                                                    Thresholds[k] = (uint8_t)(Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]));//这里是取了平均，可以通过调整占比适应不同的光照条件
                                                                                                }
                //SetText("右边var:" + maxvar + " 阈值:" + Thresholds[k] + " 比例:" + maxbin + " 灰度和" + gray_hh);
        }
        else if (k == 1)
        {
                if(gray_hh > 69 && gray_hh < 80)
                {
                        if (maxbin > 0.15f)
                        {
                                Thresholds[k] = (uint8_t)(Thresholds[0] + 3);



                                                                                                }
                }
                                                                if (abs(threshold1 - Thresholds[k]) >= 30)//相差太大直接用标准值
                                                                                                {
                                                                                                    Thresholds[k] = threshold1;
                                                                                                }
                                                                                                else
                                                                                                {
                                                                                                    Thresholds[k] = (uint8_t)(Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]));//这里是取了平均，可以通过调整占比适应不同的光照条件
                                                                                                }
                //SetText("中间var:" + maxvar + " 阈值:" + Thresholds[k] + " 比例:" + maxbin + " 灰度和" + gray_hh);
        }
        else if (k == 2)
        {
                if (maxbin < 0.85f && gray_hh < 28)
                {
                        Thresholds[k] = (uint8_t)(Thresholds[1] - 3);
                }
                else if(gray_hh > 69 && gray_hh < 79)
                {
                        if (maxbin < 0.5f || maxbin > 0.15f)
                        {
                                Thresholds[k] = (uint8_t)(Thresholds[1] - 3);
                        }
                }
                                                                if (abs(threshold1 - Thresholds[k]) >= 30)//相差太大直接用标准值
                                                                {
                                                                    Thresholds[k] = threshold1;
                                                                }
                                                                else
                                                                {
                                                                    Thresholds[k] = (uint8_t)(Thresholds[k] + 0.5f * (threshold1 - Thresholds[k]));//这里是取了平均，可以通过调整占比适应不同的光照条件
                                                                }
                //SetText("左边var:" + maxvar + " 阈值:" + Thresholds[k] + " 比例:" + maxbin + " 灰度和" + gray_hh);
        }

        for (i = 0; i < Thresholds[k]; i++)
        {
                Bin_Array[i] = 0;
        }
        #if binMOD == 3
        for (i = Thresholds[k]; i < 256; i++)
        {
                Bin_Array[i] = 255;
        }
        #else
        for (i = Thresholds[k]; i < 256; i++)
        {
                Bin_Array[i] = 1;
        }
        #endif
        for (i = ostu_point[k].y0; i <= ostu_point[k].y1; i++)
        {
                for (j = ostu_point[k].x0; j <= ostu_point[k].x1; j++)
                {
                    Pixle[i][j] = Bin_Array[*(*(p_image + i) + j)];
                }
        }
        Threshold  =  Thresholds[k];
    }
}


//巡线初步处理函数
void firstPrcImage(Mat& img, Mat& re_img, int value)
{
    Mat gray_img, use_img; //img用于ai识别,use_img用于扫线
    Mat store_img;
    //巡线图像处理操作
    resize(img, re_img, Size(120, 90), 0, 0, INTER_LINEAR); //压缩图像
    re_img = re_img(useRoi);
    cvtColor(re_img, gray_img, COLOR_BGR2GRAY);    //图像RGB转灰度图像

    #if (binMOD == 0) || (binMOD == 1)
    double otsu_thresh = threshold(gray_img, store_img, thresholdValue, maxValue, cv::THRESH_OTSU);
    int manual_thresh = otsu_thresh + value;  // cvRound用于四舍五入取整
    manual_thresh = std::clamp(manual_thresh, 0, 255); // 确保阈值在0-255范围内
    threshold(gray_img, use_img, manual_thresh, maxValue, THRESH_BINARY);

    for (int i = 0; i < LCDH; i++) 
    {
        for (int j = 0; j < LCDW; j++) 
        {
            Pixle[i][j] = use_img.at<uint8_t>(i, j); // 复制像素值
        }
    }

    #if binMOD == 1
    namedWindow("bin_img", (640, 480));
    imshow("bin_img", use_img); //显示二值化后的图像
    #endif

    for(int i = 0; i < LCDH; i ++)
    {
        for(int j = 0; j < LCDW; j ++)
        {
            if(Pixle[i][j] == 255)
                Pixle[i][j] = 1;
            else
                Pixle[i][j] = 0;
        }
    }
    #endif
    
    #if (binMOD == 2) || (binMOD == 3)
    for (int i = 0; i < LCDH; i++) 
    {
        for (int j = 0; j < LCDW; j++) 
        {
            Pixle[i][j] = gray_img.at<uint8_t>(i, j); // 复制像素值
        }
    }

    Ostu();

    #if binMOD == 3
    cv::Mat binimg = cv::Mat::zeros(LCDH, LCDW, CV_8UC1); // 创建全零图像
    for(int i = 0; i < LCDH; i++) 
    {
        for (int j = 0; j < LCDW; j++) 
        {
            binimg.at<uint8_t>(i, j) = Pixle[i][j];
        }
    }
    namedWindow("bin_img", (640, 480));
    imshow("bin_img", binimg); //显示二值化后的图像
    for(int i = 0; i < LCDH; i ++)
    {
        for(int j = 0; j < LCDW; j ++)
        {
            if(Pixle[i][j] == 255)
                Pixle[i][j] = 1;
            else
                Pixle[i][j] = 0;
        }
    }
    #endif
    #endif
}

void pixleFilter() 
{
  int nr;  //行
  int nc;  //列

  for (nr = 10; nr < LCDH - 10; nr ++) 
  {
    for (nc = 10; nc < LCDW - 10; nc = nc + 1) 
    {
      if ((Pixle[nr][nc] == 0) && 
        (Pixle[nr - 1][nc] + Pixle[nr + 1][nc] +
        Pixle[nr][nc + 1] + Pixle[nr][nc - 1] >= 3)) 
      {
        Pixle[nr][nc] = 1;
      }
    }
  }
}

void stopProtect()
{
    if(ImageStatus.OFFLine >= LCDH - 5)
        stop_flag = true;
}

void GetJumpPointFromDet(uint8_t* p,uint8_t type,int L,int H,JumpPointtypedef* Q)
                                                                               
{                                                                              
  int i = 0;
  if (type == 'L')                              //扫描左边线
  {
    for (i = H; i >= L; i--) {
      if (*(p + i) == 1 && *(p + i - 1) != 1)   //由白变黑
      {
        Q->point = i;                           //记录左边线
        Q->type = 'T';                          //正确跳变
        break;
      } else if (i == (L + 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //认为左边线是中点
          Q->type = 'W';                        //非正确跳变且中间为白，认为没有边
          break;
        } else                                  //非正确跳变且中间为黑
        {
          Q->point = H;                         //如果中间是黑的
          Q->type = 'H';                        //左边线直接最大值，认为是大跳变
          break;
        }
      }
    }
  } else if (type == 'R')                       //扫描右边线
  {
    for (i = L; i <= H; i++)                    //从右往左扫
    {
      if (*(p + i) == 1 && *(p + i + 1) != 1)   //找由黑到白的跳变
      {
        Q->point = i;                           //记录
        Q->type = 'T';
        break;
      } else if (i == (H - 1))                  //若果扫到最后也没找到
      {
        if (*(p + (L + H) / 2) != 0)            //如果中间是白的
        {
          Q->point = (L + H) / 2;               //右边线是中点
          Q->type = 'W';
          break;
        } else                                  //如果中点是黑的
        {
          Q->point = L;                         //左边线直接最大值
          Q->type = 'H';
          break;
        }
      }
    }
  }
}

int whiteColumn[LCDW];//每列白列长度
uint8_t whiteColumnLeft[2];
uint8_t whiteColumnRight[2];

static uint8_t drawLinesFirst(int scene) 
{
    pixtemp = Pixle[LCDH - 1];
    whiteColumnLeft[0] = 0;
    whiteColumnLeft[1] = 0;
    whiteColumnRight[0] = 0;
    whiteColumnRight[1] = 0;
    int i, j;
 
    
    for(i = 0;i < LCDW - 1; i ++)
    {
        whiteColumn[i] = 0;
    }

    //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j = 20; j < LCDW - 20; j ++)
    {
        for (i = LCDH - 1; i > 0; i--)
        {
            if(Pixle[i][j] == 0)
                break;
            else
                whiteColumn[j]++;
        }
    }

    if(whiteColumn[lastMidlle] <= 5)
    {
        //从左到右找左边最长白列
        for(i = 20; i < LCDW - 20; i ++)
        {
            if (whiteColumnLeft[0] < whiteColumn[i])//找最长的那一列
            {
                whiteColumnLeft[0] = whiteColumn[i];//【0】是白列长度
                whiteColumnLeft[1] = i;              //【1】是下标，第j列
            }
        }
        //从右到左找右左边最长白列
        for(i = LCDW - 20; i > 20; i --)//从右往左，注意条件，找到左边最长白列位置就可以停了
        {
            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
            {
                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                whiteColumnRight[1] = i;              //【1】是下标，第j列
            }
        }
    }
    else
    {
        /**********************************************************************************/
        //从中线到右找右边最长白列
        for(i = lastMidlle; i < LCDW - 4; i ++)
        {
            if (whiteColumnRight[0] < whiteColumn[i])//找最长的那一列
            {
                whiteColumnRight[0] = whiteColumn[i];//【0】是白列长度
                whiteColumnRight[1] = i;              //【1】是下标，第j列
            }
    
            if(whiteColumn[i] < 2)
                break;
        }
        //从中线到左找左左边最长白列
        for(i = lastMidlle; i > 4; i --)//从右往左，注意条件，找到左边最长白列位置就可以停了
        {
            if (whiteColumnLeft[0] < whiteColumn[i])//找最长的那一列
            {
                whiteColumnLeft[0] = whiteColumn[i];//【0】是白列长度
                whiteColumnLeft[1] = i;              //【1】是下标，第j列
            }
    
            if(whiteColumn[i] < 2)
                break;
        }
    }

    for (x = whiteColumnRight[1]; x < LCDW - 1; x ++)                                    
    {
        if (  *(pixtemp + x) == 0
            &&*(pixtemp + x + 1) == 0)                                        
        {
            bottomright = x;                                                   
            break;
        } else if (x == LCDW - 2) {
            bottomright = LCDW - 1;                                                   
            break;
        }
    }
    for (x = whiteColumnLeft[1]; x > 0; x --)                                 
    {
        if (  *(pixtemp + x) == 0
            &&*(pixtemp + x - 1) == 0)                                        
        {
            bottomleft = x;                
            break;
        } else if (x == 1) {
            bottomleft = 0;                                                    
            break;
        }
    }

    BottomCenter =(bottomleft + bottomright) / 2;
    lastMidlle = BottomCenter;
    ImageDeal[LCDH - 1].LeftBorder = bottomleft;                                    
    ImageDeal[LCDH - 1].RightBorder = bottomright;
    ImageDeal[LCDH - 1].Center = BottomCenter;                                      
    ImageDeal[LCDH - 1].Wide = bottomright - bottomleft;
    if(ImageDeal[LCDH - 1].LeftBorder != 0)
        ImageDeal[LCDH - 1].IsLeftFind = 'T';
    if(ImageDeal[LCDH - 1].RightBorder != LCDW - 1)
        ImageDeal[LCDH - 1].IsRightFind = 'T';
    
    for (Ysite = LCDH - 2; Ysite > LCDH - 4; Ysite--)     
    {
        pixtemp = Pixle[Ysite];
        for (x = whiteColumnRight[1]; x < LCDW - 1; x ++)                                                         
        {
            if (*(pixtemp + x) == 0 && *(pixtemp + x + 1) == 0) 
            {
                ImageDeal[Ysite].RightBorder = x;
                break;
            } 
            else if (x == LCDW - 2) 
            {
                ImageDeal[Ysite].RightBorder = LCDW - 1;
                break;
            }
        }
        for (x = whiteColumnLeft[1]; x > 0; x --)                                                             
        {
            if (*(pixtemp + x) == 0 && *(pixtemp + x - 1) == 0) 
            {
                ImageDeal[Ysite].LeftBorder = x;
                break;
            } 
            else if (x == 1) 
            {
                ImageDeal[Ysite].LeftBorder = 0;
                break;
            }
        }
        if(ImageDeal[Ysite].LeftBorder != 0)
            ImageDeal[Ysite].IsLeftFind = 'T';
        if(ImageDeal[Ysite].RightBorder != LCDW - 1)
            ImageDeal[Ysite].IsRightFind = 'T';
        ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) /2;    
        ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;        
    }
    return 'T';
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////
/*边线追逐大致得到全部边线*/
static void drawLinesProcess(void)  
{
  ImageStatus.OFFLine = 0;
  ImageStatus.WhiteLine = 0;
  ImageStatus.RWLine = 0;
  ImageStatus.LWLine = 0;

  for (Ysite = LCDH - 4 ; Ysite > ImageStatus.OFFLine; Ysite--)                 
  {                                                                         
    pixtemp = Pixle[Ysite];
    JumpPointtypedef JumpPoint[2];                                          // 0左1右

    IntervalLow =ImageDeal[Ysite + 1].RightBorder -ImageScanInterval;             //从上一行右边线-Interval的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].RightBorder + ImageScanInterval;           //到上一行右边线+Interval的点结束（确定扫描结束点）

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(pixtemp, 'R', IntervalLow, IntervalHigh,&JumpPoint[1]);     //扫右边线

    IntervalLow =ImageDeal[Ysite + 1].LeftBorder -ImageScanInterval;                //从上一行左边线-5的点开始（确定扫描开始点）
    IntervalHigh =ImageDeal[Ysite + 1].LeftBorder +ImageScanInterval;               //到上一行左边线+5的点结束（确定扫描结束点）

    LimitL(IntervalLow);   //确定左扫描区间并进行限制
    LimitH(IntervalHigh);  //确定右扫描区间并进行限制
    GetJumpPointFromDet(pixtemp, 'L', IntervalLow, IntervalHigh,&JumpPoint[0]);

    if (JumpPoint[0].type =='W')                                                    //如果本行左边线不正常跳变，即这10个点都是白的
    {
      ImageDeal[Ysite].LeftBorder =ImageDeal[Ysite + 1].LeftBorder;                 //本行左边线用上一行的数值
    } else                                                                          //左边线正常
    {
      ImageDeal[Ysite].LeftBorder = JumpPoint[0].point;                             //记录下来啦
    }

    if (JumpPoint[1].type == 'W')                                                   //如果本行右边线不正常跳变
    {
      ImageDeal[Ysite].RightBorder =ImageDeal[Ysite + 1].RightBorder;               //本行右边线用上一行的数值
    } else                                                                          //右边线正常
    {
      ImageDeal[Ysite].RightBorder = JumpPoint[1].point;                            //记录下来啦
    }

    ImageDeal[Ysite].IsLeftFind =JumpPoint[0].type;                                 //记录本行是否找到边线，即边线类型
    ImageDeal[Ysite].IsRightFind = JumpPoint[1].type;

    //重新确定那些大跳变的边缘
    if ( ImageDeal[Ysite].IsLeftFind == 'H'
         ||ImageDeal[Ysite].IsRightFind == 'H') {
      if (ImageDeal[Ysite].IsLeftFind == 'H')
      //如果左边线大跳变
        for (x = (ImageDeal[Ysite].LeftBorder + 1);
             x <= (ImageDeal[Ysite].RightBorder - 1);
             x++)                                                           //左右边线之间重新扫描
        {
          if ((*(pixtemp + x) == 0) && (*(pixtemp + x + 1) != 0)) {
            ImageDeal[Ysite].LeftBorder =x;                                 //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsLeftFind = 'T';
            break;
          } else if (*(pixtemp + x) != 0)                                   //一旦出现白点则直接跳出
            break;
          else if (x ==(ImageDeal[Ysite].RightBorder - 1))
          {
             ImageDeal[Ysite].IsLeftFind = 'T';
             ImageDeal[Ysite].LeftBorder =x;
            break;
          }
        }

      if ((ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder) <= 15)                              //图像宽度限定
      {
        ImageStatus.OFFLine = Ysite + 1;  //如果这行比7小了后面直接不要了
        break;
      }

      if (ImageDeal[Ysite].IsRightFind == 'H')
        for (x = (ImageDeal[Ysite].RightBorder - 1);
             x >= (ImageDeal[Ysite].LeftBorder + 1); x--) {
          if ((*(pixtemp + x) == 0) && (*(pixtemp + x - 1) != 0)) {
            ImageDeal[Ysite].RightBorder =
                x;                                                          //如果右边线的左边还有黑白跳变则为绝对边线直接取出
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          } else if (*(pixtemp + x) != 0)
            break;
          else if (x == (ImageDeal[Ysite].LeftBorder + 1))
          {
            ImageDeal[Ysite].RightBorder = x;
            ImageDeal[Ysite].IsRightFind = 'T';
            break;
          }
        }
    }

    /*   丢边数量统计               */
    if (   ImageDeal[Ysite].IsLeftFind == 'W'
         &&ImageDeal[Ysite].IsRightFind == 'W')
          ImageStatus.WhiteLine ++;                  //要是左右都无边，丢边数+1
    if(ImageDeal[Ysite].IsLeftFind == 'W' && Ysite < LCDH - 15 && Ysite > 15 )
        ImageStatus.LWLine ++;
    if(ImageDeal[Ysite].IsRightFind == 'W' && Ysite < LCDH - 15 && Ysite > 15 )
        ImageStatus.RWLine ++;

      LimitL(ImageDeal[Ysite].LeftBorder);          //限幅
      LimitH(ImageDeal[Ysite].LeftBorder);          //限幅
      LimitL(ImageDeal[Ysite].RightBorder);         //限幅
      LimitH(ImageDeal[Ysite].RightBorder);         //限幅

      ImageDeal[Ysite].Wide =ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
      ImageDeal[Ysite].Center =(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;

    if (ImageDeal[Ysite].Wide <= 10)                 //重新确定可视距离
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (ImageDeal[Ysite].RightBorder < ImageDeal[Ysite].LeftBorder - 5)                 //右边线必须在左边线的右边
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }

    else if (  ImageDeal[Ysite].RightBorder <= 10
             ||ImageDeal[Ysite].LeftBorder >= LCDW - 10) 
    {
      ImageStatus.OFFLine = Ysite + 1;
      break;
    }                                         //当图像宽度小于0或者左右边达到一定的限制时，则终止巡边
  }
}

//迷宫法巡线
void search_Bottom_Line_OTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row, uint8_t Col, uint8_t Bottonline)
{

    //寻找左边边界
    for (int Xsite = whiteColumnLeft[1]; Xsite > 1; Xsite--)//正常寻找左右边线
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite - 1] == 0)
        {
            ImageDeal[Bottonline].LeftBoundary = Xsite;//获取底边左边线
            break;
        }
    }
    for (int Xsite = whiteColumnRight[1]; Xsite < LCDW-2; Xsite++)
    {
        if (imageInput[Bottonline][Xsite] == 1 && imageInput[Bottonline][Xsite + 1] == 0)
        {
            ImageDeal[Bottonline].RightBoundary = Xsite;//获取底边右边线
            // std::cout << Xsite << std::endl;
            break;
        }
    }
}

void search_Left_and_Right_Lines(uint8_t imageInput[LCDH][LCDW], uint8_t Row, uint8_t Col, uint8_t Bottonline)
{
    //定义小人的当前行走状态位置为 上 左 下 右 一次要求 上：左边为黑色 左：上边为黑色 下：右边为黑色  右：下面有黑色
/*  前进方向定义：
                *   D0
                * D3   D1
                *   D2
*/
/*寻左线坐标规则*/
    int Left_Rule[2][8] = {
                                  {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},  (x,y )
                                  {-1,-1,1,-1,1,1,-1,1} //{-1,-1},{1,-1},{1,1},{-1,1}
    };
    /*寻右线坐标规则*/
    int Right_Rule[2][8] = {
                              {0,-1,1,0,0,1,-1,0 },//{0,-1},{1,0},{0,1},{-1,0},
                              {1,-1,1,1,-1,1,-1,-1} //{1,-1},{1,1},{-1,1},{-1,-1}
    };
    int num=0;
    uint8_t Left_Ysite = Bottonline;
    uint8_t Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
    uint8_t Left_Rirection = 0;//左边方向
    uint8_t Pixel_Left_Ysite = Bottonline;
    uint8_t Pixel_Left_Xsite = 0;

    uint8_t Right_Ysite = Bottonline;
    uint8_t Right_Xsite = ImageDeal[Bottonline].RightBoundary;
    uint8_t Right_Rirection = 0;//右边方向
    uint8_t Pixel_Right_Ysite = Bottonline;
    uint8_t Pixel_Right_Xsite = 0;
    uint8_t Ysite = Bottonline;
    ImageStatus.OFFLineBoundary = 1;
    while (1)
    {
        //防卡死
            num++;
            if(num > 400)
            {
                 ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
        //Ysite必须要在前方坐标的下方或者与前方坐标相等（第一次进入循环时候是相等的）
        if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
        {
            if (Ysite < ImageStatus.OFFLineBoundary)        //当Ysite跑到OFFLine前面的时候，就把OFFline设置为Ysite的值，并且跳出循环
            {
                ImageStatus.OFFLineBoundary = Ysite;
                break;
            }
            else
            {
                Ysite --;        //Ysite依次往前走
            }
        }
        /*********左边巡线*******/
        if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//左边扫线
        {
            /*计算前方坐标*/
            Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
            Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];

//            这里，2 * Left_Rirection + 1 返回 Left_Rule[0] 中与 Y 方向相关的增量:
//            当 Left_Rirection 为 0（上）时，Y 增量为 -1。
//            当 Left_Rirection 为 1（右）时，Y 增量为 0。
//            当 Left_Rirection 为 2（下）时，Y 增量为 +1。
//            当 Left_Rirection 为 3（左）时，Y 增量为 0。
//            这样，Pixel_Left_Ysite 就是根据当前方向调整过后的 Y 坐标。
//            同理，2 * Left-Rirection 返回的就是Left_Rule[0] 中与 X 方向相关的增量。

            if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//前方是黑色
            {
                //顺时针旋转90
                if (Left_Rirection == 3)    //当在方向3的时候，转过90°即是0
                    Left_Rirection = 0;
                else
                    Left_Rirection++;       //++即顺时针转90°
            }
            else//前方是白色
            {
                /*计算左前方坐标*/
                Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];

                if (imageInput[Pixel_Left_Ysite][Pixel_Left_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Left_Rirection
                    Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0){ //第一次的边线，后面又可能还会爬到这个Ysite位置。
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    }
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Left_Rirection  逆时针90度
                    Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
                    Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
                    if (ImageDeal[Left_Ysite].LeftBoundary_First == 0 )
                        ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
                    ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
                    if (Left_Rirection == 0)
                        Left_Rirection = 3;
                    else
                        Left_Rirection--;
                }
            }
        }
        /*********右边巡线*******/
        if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)//右边扫线
        {
            /*计算前方坐标*/
            Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
            Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];

            if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//前方是黑色
            {
                //逆时针旋转90
                if (Right_Rirection == 0)
                    Right_Rirection = 3;
                else
                    Right_Rirection--;
            }
            else//前方是白色
            {
                /*计算右前方坐标*/
                Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];

                if (imageInput[Pixel_Right_Ysite][Pixel_Right_Xsite] == 0)//左前方为黑色
                {
                    //方向不变  Right_Rirection
                    Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == LCDW - 1 )
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                }
                else//左前方为白色
                {
                    // 方向发生改变 Right_Rirection  逆时针90度
                    Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
                    Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
                    if (ImageDeal[Right_Ysite].RightBoundary_First == LCDW - 1)
                        ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
                    ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
                    if (Right_Rirection == 3)
                        Right_Rirection = 0;
                    else
                        Right_Rirection++;
                }
            }
        }

        if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 3)
        {

            ImageStatus.OFFLineBoundary = Ysite;            //设置OFFLine
            break;
        }
    }
    // std::cout << "1:" << ImageStatus.OFFLineBoundary << std::endl;
}

void searchBorderOTSU(uint8_t imageInput[LCDH][LCDW], uint8_t Row, uint8_t Col, uint8_t Bottonline)
{
    ImageStatus.WhiteLine_L = 0;
    ImageStatus.WhiteLine_R = 0;
    //ImageStatus.OFFLine = 1;
    /*封上下边界处理*/
    for (int Xsite = 0; Xsite < LCDW; Xsite++)
    {
        imageInput[0][Xsite] = 0;
        imageInput[Bottonline + 1][Xsite] = 0;
    }
    /*封左右边界处理*/
    for (int Ysite = 0; Ysite < LCDH; Ysite++)
    {
        ImageDeal[Ysite].LeftBoundary_First = 0;
        ImageDeal[Ysite].RightBoundary_First = LCDW - 1;

        imageInput[Ysite][0] = 0;
        imageInput[Ysite][LCDW - 1] = 0;
    }
    /********获取底部边线*********/
    // search_Bottom_Line_OTSU(imageInput, Row, Col, Bottonline);

    ImageDeal[Bottonline].LeftBoundary = ImageDeal[Bottonline].LeftBorder;//获取底边左边线
    ImageDeal[Bottonline].RightBoundary = ImageDeal[Bottonline].RightBorder;//获取底边右边线

    /********获取左右边线*********/
    search_Left_and_Right_Lines(imageInput, Row, Col, Bottonline);

    // std::cout << "2:" << ImageDeal[58].RightBoundary << std::endl;

    for (int Ysite = Bottonline; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary < 5)
        {
            ImageStatus.WhiteLine_L++;
        }
        if (ImageDeal[Ysite].RightBoundary > LCDW - 5)
        {
            ImageStatus.WhiteLine_R++;
        }
    }
}


float straightJudge(uint8_t dir, uint8_t start, uint8_t end)     //返回结果小于某个值时判断为直线start < end
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    if(start >= end)
        return 2;

    switch (dir)
    {
    case 1:k = (float)(ImageDeal[start].LeftBorder - ImageDeal[end].LeftBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder) * (ImageDeal[start].LeftBorder + k * i - ImageDeal[i + start].LeftBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:k = (float)(ImageDeal[start].RightBorder - ImageDeal[end].RightBorder) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder) * (ImageDeal[start].RightBorder + k * i - ImageDeal[i + start].RightBorder);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    }
    return S;
}


void cross_handle()
{
    static int right_down_x = 0;
    static int right_down_y = 0;
    int right_down_flag = 0;

    static int left_down_x = 0;
    static int left_down_y = 0;
    int left_down_flag = 0;

    static int right_up_x = 0;
    static int right_up_y = 0;
    int right_up_flag = 0;

    static int left_up_x = 0;
    static int left_up_y = 0;
    int left_up_flag = 0;
    
    int down_max = 0;
    int up_min = 0;

    int change_pointY1 = 0;//行跳变点纵坐标
    int change_pointY2 = 0;
    int change_pointY3 = 0;

    static bool flag1 = false;
    static bool flag2 = false;

    float slope_k_L = 0;
    float slope_k_R = 0;

    int W_R_Linecount = 0 ,W_L_Linecount = 0;

    int  min_up_y = 0;
    bool whiteline_flag = false;
    // float right = 0;
    // float left = 0;
    //前三个for是正常圆环判断方案
    for (int Ysite = 53; Ysite >= 15; Ysite--)//15
    {
        if (ImageDeal[Ysite-3].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 6 )
        {
            right_down_y = Ysite;
            right_down_x = ImageDeal[Ysite].RightBoundary_First;
            // if(abs(ImageDeal[right_down_y + 3].RightBoundary_First - ImageDeal[right_down_y ].RightBoundary_First) <= 5)
                right_down_flag = 1;
            // std::cout << "point1:" << Right_RingsFlag_Point1_Ysite << std::endl;
            break;
        }
    }
    for (int Ysite = 53; Ysite >= 15; Ysite--)//15
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite-3].LeftBoundary_First > 6 )
        {
            left_down_y = Ysite + 2;
            left_down_x = ImageDeal[Ysite + 2].LeftBoundary_First;
            // if(abs(ImageDeal[left_down_y + 3].LeftBoundary_First - ImageDeal[left_down_y ].LeftBoundary_First) <= 5)
                left_down_flag = 1;
            // std::cout << "point1:" << Right_RingsFlag_Point1_Ysite << std::endl;
            break;
        }
    }
    for (int Ysite = ImageStatus.OFFLine + 10; Ysite < 50 ; Ysite++)
    {
        int W_RU_count = 0;
        if (ImageDeal[Ysite].RightBoundary_First - ImageDeal[Ysite-3].RightBoundary_First >= 5 
        // && ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite-1].RightBoundary <= 2
        // && ImageDeal[Ysite - 1].RightBoundary - ImageDeal[Ysite-2].RightBoundary <= 2
        // && ImageDeal[Ysite - 2].RightBoundary - ImageDeal[Ysite-3].RightBoundary <= 2
        // && Ysite >= 20 
        )
        {
            right_up_y = Ysite - 5;
            right_up_x = ImageDeal[Ysite-5].RightBoundary_First ;
            for(int i = right_up_y ; i < right_up_y + 14; i++)
            {
                if(ImageDeal[i].RightBoundary_First - ImageDeal[right_up_y].RightBoundary_First >= 7)
                {
                    W_RU_count ++;
                }
                if(W_RU_count >= 6)
                {
                    right_up_flag = 1;
                    break;
                }
            }
            if(right_up_flag) break;
        }
    }
    for (int Ysite = ImageStatus.OFFLine + 10; Ysite < 50 ; Ysite++)
    {
        int W_LU_count = 0;
        if (ImageDeal[Ysite-3].LeftBoundary_First - ImageDeal[Ysite].LeftBoundary_First >= 5 
        // && ImageDeal[Ysite-1].LeftBoundary - ImageDeal[Ysite].LeftBoundary <= 2
        // && ImageDeal[Ysite - 2].LeftBoundary - ImageDeal[Ysite-1].LeftBoundary <= 2
        // && ImageDeal[Ysite - 3].LeftBoundary - ImageDeal[Ysite-2].LeftBoundary <= 2
        // && Ysite >= 20 
        )
        {
            left_up_y = Ysite - 5;
            left_up_x = ImageDeal[Ysite-5].LeftBoundary_First;
            for(int i = left_up_y ; i < left_up_y + 14; i++)
            {
                if(ImageDeal[left_up_y].LeftBoundary_First - ImageDeal[i].LeftBoundary_First >= 7)
                {
                    W_LU_count ++;
                }
                if(W_LU_count >= 6)
                {
                    left_up_flag = 1;
                    break;
                }
            }
            if(left_up_flag) break;
        }
    }
    // if(right_down_y >= 7 && right_down_y <= 52 && left_down_y >= 7 && left_down_y <=52 )
    // {
    //     right =  straightJudge(2,right_down_y - 5,right_down_y + 5) ;
    //     left = straightJudge(1,left_down_y - 5,left_down_y + 5) ;
    //     std::cout << "右拐点上下五行是否是直线 :" <<right<< std::endl;
    //     std::cout << "左拐点上下五行是否是直线 :" << left<< std::endl;

    // }
    // std::cout << straightJudge(2,right_down_y - 5,right_down_y + 5) << std::endl;
    // std::cout << "左拐点上下五行是否是直线 :" << straightJudge(1,left_down_y - 5,left_down_y + 5) << std::endl;

    down_max = right_down_y >= left_down_y ? right_down_y : left_down_y;
    up_min = right_up_y <= left_up_y ? right_up_y : left_up_y;
    
    if(ImageFlag.image_element_cross == 0)
    {
        if(right_down_flag && left_down_flag && left_down_y >= 15 && right_down_y >= 15 && abs(right_down_y - left_down_y) <= 30)
        {
            for(int i = right_down_y ; i > right_down_y - 15; i--)
            {
                if(ImageDeal[i].RightBoundary_First - ImageDeal[right_down_y].RightBoundary_First >= 7)
                {
                    W_R_Linecount ++;
                }
            }
            for(int i = left_down_y ; i > left_down_y - 15; i--)
            {
                if(ImageDeal[left_down_y].LeftBoundary_First - ImageDeal[i].LeftBoundary_First >= 7)
                {
                    W_L_Linecount ++;
                }
            }
            if(right_up_flag == 1 && left_up_flag == 0)                   
            {
                change_pointY1 = speedCtrl.search_cols_change_point((right_down_x + left_down_x + 4)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((right_down_x + left_down_x + 4)/2 + 7 < 118 ? (right_down_x + left_down_x + 4)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((right_down_x + left_down_x + 4)/2 - 7 > 2 ? (right_down_x + left_down_x + 4)/2 - 7 : 2);
            }
            else if(left_up_flag == 1 && right_up_flag == 0)
            {
                change_pointY1 = speedCtrl.search_cols_change_point((right_down_x + left_down_x - 4)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((right_down_x + left_down_x - 4)/2 + 7 < 118 ? (right_down_x + left_down_x - 4)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((right_down_x + left_down_x - 4)/2 - 7 > 2 ? (right_down_x + left_down_x - 4)/2 - 7 : 2);
            }
            else if(left_up_flag == 1 && right_up_flag == 1)
            {
                change_pointY1 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2 + 7 < 118 ? (left_up_x + right_up_x)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2 - 7 > 2 ? (left_up_x + right_up_x)/2 - 7 : 2);
            }
            else    
            {
                change_pointY1 = speedCtrl.search_cols_change_point((right_down_x + left_down_x)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((right_down_x + left_down_x)/2 + 7 < 118 ? (right_down_x + left_down_x)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((right_down_x + left_down_x)/2 - 7 > 2 ? (right_down_x + left_down_x)/2 - 7 : 2);
            }
        }
        else if(right_up_flag == 1 && left_up_flag == 1 && (right_down_flag || left_down_flag))
        {
            
            if(ImageStatus.WhiteLine > 8)
            {
                whiteline_flag = true;
            }
            if(right_down_flag == 1 && left_down_flag == 0)                   
            {
                change_pointY1 = speedCtrl.search_cols_change_point((right_up_x + left_up_x - 4)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((right_up_x + left_up_x - 4)/2 + 7 < 118 ? (right_up_x + left_up_x - 4)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((right_up_x + left_up_x - 4)/2 - 7 > 2 ? (right_up_x + left_up_x - 4)/2 - 7 : 2);
            }
            else if(left_down_flag == 1 && right_down_flag == 0)
            {
                change_pointY1 = speedCtrl.search_cols_change_point((right_up_x + left_up_x + 4)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((right_up_x + left_up_x + 4)/2 + 7 < 118 ? (right_up_x + left_up_x + 4)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((right_up_x + left_up_x + 4)/2 - 7 > 2 ? (right_up_x + left_up_x + 4)/2 - 7 : 2);
            }
            else if(left_down_flag == 1 && right_down_flag == 1)
            {
                change_pointY1 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2);
                change_pointY2 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2 + 7 < 118 ? (left_up_x + right_up_x)/2 + 7 : 118);
                change_pointY3 = speedCtrl.search_cols_change_point((left_up_x + right_up_x)/2 - 7 > 2 ? (left_up_x + right_up_x)/2 - 7 : 2);
            }
        }
        if((right_down_flag == 1 && left_down_flag == 1  && W_R_Linecount >= 10 && W_L_Linecount >= 10 
        && (change_pointY1 <= 4 || change_pointY2 <= 4  || change_pointY3 <= 4)
        && (right_up_flag || left_up_flag))
        || (right_up_flag && left_up_flag && whiteline_flag && (right_down_flag || left_down_flag) && (change_pointY1 <= 4 || change_pointY2 <= 4  || change_pointY3 <= 4)  ) 
        )
        {
            ImageFlag.image_element_cross = 1;
        }
        // std::cout << "change_pointY第一:" << change_pointY1 << std::endl;
        // std::cout << "change_pointY第二:" << change_pointY2 << std::endl;
        // std::cout << "change_pointY第三:" << change_pointY3 << std::endl;
        // std::cout << "左下角点上丢边数（是否大于等于10）:" << W_L_Linecount << std::endl;
        // std::cout << "右下角点上丢边数（是否大于等于10）:" << W_R_Linecount << std::endl;
        // std::cout << "右下flag:" << right_down_flag << std::endl;
        // std::cout << "right_down_x:" << right_down_x << std::endl;
        // std::cout << "right_down_y:" << right_down_y << std::endl;
        // std::cout << "右上flag:" << right_up_flag << std::endl;
        // std::cout << "right_up_x:" << right_up_x << std::endl;                    
        // std::cout << "right_up_y:" << right_up_y << std::endl;
        // std::cout << "左下flag:" << left_down_flag << std::endl;
        // std::cout << "left_down_x:" << left_down_x << std::endl;
        // std::cout << "left_down_y:" << left_down_y << std::endl;
        // std::cout << "左上flag:" << left_up_flag << std::endl;
        // std::cout << "left_up_x:" << left_up_x << std::endl;
        // std::cout << "left_up_y:" << left_up_y << std::endl;
        // std::cout << "十字吗:" << ImageFlag.image_element_cross  << std::endl;

    }
    if(ImageFlag.image_element_cross == 1)
    {
        //消除极端情况下的角点标志位
        if(right_down_flag && right_down_y > 53) right_down_flag = 0;
        if(left_down_flag && left_down_y > 53)left_down_flag = 0;
        if(right_up_x > right_down_x + 5 )right_up_flag = 0;
        if(left_up_x < left_down_x - 5)left_up_flag = 0;

        if(right_down_flag && left_down_flag)
        {
            if(right_down_y <= 45 && left_down_y <= 45){right_up_flag = 0;left_up_flag = 0;}
            if(right_up_flag && left_up_flag)
            {
                slope_k_L = (float)(left_up_x - left_down_x)/(float)(left_up_y - left_down_y);//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - right_down_x)/(float)(right_up_y - right_down_y);//根据路径计算右斜率

                for (int Ysite = left_down_y; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = right_down_y; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_down_y) + right_down_x;                        
                    if(ImageDeal[Ysite].RightBorder >= LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = down_max; Ysite >= up_min; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if(right_up_flag && !left_up_flag)//只能找到右上点
            {
                slope_k_L = (float)(left_down_x - ImageDeal[LCDH - 2].LeftBorder)/(float)(left_down_y - LCDH + 2);//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - right_down_x)/(float)(right_up_y - right_down_y);//根据路径计算右斜率

                for (int Ysite = left_down_y; Ysite > 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = right_down_y; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_down_y) + right_down_x;                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = down_max; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if( !right_up_flag && left_up_flag)//只能找到左上点
            {
                slope_k_L = (float)(left_up_x - left_down_x)/(float)(left_up_y - left_down_y);//根据路径计算左斜率
                slope_k_R = (float)(right_down_x - ImageDeal[LCDH - 2].RightBorder)/(float)(right_down_y - LCDH + 2);//根据路径计算右斜率

                for (int Ysite = right_down_y; Ysite > 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_down_y) + right_down_x;
                   
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = left_down_y; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                
                for (int Ysite = down_max; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if( !right_up_flag && !left_up_flag)//找不到左上点和右上点
            {
                slope_k_L = (float)(left_down_x - ImageDeal[LCDH - 2].LeftBorder)/(float)(left_down_y - (LCDH - 2));//根据路径计算左斜率
                slope_k_R = (float)(right_down_x - ImageDeal[(LCDH - 2)].RightBorder)/(float)(right_down_y - (LCDH - 2));//根据路径计算右斜率

                for (int Ysite = left_down_y; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = right_down_y; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_down_y) + right_down_x;                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = down_max; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            
        }
        else if(right_down_flag && !left_down_flag)
        {
            if(right_up_flag && left_up_flag)
            {
                slope_k_L = (float)(left_up_x - 15)/(float)(left_up_y - 55);//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - right_down_x)/(float)(right_up_y - right_down_y);//根据路径计算右斜率

                for (int Ysite = 58; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - 55) + 15;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = right_down_y; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_down_y) + right_down_x;                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = down_max; Ysite >= up_min; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if(right_up_flag && !left_up_flag)//只能找到右上点
            {
                slope_k_R = (float)(right_up_x - right_down_x)/(float)(right_up_y - right_down_y);//根据路径计算右斜率

                for (int Ysite = right_down_y; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_up_y) + right_up_x);   
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] ;//计算中线
                    if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].LeftBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].RightBorder;//计算右边线
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;

                }
            }
            else if( !right_up_flag && left_up_flag)//只能找到左上点
            {
                if(right_down_y > 53)
                {
                    slope_k_L = (float)(left_up_x - 15)/(float)(left_up_y - 55);//根据路径计算左斜率
                    // slope_k_R = (float)(right_down_x - ImageDeal[(LCDH - 2)].RightBorder)/(float)(right_down_y - (LCDH - 2));//根据路径计算右斜率
                    
                    for (int Ysite = LCDH - 1; Ysite >= left_up_y; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].LeftBorder = (int)(slope_k_L * (Ysite - left_up_y) + left_up_x);
                        if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite]  ;//计算中线
                    }
                    for (int Ysite = LCDH - 1; Ysite > 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].RightBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].LeftBorder;
                       
                        if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                    }
                }
                else
                {
                    slope_k_L = (float)(left_up_x - 15)/(float)(left_up_y - 55);//根据路径计算左斜率
                    slope_k_R = (float)(right_down_x - ImageDeal[(LCDH - 2)].RightBorder)/(float)(right_down_y - (LCDH - 2));//根据路径计算右斜率

                    for (int Ysite = right_down_y + 6; Ysite >= 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_down_y) + right_down_x);   
                        if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                    {
                        ImageDeal[Ysite].LeftBorder = (int)(slope_k_L * (Ysite - left_up_y) + left_up_x);//计算右边线
                        if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;

                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] ;//计算中线
                        if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                    }
                }
                
            }
            else if( !right_up_flag && !left_up_flag)//找不到左上点和右上点
            {
                slope_k_R = (float)(right_down_x - ImageDeal[(LCDH - 2)].RightBorder)/(float)(right_down_y - (LCDH - 2));//根据路径计算右斜率

                for (int Ysite = right_down_y + 6; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_down_y) + right_down_x);   
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] ;//计算中线
                    if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].LeftBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].RightBorder;//计算右边线
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;

                }
            }
        }
        else if(!right_down_flag && left_down_flag)
        {
            if(right_up_flag && left_up_flag)
            {
                slope_k_L = (float)(left_up_x - left_down_x)/(float)(left_up_y - left_down_y);//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - 100)/(float)(right_up_y - 53);//根据路径计算右斜率

                for (int Ysite = left_down_y; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = (int)(slope_k_L * (Ysite - left_down_y) + left_down_x);
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_up_y) + right_up_x);                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= up_min; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if(right_up_flag && !left_up_flag)//只能找到右上点
            {
                if(left_down_y > 53)
                {
                    // slope_k_L = (float)(left_down_x - ImageDeal[(LCDH - 2)].LeftBorder)/(float)(left_down_y - (LCDH - 2));//根据路径计算左斜率
                    slope_k_R = (float)(right_up_x - 100)/(float)(right_up_y - 53);//根据路径计算右斜率
    
                    for (int Ysite = LCDH - 1; Ysite >= right_up_y; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_up_y) + right_up_x);                        
                        if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite]  ;//计算中线
                        if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                    }
                    for (int Ysite = LCDH - 1; Ysite > 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].LeftBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].RightBorder;
                        if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                    }
                }
                else
                {
                    slope_k_R = (float)(right_up_x - 100)/(float)(right_up_y - 53);//根据路径计算右斜率
                    slope_k_L = (float)(left_down_x - ImageDeal[(LCDH - 2)].LeftBorder)/(float)(left_down_y - (LCDH - 2));//根据路径计算左斜率

                    for (int Ysite = left_down_y; Ysite >= 2; Ysite--)         //十字布线
                    {
                        ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                        if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                    {
                        ImageDeal[Ysite].RightBorder = slope_k_R * (Ysite - right_up_y) + right_up_x;
                        if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                    }
                    for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                    {
                        ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder )/2; 
                        if(ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 1;
                    }
                }
            }
            else if( !right_up_flag && left_up_flag)//只能找到左上点
            {
                slope_k_L = (float)(left_up_x - left_down_x)/(float)(left_up_y - left_down_y);//根据路径计算左斜率

                for (int Ysite = left_down_y; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder +  Half_Road_Wide[Ysite]; 
                    if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                {
                    ImageDeal[Ysite].RightBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].LeftBorder ; 
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
            }
            else if( !right_up_flag && !left_up_flag)//找不到左上点和右上点
            {
                slope_k_L = (float)(left_down_x - ImageDeal[(LCDH - 2)].LeftBorder)/(float)(left_down_y - (LCDH - 2));//根据路径计算左斜率

                for (int Ysite = left_down_y; Ysite >= 2; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_down_y) + left_down_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder +  Half_Road_Wide[Ysite]; 
                    if(ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--) 
                {
                    ImageDeal[Ysite].RightBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].LeftBorder ; 
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
            }
        }
        else if(!right_down_flag && !left_down_flag)//找不到左右下点
        {
            if(right_up_flag && left_up_flag)
            {
                slope_k_L = (float)(left_up_x - 15)/(float)(left_up_y - 55);//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - 100)/(float)(right_up_y - 53);//根据路径计算右斜率

                for (int Ysite = LCDH - 1; Ysite >= left_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = (int)(slope_k_L * (Ysite - left_up_y) + left_up_x);
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_up_y) + right_up_x);                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= up_min; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].Center = (int)(ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2 ;//计算中线
                }
            }
            else if(right_up_flag && !left_up_flag)//只能找到右上点
            {
                // slope_k_L = (float)(left_down_x - ImageDeal[(LCDH - 2)].LeftBorder)/(float)(left_down_y - (LCDH - 2));//根据路径计算左斜率
                slope_k_R = (float)(right_up_x - 100)/(float)(right_up_y - 53);//根据路径计算右斜率

                for (int Ysite = LCDH - 1; Ysite >= right_up_y; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].RightBorder = (int)(slope_k_R * (Ysite - right_up_y) + right_up_x);                        
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] ;//计算中线
                    if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].LeftBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].RightBorder ;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
            }
            else if( !right_up_flag && left_up_flag)//只能找到左上点
            {
                slope_k_L = (float)(left_up_x - 15)/(float)(left_up_y - 55);//根据路径计算左斜率

                for (int Ysite = LCDH - 1; Ysite >= left_up_y ; Ysite--)         //十字布线
                {
                    ImageDeal[Ysite].LeftBorder = slope_k_L * (Ysite - left_up_y) + left_up_x;
                    if(ImageDeal[Ysite].LeftBorder < 0) ImageDeal[Ysite].LeftBorder = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder +  Half_Road_Wide[Ysite]; 
                    if(ImageDeal[Ysite].Center < 0) ImageDeal[Ysite].Center = 0;
                }
                for (int Ysite = LCDH - 1; Ysite >= 2; Ysite--)
                {
                    ImageDeal[Ysite].RightBorder = 2*ImageDeal[Ysite].Center - ImageDeal[Ysite].LeftBorder ; 
                    if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 1;
                }
            }
        }

        ///////////////////////////              状态转换关键                   ///////////////////////////////
        // if(ImageStatus.WhiteLine >= 28)flag1 = true;
        // min_up_y = (right_up_y <=  left_up_y ? right_up_y : left_up_y);
        if(down_max > 46 && ImageDeal[46].RightBoundary_First - ImageDeal[46].LeftBoundary_First > 100)flag1 = true;
        if(flag1 && ImageDeal[30].RightBoundary_First - ImageDeal[30].LeftBoundary_First < 55)flag2 = true;
        if(up_min >= 30 || flag2)
        {
            flag1 = false;
            flag2 = false;

            ImageFlag.image_element_cross =0;
            // ImageFlag.image_element_cross_flag =0; 
            ImageStatus.Road_type = Normol;

            right_down_x = 0;
            right_down_y = 0;
            
            left_down_x = 0;
            left_down_y = 0;
            
            right_up_x = 0;
            right_up_y = 0;
            
            left_up_x = 0;
            left_up_y = 0;
        }
        // std::cout << "change_pointY:" << change_pointY << std::endl;
        // std::cout << "上角点y最小值:" << up_min << std::endl;
        // std::cout << "下角点y最大值:" << down_max << std::endl;
        // std::cout << "46行是否大于100:" << ImageDeal[46].RightBoundary_First - ImageDeal[46].LeftBoundary_First << std::endl;
        // std::cout << "30行是否小于55:" << ImageDeal[30].RightBoundary_First - ImageDeal[30].LeftBoundary_First  << std::endl;
        // std::cout << "标志位1:" << flag1 << std::endl;
        // std::cout << "标志位2:" << flag2 << std::endl;
        // std::cout << "左下角点上6行丢边数:" << W_L_Linecount << std::endl;
        // std::cout << "右下角点上6行丢边数:" << W_R_Linecount << std::endl;
        // std::cout << "右下flag:" << right_down_flag << std::endl;
        // std::cout << "right_down_x:" << right_down_x << std::endl;
        // std::cout << "right_down_y:" << right_down_y << std::endl;
        // std::cout << "右上flag:" << right_up_flag << std::endl;
        // std::cout << "right_up_x:" << right_up_x << std::endl;                    
        // std::cout << "right_up_y:" << right_up_y << std::endl;
        // std::cout << "左下flag:" << left_down_flag << std::endl;
        // std::cout << "left_down_x:" << left_down_x << std::endl;
        // std::cout << "left_down_y:" << left_down_y << std::endl;
        // std::cout << "左上flag:" << left_up_flag << std::endl;
        // std::cout << "left_up_x:" << left_up_x << std::endl;
        // std::cout << "left_up_y:" << left_up_y << std::endl;
        // std::cout << "十字吗:" << ImageFlag.image_element_cross  << std::endl;

    }
}
/****圆环检测***/
int Point_Xsite,Point_Ysite;                   //拐点横纵坐标
int Repair_Point_Xsite,Repair_Point_Ysite;     //补线点横纵坐标
int k_count = 0;                               //出环拐点计数
float out_circle_k = 0;                          //出环k
bool firstringFlag = true;

//环岛检测
void elementJudgmentLeftRings()
{
    if( 
    (straightJudge(2, ImageStatus.OFFLine + 5, 55) > 1)
    ||ImageDeal[55].Wide < 70
    ||ImageDeal[50].Wide < 65
    ||ImageDeal[45].Wide < 60
    ||ImageDeal[40].Wide < 55
    ||ImageDeal[35].Wide < 50
    ||ImageStatus.OFFLine >= 5)
    return;

    int ring_ysite = 25;  //环岛检测行
    int right_RingsFlag_Point1_Ysite = 0;
    int right_RingsFlag_Point2_Ysite = 0;

    Left_RingsFlag_Point1_Ysite = 0;
    Left_RingsFlag_Point2_Ysite = 0;

    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    if(right_RingsFlag_Point2_Ysite >= right_RingsFlag_Point1_Ysite + 1)
    return;

    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            Left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)  
        {
            Left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    if(Left_RingsFlag_Point2_Ysite >= Left_RingsFlag_Point1_Ysite + 1 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.LWLine > 7 )//13
            Ring_Help_Flag = 1;
    }
    if (Left_RingsFlag_Point2_Ysite >= Left_RingsFlag_Point1_Ysite + 1 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag ==0)
    {

        ImageFlag.image_element_rings = 1;
        ImageFlag.image_element_rings_flag = 1;
    }
    Ring_Help_Flag = 0;
}

void elementJudgmentRightRings()
{
    if((straightJudge(1, ImageStatus.OFFLine + 5, 55) > 1)
    ||ImageDeal[55].Wide < 70
    ||ImageDeal[50].Wide < 65
    ||ImageDeal[45].Wide < 60
    ||ImageDeal[40].Wide < 55
    ||ImageDeal[35].Wide < 50
    ||ImageStatus.OFFLine >= 5)
    return;

    int ring_ysite = 25;
    int left_RingsFlag_Point1_Ysite = 0;
    int left_RingsFlag_Point2_Ysite = 0;
    Right_RingsFlag_Point1_Ysite = 0;
    Right_RingsFlag_Point2_Ysite = 0;
    
    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].LeftBoundary_First - ImageDeal[Ysite - 1].LeftBoundary_First > 4)
        {
            left_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite + 1].LeftBoundary - ImageDeal[Ysite].LeftBoundary > 4)  
        {
            left_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }

    if(left_RingsFlag_Point2_Ysite >= left_RingsFlag_Point1_Ysite + 1)
      return;

    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite - 1].RightBoundary_First - ImageDeal[Ysite].RightBoundary_First > 4)
        {
            Right_RingsFlag_Point1_Ysite = Ysite;
            break;
        }
    }
    for (int Ysite = 57; Ysite > ring_ysite; Ysite--)
    {
        if (ImageDeal[Ysite].RightBoundary - ImageDeal[Ysite + 1].RightBoundary > 4)
        {
            Right_RingsFlag_Point2_Ysite = Ysite;
            break;
        }
    }


    if(Right_RingsFlag_Point2_Ysite >= Right_RingsFlag_Point1_Ysite + 1 && Ring_Help_Flag == 0)
    {
        if(ImageStatus.RWLine > 7)
            Ring_Help_Flag = 1;
    }


    if (Right_RingsFlag_Point2_Ysite >= Right_RingsFlag_Point1_Ysite + 1 && Ring_Help_Flag == 1 && ImageFlag.image_element_rings_flag == 0)
    {

        ImageFlag.image_element_rings = 2;
        ImageFlag.image_element_rings_flag = 1;  
    }
    Ring_Help_Flag = 0;

}


void leftRingHandle(void)
{
    int num = 0;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsLeftFind == 'W')  //num阈值可以改一改
        {
            num ++;
        }
        if(ImageDeal[Ysite+3].IsLeftFind == 'W' && ImageDeal[Ysite+2].IsLeftFind == 'W'
        && ImageDeal[Ysite+1].IsLeftFind == 'W' && ImageDeal[Ysite].IsLeftFind == 'T')
        {
            break;
        }   
    }

    if (ImageFlag.image_element_rings_flag == 1 && num > 9)
    { 
        ImageStatus.Road_type = LeftCirque;
        ImageFlag.image_element_rings_flag = 2;
    }

    if (ImageFlag.image_element_rings_flag == 2 && num < 2) //5
    {
        ImageFlag.image_element_rings_flag = 5; //快要入环了，差不多在弧度那个位置
    }

    if(ImageFlag.image_element_rings_flag == 5 && /*num>15)*/ImageStatus.RWLine > 15)
    {
        ImageFlag.image_element_rings_flag = 6;//开始进环
    }

    //进圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.RWLine < 7)
    {
        ImageFlag.image_element_rings_flag = 7;
    }

    //环内 
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Ysite = 0;
        Point_Xsite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)  //Ysite > ImageStatus.OFFLine + 3
        {
        //寻找拐点RightBorder
        if (
            (ImageDeal[Ysite - 5].RightBoundary_First - ImageDeal[Ysite + 1].RightBoundary_First >= 5))
        {
            Point_Xsite = ImageDeal[Ysite + 1].RightBorder;
            Point_Ysite = Ysite + 1;
            break;
        }
        }
        if (Point_Ysite >= 40) //拐点在图像的y坐标
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }

    //出环后
    if (ImageFlag.image_element_rings_flag == 8)
    {
        if (straightJudge(2, ImageStatus.OFFLine + 10, 35) < 1 
            && ImageStatus.OFFLine < 8
            && ImageStatus.RWLine <= 10)    //右边为直线且截止行（前瞻值）很小
            {
                ImageFlag.image_element_rings_flag = 9;
            }
    }

    //结束圆环进程
    if (ImageFlag.image_element_rings_flag == 9)
    {
        // int  flag_Xsite_2 = 0;
        int flag_Ysite_2 = 0;
        // float Slope_Rings = 0;

        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite --)//下面弧点
        {
            for(x = ImageDeal[Ysite].RightBorder - 1;x > ImageDeal[Ysite].LeftBorder + 1;x --)
            {
                if(Pixle[Ysite][x + 1] == 1 && Pixle[Ysite][x] == 0 && Pixle[Ysite][x - 1] == 0)
                {
                flag_Ysite_2 = Ysite;
                // flag_Xsite_2 = x;
                // Slope_Rings = (float)(LCDW  - flag_Xsite_2) / (float)(LCDH - 1 - flag_Ysite_2);//79
                break;
                }
            }

            if(flag_Ysite_2 != 0)
            {
                // std::cout << "flag_Ysite_1:" << flag_Ysite_1 << std::endl;

                break;
            }
        }
        if (flag_Ysite_2 > 48) //拐点在图像的y坐标
        {
            ImageStatus.Road_type = Normol;   //出环处理完道路类型清0
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            Point_Xsite = 0;
            Point_Ysite = 0;                   //拐点横纵坐标
            Repair_Point_Xsite = 0;
            Repair_Point_Ysite = 0;     //补线点横纵坐标
            k_count = 0;   
            out_circle_k = 0;
            if(firstringFlag == true)
            {
                std::cout << "第一个环跑完" << std::endl;
            }
            firstringFlag = false;
        }
    }

    /***************************************处理**************************************/
    //准备进环  半宽处理
    if ( ImageFlag.image_element_rings_flag == 1
    || ImageFlag.image_element_rings_flag == 2
    || ImageFlag.image_element_rings_flag == 3
    || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite];//15
        }
    }
        //进环  补线
    if  ( ImageFlag.image_element_rings_flag == 5
        ||ImageFlag.image_element_rings_flag == 6)
    {
        int  flag_Xsite_1 = 0;
        int flag_Ysite_1 = 0;
        float Slope_Rings = 0;

        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite --)//下面弧点
        {
            for(x = ImageDeal[Ysite].RightBorder - 1;x > ImageDeal[Ysite].LeftBorder + 1;x --)
            {
            if(Pixle[Ysite][x + 1] == 1 && Pixle[Ysite][x] == 0 && Pixle[Ysite][x - 1] == 0)
                {
                flag_Ysite_1 = Ysite;
                flag_Xsite_1 = x;
                Slope_Rings = (float)(LCDW  - flag_Xsite_1) / (float)(LCDH - 1 - flag_Ysite_1);//79
                break;
                }
            }

            if(flag_Ysite_1 != 0)
            {
                // std::cout << "flag_Ysite_1:" << flag_Ysite_1 << std::endl;

                break;
            }
        }

        //补线
        if(flag_Ysite_1 != 0)     //如果找到了左上拐点那么就进行补线
        {
        for(Ysite = flag_Ysite_1; Ysite < 59; Ysite ++)//从左上拐点开始向下进行补线
        {
            ImageDeal[Ysite].RightBorder = flag_Xsite_1 + Slope_Rings*(Ysite - flag_Ysite_1);     //y=y0-K(x=x0);从左上拐点补到图像的右下点
            ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;//重新计算中线
            if(ImageDeal[Ysite].Center < 0)
                ImageDeal[Ysite].Center = 0;
        }

        ImageDeal[flag_Ysite_1].RightBorder = flag_Xsite_1;//左上拐点到右下点的补线完成

        for(Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite --) //左上拐点上方进行扫线,不做这步的话图像就成石了
        {
            for(x=ImageDeal[Ysite + 1].RightBorder - 10; x < ImageDeal[Ysite + 1].RightBorder + 2;x ++)
            {
                if(Pixle[Ysite][x] == 1 && Pixle[Ysite][x + 1] == 0)//寻找外环上小弧部分
                {
                    ImageDeal[Ysite].RightBorder = x;
                    ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;//更新中线

                    if(ImageDeal[Ysite].Center < 0)
                        ImageDeal[Ysite].Center = 0;

                    ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;
                    break;
                }
            }

            if(ImageDeal[Ysite].Wide > 8 && ImageDeal[Ysite].RightBorder < ImageDeal[Ysite+2].RightBorder)
            {
                continue;
            }
            else
            {
                ImageStatus.OFFLine = Ysite + 2;    //截止行更新
                break;
            }
        }
        }
    }
    if (ImageFlag.image_element_rings_flag == 7) 
    {
        // std::cout << "Point_Ysite:" << Point_Ysite << std::endl;
        // std::cout << "Point_Xsite:" << Point_Xsite << std::endl;
        if( Point_Ysite >= 19 && Point_Ysite + 8 < 58 && k_count == 0)
        {
            k_count ++;
            Repair_Point_Xsite = ImageDeal[Point_Ysite + 3].RightBorder ;
            Repair_Point_Ysite = Point_Ysite + 3;
            out_circle_k = (Point_Xsite - Repair_Point_Xsite) / (Point_Ysite - Repair_Point_Ysite);
            // std::cout << "out_circle_k:" << out_circle_k << std::endl;
        }
        if(k_count != 0)
        {
            for (int Ysite = Point_Ysite ; Ysite > ImageStatus.OFFLine + 1; Ysite--)
            {
                ImageDeal[Ysite].RightBorder = out_circle_k * (Ysite - Point_Ysite)  + Point_Xsite;
                // ImageDeal[Ysite].RightBorder = LCDW - 1;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 2;
                if( ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 2;
            }

        }

    }

    if (ImageFlag.image_element_rings_flag == 8) 
    {
        // Repair_Point_Xsite = LCDW - 1 ;
        // Repair_Point_Ysite = 0;

        if(k_count != 0)
        {
            for (int Ysite = 57 ; Ysite > ImageStatus.OFFLine + 1; Ysite--)
            {
                ImageDeal[Ysite].RightBorder = out_circle_k * (Ysite - Point_Ysite)  + Point_Xsite;
                // ImageDeal[Ysite].RightBorder = LCDW - 1;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].RightBorder > LCDW - 2) ImageDeal[Ysite].RightBorder = LCDW - 2;
                if( ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 2;
            }

        }
    }
    
    //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 58; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].RightBorder - Half_Road_Wide[Ysite] - 1;
        }
    }

}
void rightRingHandle(void)
{
    /****************判断*****************/
    int num =0 ;
    for (int Ysite = 55; Ysite > 30; Ysite--)
    {
        if(ImageDeal[Ysite].IsRightFind == 'W')
        {
            num++;
        }
        if(ImageDeal[Ysite+3].IsRightFind == 'W' && ImageDeal[Ysite+2].IsRightFind == 'W'
            && ImageDeal[Ysite+1].IsRightFind == 'W' && ImageDeal[Ysite].IsRightFind == 'T' )//3.24
            break;
    }

    #if binMOD == 1
    #endif

        //准备进环
    if (ImageFlag.image_element_rings_flag == 1 && num > 9)
    {    
        ImageStatus.Road_type = RightCirque;
        ImageFlag.image_element_rings_flag = 2;
    }
    if (ImageFlag.image_element_rings_flag == 2 && num < 2)
    {
        ImageFlag.image_element_rings_flag = 5;
    }
        //进环
    if(ImageFlag.image_element_rings_flag == 5 && ImageStatus.LWLine > 15)
    {
        ImageFlag.image_element_rings_flag = 6;
    }
        //进环小圆环
    if(ImageFlag.image_element_rings_flag == 6 && ImageStatus.LWLine < 7)
    {
        ImageFlag.image_element_rings_flag = 7;
    }
    if (ImageFlag.image_element_rings_flag == 7)
    {
        Point_Xsite = 0;
        Point_Ysite = 0;
        for (int Ysite = 50; Ysite > ImageStatus.OFFLine + 3; Ysite--)
        {
            if(
                (ImageDeal[Ysite + 1].LeftBoundary_First - ImageDeal[Ysite - 5].LeftBoundary_First >= 5))
            {
                Point_Xsite = ImageDeal[Ysite + 1].LeftBorder;
                Point_Ysite = Ysite + 1;
                break;
            }
        }

        if (Point_Ysite >= 40)
        {
            ImageFlag.image_element_rings_flag = 8;
        }
    }

    if (ImageFlag.image_element_rings_flag == 8)
    {
        if (   straightJudge(1, ImageStatus.OFFLine + 10, 35) < 1
            && ImageStatus.OFFLine < 8
            && ImageStatus.LWLine <= 10)
        {
            ImageFlag.image_element_rings_flag = 9;
        }
    }

    if(ImageFlag.image_element_rings_flag == 9 )
    {
        int flag_Xsite_2 = 0;
        int flag_Ysite_2 = 0;
        float Slope_Right_Rings = 0;
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(x = ImageDeal[Ysite].LeftBorder + 1; x < ImageDeal[Ysite].RightBorder - 1;x ++)
            {
                if(Pixle[Ysite][x - 1]==1 && Pixle[Ysite][x]==0 && Pixle[Ysite][x + 1]==0)
                {
                    flag_Ysite_2 = Ysite;
                    flag_Xsite_2 = x;
                    Slope_Right_Rings = (float)(0 - flag_Xsite_2) / (float)(LCDH - 1 - flag_Ysite_2);
                    break;
                }
            }
            if(flag_Ysite_2 != 0)
            {
                //  std::cout << "flag_Ysite_2:" << flag_Ysite_2 << std::endl;

                break;
            }
        }
        if(flag_Ysite_2 > 48)
        {
            ImageStatus.Road_type = Normol;   
            ImageFlag.image_element_rings_flag = 0;
            ImageFlag.image_element_rings = 0;
            Point_Xsite = 0;
            Point_Ysite = 0;                   //拐点横纵坐标
            Repair_Point_Xsite = 0;
            Repair_Point_Ysite = 0;     //补线点横纵坐标
            k_count = 0;   
            out_circle_k = 0;
            if(firstringFlag == true)
            {
                std::cout << "第一个环跑完" << std::endl;
            }
            firstringFlag = false;
        }
    }
    /***************************************处理**************************************/
        //准备进环  半宽处理
    if (   ImageFlag.image_element_rings_flag == 1
        || ImageFlag.image_element_rings_flag == 2
        || ImageFlag.image_element_rings_flag == 3
        || ImageFlag.image_element_rings_flag == 4)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite];
        }
    }

        //进环  补线
    if (   ImageFlag.image_element_rings_flag == 5
        || ImageFlag.image_element_rings_flag == 6)
    {
        int flag_Xsite_1 = 0;
        int flag_Ysite_1 = 0;
        float Slope_Right_Rings = 0;
        for(Ysite = 55; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            for(x = ImageDeal[Ysite].LeftBorder + 1; x < ImageDeal[Ysite].RightBorder - 1;x ++)
            {
                if(Pixle[Ysite][x - 1]==1 && Pixle[Ysite][x]==0 && Pixle[Ysite][x + 1]==0)
                {
                    flag_Ysite_1 = Ysite;
                    flag_Xsite_1 = x;
                    Slope_Right_Rings = (float)(0 - flag_Xsite_1) / (float)(LCDH - 1 - flag_Ysite_1);
                    break;
                }
            }
            if(flag_Ysite_1!=0)
            {
                //  std::cout << "flag_Ysite_1:" << flag_Ysite_1 << std::endl;

                break;
            }
        }
        
        //补线
        if(flag_Ysite_1 != 0)
        {
            for(Ysite = flag_Ysite_1; Ysite < 58; Ysite ++) //LCDH - 1
            {
                ImageDeal[Ysite].LeftBorder = flag_Xsite_1 + Slope_Right_Rings * (Ysite - flag_Ysite_1);
                ImageDeal[Ysite].Center=(ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;//板块
                if(ImageDeal[Ysite].Center > LCDW - 1)
                    ImageDeal[Ysite].Center = LCDW - 1;
            }
            ImageDeal[flag_Ysite_1].LeftBorder = flag_Xsite_1;

            for(Ysite = flag_Ysite_1 - 1; Ysite > 10; Ysite --) //A点上方进行扫线
            {
                for(x = ImageDeal[Ysite + 1].LeftBorder + 8; x > ImageDeal[Ysite+1].LeftBorder - 4; x --)
                {
                    if(Pixle[Ysite][x] == 1 && Pixle[Ysite][x - 1] == 0)
                    {
                    ImageDeal[Ysite].LeftBorder = x;
                    ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBorder + ImageDeal[Ysite].RightBorder) / 2;
                    ImageDeal[Ysite].Wide = ImageDeal[Ysite].RightBorder - ImageDeal[Ysite].LeftBorder;

                    if(ImageDeal[Ysite].Center > LCDW - 1)
                        ImageDeal[Ysite].Center = LCDW - 1;
                    //  if(ImageDeal[Ysite].Center < 5)
                    //      ImageDeal[Ysite].Center = 5;
                    break;
                    }
                }

                if(ImageDeal[Ysite].Wide>8 && ImageDeal[Ysite].LeftBorder > ImageDeal[Ysite + 2].LeftBorder)
                {
                    continue;
                }
                else
                {
                    ImageStatus.OFFLine=Ysite + 2;
                    break;
                }
            }
        }
    }
    //出环布线
    if (ImageFlag.image_element_rings_flag == 7) 
    {
        // std::cout << "Point_Ysite:" << Point_Ysite << std::endl;
        // std::cout << "Point_Xsite:" << Point_Xsite << std::endl;
        if( Point_Ysite >= 19 && Point_Ysite + 8 < 58 && k_count == 0)
        {
            k_count ++;
            Repair_Point_Xsite = ImageDeal[Point_Ysite + 3].LeftBorder;
            Repair_Point_Ysite = Point_Ysite + 3;
            out_circle_k = (Point_Xsite - Repair_Point_Xsite) / (Point_Ysite - Repair_Point_Ysite);
            // std::cout << "out_circle_k:" << out_circle_k << std::endl;
        }
        if(k_count != 0)
        {
            for (int Ysite = Point_Ysite ; Ysite > ImageStatus.OFFLine + 1; Ysite--)
            {
                ImageDeal[Ysite].LeftBorder = out_circle_k * (Ysite - Point_Ysite)  + Point_Xsite;
                // ImageDeal[Ysite].RightBorder = LCDW - 1;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].LeftBorder < 2) ImageDeal[Ysite].LeftBorder = 1;
                if( ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 2;
            }

        }

    }

    if (ImageFlag.image_element_rings_flag == 8) 
    {
        // Repair_Point_Xsite = LCDW - 1 ;
        // Repair_Point_Ysite = 0;

        if(k_count != 0)
        {
            for (int Ysite = 57; Ysite > ImageStatus.OFFLine + 1; Ysite--)         //补线   //出环补线,这个是上边界的58列和58行的左边线斜拉补线，不应该是和圆环的右下拐点进行拉线吗？？？
            {
                ImageDeal[Ysite].LeftBorder = out_circle_k * (Ysite - Point_Ysite) + Point_Xsite;//y=（y1-y2）/(x1-x2)*(x-x1)+y1
                // ImageDeal[Ysite].RightBorder = LCDW - 1;
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].RightBorder + ImageDeal[Ysite].LeftBorder) / 2;
                if(ImageDeal[Ysite].LeftBorder < 2) ImageDeal[Ysite].LeftBorder = 1;
                if( ImageDeal[Ysite].Center > LCDW - 2) ImageDeal[Ysite].Center = LCDW - 2;
            }

        }
    }
        //已出环 半宽处理
    if (ImageFlag.image_element_rings_flag == 9)
    {
        for (int Ysite = 57; Ysite > ImageStatus.OFFLine; Ysite--)
        {
            ImageDeal[Ysite].Center = ImageDeal[Ysite].LeftBorder + Half_Road_Wide[Ysite] - 2;

        }
    }
}

void elementTest(void)
{
    ImageStatus.Road_type = Normol;

    // //十字判断
    // if(ImageStatus.Road_type != LeftCirque
    // && ImageStatus.Road_type != RightCirque)
    // {
    //     cross_handle();
    //     if(ImageFlag.image_element_cross == 1)
    //     {
    //         ImageStatus.Road_type = Cross;
    //     }
    // }

    if(ImageFlag.image_element_rings_flag == 0)
    {
        cross_handle();
        if(ImageFlag.image_element_cross == 1)
        {
            ImageStatus.Road_type = Cross;
        }
    }

    //圆环判断
    if(ImageStatus.Road_type != Cross && ImageStatus.Road_type != LeftCirque && ImageStatus.Road_type != RightCirque)//|| ImageStatus.Road_type != LeftCirque || ImageStatus.Road_type != RightCirque
    {
        elementJudgmentLeftRings();
        elementJudgmentRightRings();
        if(ImageFlag.image_element_rings == 1)ImageStatus.Road_type = LeftCirque;
        if(ImageFlag.image_element_rings == 2)ImageStatus.Road_type = RightCirque;
    }
}

void elementHandle(void)
{
    //左圆环
    if(ImageFlag.image_element_rings == 1)
      leftRingHandle();
    //右圆环
    if(ImageFlag.image_element_rings == 2)
      rightRingHandle();
}

float Weighting[10] = {0.9, 0.9, 0.8, 0.8, 0.7, 0.7,  0.6,  0.6,  0.5,  0.5};//10行权重参数大致按照正态分布即可
int speed_now;
int speed_min;

void getDet(int straighet_towpoint, Mat& img)
{
    //straighet_towpoint = json_.params.straight_towpoint;  //太小角可能会打的太早
    float DetTemp = 0;
    int TowPoint = 0;
    float UnitAll = 0;

    //前瞻限定
    if (ImageStatus.Road_type == RightCirque || ImageStatus.Road_type == LeftCirque)
    {   
        #if circleMod == 0
        TowPoint = 30;
        #else
        if(firstringFlag)
            TowPoint = 30;
        else
            TowPoint = 21;
        #endif

        ImageStatus.towPoint = TowPoint;
    }

    else if (ImageStatus.Road_type == Straight)
    {
        TowPoint = straighet_towpoint;
        ImageStatus.towPoint = TowPoint;
    }

    else if(ImageStatus.Road_type == Cross)
    {
        TowPoint = straighet_towpoint;
        ImageStatus.towPoint = TowPoint;
    }
    else
    {
        TowPoint = straighet_towpoint;
        ImageStatus.towPoint = TowPoint;
    }

    if (TowPoint < ImageStatus.OFFLine)
    {
        TowPoint = ImageStatus.OFFLine + 1;
        ImageStatus.towPoint = TowPoint;
    }

    if (TowPoint >= 49)
    {
        TowPoint = 49;
        ImageStatus.towPoint = TowPoint;
    }

    //画线
    for(int i = LCDH - 2; i > ImageStatus.OFFLine; i --)
    {
        if(i != TowPoint)
            circle(img,Point(ImageDeal[i].Center, i), 1, Scalar(255,0,0), -1);
        else
            circle(img,Point(ImageDeal[TowPoint].Center, TowPoint), 1, Scalar(255,255,255), -1);    //画线
    }

    if ((TowPoint - 5) >= ImageStatus.OFFLine) //前瞻取设定前瞻还是可视距离  需要分情况讨论
    {                                          
        for (int Ysite = (TowPoint - 5); Ysite < TowPoint; Ysite++) 
        {
            DetTemp = DetTemp + Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll = UnitAll + Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + 5); Ysite > TowPoint; Ysite--) 
        {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[TowPoint].Center + DetTemp) / (UnitAll + 1);
    } 
    else if (TowPoint > ImageStatus.OFFLine) 
    {
        for (Ysite = ImageStatus.OFFLine; Ysite < TowPoint; Ysite++) 
        {
            DetTemp += Weighting[TowPoint - Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[TowPoint - Ysite - 1];
        }
        for (Ysite = (TowPoint + TowPoint - ImageStatus.OFFLine); Ysite > TowPoint;
            Ysite--) 
        {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[Ysite].Center + DetTemp) / (UnitAll + 1);
    } 
    else if (ImageStatus.OFFLine < 49) 
    {
        for (Ysite = (ImageStatus.OFFLine + 3); Ysite > ImageStatus.OFFLine;
            Ysite--) 
        {
            DetTemp += Weighting[-TowPoint + Ysite - 1] * (ImageDeal[Ysite].Center);
            UnitAll += Weighting[-TowPoint + Ysite - 1];
        }
        DetTemp = (ImageDeal[ImageStatus.OFFLine].Center + DetTemp) / (UnitAll + 1);
    } 
    else
        DetTemp = ImageStatus.Det;                                                     //如果是出现OFFLine>50情况，保持上一次的偏差值

    ImageStatus.Det = DetTemp;                                                      //此时的解算出来的平均图像偏差

}

//画线函数
void trackImg(Mat& img)
{
    //画线
    for(int i = LCDH - 2; i > ImageStatus.OFFLine; i --)
    {
      #if trackMod == 1
        circle(img,Point(ImageDeal[i].LeftBorder, i), 1, Scalar(0,0,255), -1);
        circle(img,Point(ImageDeal[i].RightBorder, i), 1, Scalar(0,255,0), -1);
      #elif trackMod == 2
        circle(img,Point(ImageDeal[i].LeftBoundary, i), 1, Scalar(0,0,255), -1);
        circle(img,Point(ImageDeal[i].RightBoundary, i), 1, Scalar(0,255,0), -1);
      #elif trackMod == 3
        circle(img,Point(ImageDeal[i].LeftBoundary_First, i), 1, Scalar(0,0,255), -1);
        circle(img,Point(ImageDeal[i].RightBoundary_First, i), 1, Scalar(0,255,0), -1);
      #endif
    }
    // circle(img,Point(ImageDeal[39].Center, 39), 1, Scalar(255,0,0), -1);
}

void imageProcess(int scene)
{
    //边界与标志位初始化
    ImageStatus.OFFLine = 2;  
    ImageStatus.WhiteLine = 0;
    for (Ysite = LCDH - 1; Ysite >= ImageStatus.OFFLine; Ysite--) // 2---59 行
    {
      ImageDeal[Ysite].IsLeftFind = 'W';
      ImageDeal[Ysite].IsRightFind = 'W';
      ImageDeal[Ysite].LeftBorder = 0;
      ImageDeal[Ysite].RightBorder = LCDW - 1;   //宽119
      ImageDeal[Ysite].Center = LCDW / 2;        //中心60
    }                     

    pixleFilter(); //噪点滤除
    drawLinesFirst(scene);
    drawLinesProcess();
    searchBorderOTSU(Pixle, LCDH, LCDW, LCDH - 2);
}

void imageProcess2(bool speedflag)
{
    stopProtect();
    elementTest();

    //直道判断        
    if(ImageStatus.Road_type != Cross 
    && ImageStatus.Road_type != LeftCirque 
    && ImageStatus.Road_type != RightCirque
    // && (speedflag == true ||speedflag == false)
    && speedflag == true
    )
    {
        speedCtrl.update(21);//括号内部为前瞻
    }
        // std::cout << "RingFlag is :" << ImageFlag.image_element_rings_flag << std::endl;

    elementHandle();
    
    // std::cout << "道路类型 :" << ImageStatus.Road_type << std::endl;
    // std::cout << "RingFlag is :" << ImageFlag.image_element_rings_flag << std::endl;
    #if binMOD == 1
    // std::cout << "OFFLine:" << ImageStatus.OFFLine << std::endl;
    // std::cout << "OFFLineBoard" << ImageStatus.OFFLineBoundary << std::endl;
    // std::cout << "RWLine:" << ImageStatus.RWLine << std::endl;
    // std::cout << "LWLine:" << ImageStatus.LWLine << std::endl;
    #endif

}
