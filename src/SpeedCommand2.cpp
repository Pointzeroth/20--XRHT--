// SpeedController.h
#pragma once
#include <algorithm>
#include <cstdint>
#include "../include/ImagePrc.h"

extern uint8_t Pixle[LCDH][LCDW];   //包黑边了，注意下

//注意考虑坡道。。。。。坡道到时候交给ai元素识别了，先不管
//第一种曲率加长直道
class SpeedController {
    public:
        void update(int towpoint)
        {
            // float current_curv = getcurv();
            // int current_wcolerror = calculate_wcolerror();
            int cols60_change_Y_point = search_cols_change_point(57);      //要根据中线位置进行调整
            
            int test_towpoint_min = towpoint - 15;
            int test_towpoint_max = towpoint + 15;
            bool leftline_flag = calculate_woline(true);
            bool rightline_flag = calculate_woline(false);

            //限幅
            test_towpoint_min = test_towpoint_min < 9 ? 9 : test_towpoint_min;
            test_towpoint_max = test_towpoint_max > 45 ? 45 : test_towpoint_max;

            if(cols60_change_Y_point < 10 && 
                leftline_flag == true && 
                rightline_flag == true && 
                straightJudge(1,test_towpoint_min,test_towpoint_max) < 1 &&
                straightJudge(2,test_towpoint_min,test_towpoint_max) < 1 &&
                (ImageStatus.Road_type == Normol || ImageStatus.Road_type == Straight) ) 
                    ImageStatus.Road_type = Straight;
            else ImageStatus.Road_type = Normol;
            //std::cout << "Road:" << (ImageStatus.Road_type) << std::end;
        }
    
        float getcurv(){
            float curv = calculate_curvature();//计算曲率
            return curv;
        }
    
        bool calculate_woline(bool flag)
        {
            int count = 0;
            if(flag == true){
                for(int i = 15 ; i <= 45 ; i ++ ){
                    if(ImageDeal[i].IsLeftFind == 'T'){
                        count ++;
                    }
                }
            }
            else{
                for(int i = 15 ; i <= 45 ; i ++ ){
                    if(ImageDeal[i].IsRightFind == 'T'){
                        count ++;
                    }
                }
            }
            if(count > 28)return true;
            else return false;
        }
        
        static constexpr float ROW_SPACING = 1.0f; // 假设每行对应5cm物理距离！！！！具体对应实际多少距离还需要调
        static constexpr int CAL_ROWS = 20;        // 使用20行数据计算
        static constexpr int CAL_COLS_one = 53;        // 使用 第几列 数据计算
        static constexpr int CAL_OFFSET_two = 67;      // 使用 第几列 数据计算
        int calculate_wcolerror()
        {
            
            int center_left7col_change=0;
            int center_right7col_change=0;
            int Wcolerror=0;
            
            bool center_left7col_flag = false;
            bool center_right7col_flag = false;

            for(int i=50;i>5;i-=4){
                if(Pixle[i][CAL_COLS_one]==1&&Pixle[i - 4][CAL_COLS_one]==0){
                    for(int y=0;y<10;y++){
                        if(Pixle[i-y][CAL_COLS_one]==1&&Pixle[i-y-1][CAL_COLS_one]==0){
                            center_left7col_change=i;
                            //记录下来中点左的跳变行
                            center_left7col_flag = true;
                            // std::cout << "中心第55列跳变点:" << center_left5col_change << std::endl;

                            break;
                        }
                    }
                    if(center_left7col_flag) break;
                }
            }
            for(int i=50;i>5;i-=4){
                if(Pixle[i][CAL_OFFSET_two]==1&&Pixle[i - 4][CAL_OFFSET_two]==0){
                    for(int y=0;y<10;y++){
                        if(Pixle[i-y][CAL_OFFSET_two]==1&&Pixle[i-y-1][CAL_OFFSET_two]==0){
                            center_right7col_change=i;
                            //记录下来中点右的跳变行
                            center_right7col_flag = true;
                            // std::cout << "中心第65列跳变点:" << center_right5col_change << std::endl;
                            break;
                        }
                    }
                    if(center_right7col_flag)break;
                }
            }
            Wcolerror=abs(center_left7col_change - center_right7col_change);
            // std::cout << "第55和第60的跳变点Y差值:" << Wcolerror << std::endl;

            return Wcolerror;
        }
        int search_cols_change_point(int n)
        {
            int center_change=0;
            // bool center_flag = false;
            for(int i = 45 ; i >= 2 ; i -- ){
                if(Pixle[i][n]==1&&Pixle[i - 1][n]==0){
                    center_change=i;
                    //记录下来中点的跳变行
                    // center_flag = true;
                    // std::cout << "中心第60列跳变点:" << center_change << std::endl;
                    break;
                }
            }
            return center_change;
        }
        float calculate_curvature()
        {
            // 获取最近的30-50一共20行中线数据（从下往上）
            float y[CAL_ROWS];
            for(int i = 0; i < CAL_ROWS; ++i) {
                y[i] = ImageDeal[LCDH - 10 - i].Center;  //获取图像30-50行的数据
            }
            
            //一阶导数计算（中心差分法） 
            float dy[CAL_ROWS - 2] = {0};  // 存储13个一阶导数值
            for(int i = 1; i < CAL_ROWS-1; ++i) {
                dy[i-1] = (y[i+1] - y[i-1]) / (2 * ROW_SPACING);
            }
    
            // 二阶导数计算（中心差分法）
            float ddy[CAL_ROWS - 4] = {0};  // 存储11个二阶导数值
            for(int i = 1; i < CAL_ROWS-3; ++i) {
                ddy[i-1] = (dy[i+1] - dy[i-1]) / (2 * ROW_SPACING);
            }
    
            // 曲率计算与平均 
            float curvature_sum = 0.0f;
            const int valid_points = CAL_ROWS - 4;
            
            for(int i = 0; i < valid_points; ++i) {
                // 曲率公式：κ = |d²y/dx²| / (1 + (dy/dx)^2)^1.5
                float denominator = powf(1.0f + dy[i+1]*dy[i+1], 1.5f);
                curvature_sum += fabsf(ddy[i]) / (denominator + 1e-6f);  // 加小值防止除零
            }
    
            return 100*curvature_sum / valid_points;  // 返回平均曲率,放大曲率100倍，根据不同曲率判断出是否为长直道或者为缓弯急弯
        }
    };
    /*
    speed=300.0f * expf(-curv * 0.4f) * (1.0f - fabsf(midline_offset)/30.0f);
    车辆速度衰减模型参数解析
    300为小车基准速度（每种元素基准速度可以调整为不同）
    speed为期望速度，上位机可以作为期望速度传给下位机
    curv为赛道曲率，由上位机计算得到，正值为左弯，负值为右弯
    0.4f为曲率衰减系数（控制曲率对速度的敏感程度）
    midline_offset为小车偏离赛道中线的距离，正值为右偏，负值为左偏
    30为最大允许偏移阈值（当偏移值达到该值时，车辆速度为零，自动停止）也可达到保护的目的
    */ 
    // // 带限幅的速度更新
    // float delta = speed - current_speed;
    // //const float delta = std::clamp(speed - current_speed, 80.0f, 120.0f);//限制一个值在120到80之间
    // if(delta < 50)
    //     delta = 50;
    // if(delta > 300)
    //     delta = 300;
    // return current_speed += delta;