#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <matplotlibcpp.h>
#include <stdlib.h>
#include <algorithm>

#define PI 3.14159
class DynWinAlgorithm
{
private:
    typedef struct
    {
        double x = 0;
        double y = 0;
    } Pos;

    struct Weight
    {
        double heading = 1;
        double dist = 3;
        double speed = 3;
    } weight_;

    typedef struct
    {
        double linear_vel_min = 0;
        double linear_vel_max = 0;
        double angular_vel_min = 0;
        double angular_vel_max = 0;

    } SpdRange;

    typedef struct
    {
        double pos_x = 0;
        double pos_y = -1.75;
        double heading = 0;
        double linear_vel = 2;
        double angular_vel = 0;
    } VehState;

    typedef struct
    {
        double linear_vel = 0;
        double angular_vel = 0;
        std::vector<VehState> trajectory;
        double heading_value;
        double dist_value;
        double vel_value;
        double value = 0;
    } WindowInfo;

    typedef struct
    {
        double center_x = 0;
        double center_y = 0;
        double radius = 0;
    } Obstacle;

    // 场景基本参数
    double road_width_ = 3.5; // 道路标准宽度
    double road_len_ = 60;    // 道路长度
    double veh_width_ = 1.6;  // 汽车宽度
    double veh_len_ = 4.5;    // 车长
    Pos goal_pos_;            // 目标点位置


    // 时间参数
    double dt_ = 0.1;        // 车辆单步运动时间
    double window_time_ = 3; // 窗口时间

    // 自车运动学模型参数
    double v_max_ = 15;                 // 最高速度
    double omega_max_ = 200 * PI / 180; // 最高角速度
    double acc_max_ = 3;                // 最高加速度
    double alpha_max_ = 50 * PI / 180;  // 最高角加速度
    double v_res_ = 0.1;                // 速度分辨率
    double omega_res_ = 2 * PI / 180;   // 角速度分辨率

    // 其他变量
    VehState veh_state_;
    std::vector<WindowInfo> window_info_;
    std::vector<Obstacle> obstacles_;
    std::vector<double> path_x;
    std::vector<double> path_y;

public:
    DynWinAlgorithm(/* args */){};
    ~DynWinAlgorithm(){};

    // 主函数
    bool Excute();

    // 设置障碍物环境
    void SetObstacles();

    // 根据当前状态和约束计算当前的速度参数允许范围
    SpdRange GetSpdRange();

    // 获取窗口信息
    void GetWindowInfo(DynWinAlgorithm::SpdRange spd_range);

    // 更新自车状态
    void UpdateState(int max_value_idx);

    // 找到最大评价函数对应的索引
    int FindMaxValue();

    // 获取本时间窗内的轨迹
    void GetTrajectory(std::vector<VehState> *trajectory, double linear_vel, double angular_vel);

    // 画图
    void Plot(bool to_goal_dist_flag, int max_value_idx);

    // 获取自车几个点位
    void GetEgoPoint(std::vector<double> *x_list, std::vector<double> *y_list);
};
