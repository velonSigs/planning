#include "dynamic_window_algorithm.h"

// 主程序
bool DynWinAlgorithm::Excute()
{
    goal_pos_.x = 57;
    goal_pos_.y = 2.5;

    // 设置障碍物环境
    DynWinAlgorithm::SetObstacles();
    bool to_goal_dist_flag = false;

    for (size_t i = 0; i < 1000; i++)
    {
        // 根据当前状态和约束计算当前的速度参数允许范围
        DynWinAlgorithm::SpdRange spd_range = GetSpdRange();

        // 根据速度范围和分辨率，生成若干条运动轨迹
        DynWinAlgorithm::GetWindowInfo(spd_range);

        // 找到最大评价函数对应的索引
        int max_value_idx = DynWinAlgorithm::FindMaxValue();

        // 更新自车状态
        DynWinAlgorithm::UpdateState(max_value_idx);

        // 判断是否到达终点，并画图
        std::cout << "iter: " << i << "; pos_x: " << veh_state_.pos_x << "; pos_y: " << veh_state_.pos_y
                  << "; heading: " << veh_state_.heading << "; linear_vel: " << veh_state_.linear_vel
                  << "; angular_vel: " << veh_state_.angular_vel << std::endl;
        double to_goal_dist = std::sqrt(std::pow(veh_state_.pos_x - goal_pos_.x, 2) +
                                        std::pow(veh_state_.pos_y - goal_pos_.y, 2));
        if (to_goal_dist < 1)
        {
            std::cout << "finish !" << std::endl;
            to_goal_dist_flag = true;
            DynWinAlgorithm::Plot(to_goal_dist_flag, max_value_idx);
            break;
        }
        DynWinAlgorithm::Plot(to_goal_dist_flag, max_value_idx);
    }
    return true;
}

void DynWinAlgorithm::SetObstacles()
{
    DynWinAlgorithm::Obstacle obstacle_tmp;
    // 1
    obstacle_tmp.center_x = 13.0;
    obstacle_tmp.center_y = -1.75;
    obstacle_tmp.radius = 2.0;
    obstacles_.push_back(obstacle_tmp);

    // 2
    obstacle_tmp.center_x = 27.0;
    obstacle_tmp.center_y = -1.75;
    obstacle_tmp.radius = 2.0;
    obstacles_.push_back(obstacle_tmp);

    // 3
    obstacle_tmp.center_x = 40.0;
    obstacle_tmp.center_y = 1.75;
    obstacle_tmp.radius = 2.0;
    obstacles_.push_back(obstacle_tmp);

    // 4
    obstacle_tmp.center_x = 50.0;
    obstacle_tmp.center_y = -1.75;
    obstacle_tmp.radius = 2.0;
    obstacles_.push_back(obstacle_tmp);

    // 将道路便捷模拟为若干个小的障碍物，下边界
    for (size_t i = 0; i < 120; i++)
    {
        obstacle_tmp.center_x = i * 0.5;
        obstacle_tmp.center_y = 3.5;
        obstacle_tmp.radius = 0.5;
        obstacles_.push_back(obstacle_tmp);
    }

    // 将道路便捷模拟为若干个小的障碍物，上边界
    for (size_t i = 0; i < 120; i++)
    {
        obstacle_tmp.center_x = i * 0.5;
        obstacle_tmp.center_y = -3.5;
        obstacle_tmp.radius = 0.5;
        obstacles_.push_back(obstacle_tmp);
    }
}

DynWinAlgorithm::SpdRange DynWinAlgorithm::GetSpdRange()
{
    DynWinAlgorithm::SpdRange spd_range;
    spd_range.linear_vel_min = std::max(veh_state_.linear_vel - acc_max_ * dt_, 0.0);
    spd_range.linear_vel_max = std::min(veh_state_.linear_vel + acc_max_ * dt_, v_max_);
    spd_range.angular_vel_min = std::max(veh_state_.angular_vel - alpha_max_ * dt_, -omega_max_);
    spd_range.angular_vel_max = std::min(veh_state_.angular_vel + alpha_max_ * dt_, omega_max_);
    return spd_range;
}

void DynWinAlgorithm::GetWindowInfo(DynWinAlgorithm::SpdRange spd_range)
{
    window_info_.clear(); //清空
    DynWinAlgorithm::WindowInfo window_info_tmp;
    std::vector<VehState> trajectory;

    double linear_vel = spd_range.linear_vel_min;
    while (true)
    {
        double angular_vel = spd_range.angular_vel_min;
        while (true)
        {
            // 初始化轨迹
            trajectory.clear();
            GetTrajectory(&trajectory, linear_vel, angular_vel);

            // 赋值
            window_info_tmp.linear_vel = linear_vel;
            window_info_tmp.angular_vel = angular_vel;
            window_info_tmp.trajectory = trajectory;
            window_info_.push_back(window_info_tmp);

            // 判断是否退出循环
            angular_vel = angular_vel + omega_res_;
            if (angular_vel >= spd_range.angular_vel_max)
            {
                break;
            }
        }
        // 判断是否退出循环
        linear_vel = linear_vel + v_res_;
        if (linear_vel >= spd_range.linear_vel_max)
        {
            break;
        }
    }
}

void DynWinAlgorithm::GetTrajectory(std::vector<DynWinAlgorithm::VehState> *trajectory, double linear_vel, double angular_vel)
{
    DynWinAlgorithm::VehState veh_state = veh_state_;
    trajectory->push_back(veh_state);

    // 循环获得窗口时间内的轨迹
    double time = 0;
    while (time <= window_time_)
    {
        time = time + dt_; // 时间更新

        Eigen::Matrix<double, 5, 5> A;
        A << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0;

        Eigen::Matrix<double, 5, 2> B;
        B << dt_ * std::cos(veh_state.heading), 0,
            dt_ * std::sin(veh_state.heading), 0,
            0, dt_,
            1, 0,
            0, 1;

        Eigen::Matrix<double, 5, 1> x;
        x << veh_state.pos_x,
            veh_state.pos_y,
            veh_state.heading,
            veh_state.linear_vel,
            veh_state.angular_vel;

        Eigen::Matrix<double, 2, 1> u;
        u << linear_vel,
            angular_vel;

        // 状态更新
        Eigen::Matrix<double, 5, 1> state_new;
        state_new = A * x + B * u;

        // 更新state指针
        veh_state.pos_x = state_new(0);
        veh_state.pos_y = state_new(1);
        veh_state.heading = state_new(2);
        veh_state.linear_vel = state_new(3);
        veh_state.angular_vel = state_new(4);

        trajectory->push_back(veh_state);
    }
}

int DynWinAlgorithm::FindMaxValue()
{
    double heading_value_sum = 0;
    double dist_value_sum = 0;
    double vel_value_sum = 0;
    for (std::vector<DynWinAlgorithm::WindowInfo>::iterator iter = window_info_.begin(); iter != window_info_.end();)
    {
        DynWinAlgorithm::VehState end_state = iter->trajectory[iter->trajectory.size() - 1];

        // 计算航向角评价函数
        double theta = end_state.heading * 180 / PI;
        double goal_theta = std::atan2(goal_pos_.y - end_state.pos_y, goal_pos_.x - end_state.pos_x) * 180 / PI;
        double target_theta = std::abs(goal_theta - theta);
        double heading_value = 180 - target_theta;

        // 计算轨迹终点距离最近障碍物距离的评价函数
        double dist_value = 1e5;
        for (size_t i = 0; i < obstacles_.size(); i++)
        {
            double dist = std::sqrt(std::pow(end_state.pos_x - obstacles_[i].center_x, 2) +
                                    std::pow(end_state.pos_y - obstacles_[i].center_y, 2)) -
                          obstacles_[i].radius;
            dist_value = std::min(dist_value, dist);
        }
        if (dist_value < 0)
        {
            iter = window_info_.erase(iter);
            continue;
        }
        iter->dist_value = dist_value;

        // 计算速度的评价函数
        double vel_value = end_state.linear_vel;
        double stop_dist = std::pow(end_state.linear_vel, 2) / (2 * acc_max_);
        if (dist_value < stop_dist)
        {
            iter = window_info_.erase(iter);
            continue;
        }
        iter->vel_value = vel_value;

        // 计算累计值，用于归一化处理
        heading_value_sum = heading_value_sum + heading_value;
        dist_value_sum = dist_value_sum + dist_value;
        vel_value_sum = vel_value_sum + vel_value;

        iter++;
    }

    // 归一化处理
    int idx = 0;
    double max_value = 0;
    for (size_t i = 0; i < window_info_.size(); i++)
    {
        double heading_value_tmp = window_info_[i].heading_value / std::max(heading_value_sum, 0.01);
        double dist_value_tmp = window_info_[i].dist_value / std::max(dist_value_sum, 0.01);
        double vel_value_tmp = window_info_[i].vel_value / std::max(vel_value_sum, 0.01);
        window_info_[i].value = heading_value_tmp * weight_.heading +
                                dist_value_tmp * weight_.dist + vel_value_tmp * weight_.speed;
        window_info_[i].value > max_value ? idx = i : idx = idx;
    }
    return idx;
}

void DynWinAlgorithm::UpdateState(int max_value_idx)
{
    std::vector<DynWinAlgorithm::WindowInfo>::iterator iter = window_info_.begin() + max_value_idx;
    veh_state_ = *(iter->trajectory.begin() + 1);
    path_x.push_back(veh_state_.pos_x);
    path_y.push_back(veh_state_.pos_y);
}

void DynWinAlgorithm::Plot(bool to_goal_dist_flag, int max_value_idx)
{
    matplotlibcpp::clf();
    matplotlibcpp::title("DWA");
    matplotlibcpp::xlabel("X-axis");
    matplotlibcpp::ylabel("Y-axis");

    // 画灰色路面
    std::vector<double> x_list = {-5, -5, road_len_, road_len_};
    std::vector<double> y_list = {-road_width_ - 0.5, road_width_ + 0.5,
                                  road_width_ + 0.5, -road_width_ - 0.5};
    std::map<std::string, std::string> keywords = {{"color", "0.55"}};
    matplotlibcpp::fill(x_list, y_list, keywords);

    // 画车道线
    x_list = {-5, road_len_};
    y_list = {0, 0};
    keywords = {{"linestyle", "dashed"}, {"color", "w"}};
    matplotlibcpp::plot(x_list, y_list, keywords);

    x_list = {-5, road_len_};
    y_list = {road_width_, road_width_};
    matplotlibcpp::plot(x_list, y_list, "w");

    x_list = {-5, road_len_};
    y_list = {-road_width_, -road_width_};
    matplotlibcpp::plot(x_list, y_list, "w");

    // 画交通车辆
    for (size_t i = 0; i < 4; i++)
    {
        x_list = {obstacles_[i].center_x, obstacles_[i].center_x,
                  obstacles_[i].center_x - veh_len_, obstacles_[i].center_x - veh_len_};
        y_list = {obstacles_[i].center_y - veh_width_ / 2, obstacles_[i].center_y + veh_width_ / 2,
                  obstacles_[i].center_y + veh_width_ / 2, obstacles_[i].center_y - veh_width_ / 2};
        keywords = {{"color", "b"}};
        matplotlibcpp::fill(x_list, y_list, keywords);
    }

    // 画自车
    GetEgoPoint(&x_list, &y_list);
    keywords = {{"color", "r"}};
    matplotlibcpp::fill(x_list, y_list, keywords);

    // 画窗口轨迹簇
    for (size_t i = 0; i < window_info_.size(); i++)
    {
        x_list.clear();
        y_list.clear();
        for (size_t j = 0; j < window_info_[i].trajectory.size(); j++)
        {
            x_list.push_back(window_info_[i].trajectory[j].pos_x);
            y_list.push_back(window_info_[i].trajectory[j].pos_y);
        }
        matplotlibcpp::plot(x_list, y_list, "g");
    }

    // 画本窗口内的最优轨迹
    x_list.clear();
    y_list.clear();
    for (size_t i = 0; i < window_info_[max_value_idx].trajectory.size(); i++)
    {
        x_list.push_back(window_info_[max_value_idx].trajectory[i].pos_x);
        y_list.push_back(window_info_[max_value_idx].trajectory[i].pos_y);
    }
    matplotlibcpp::plot(x_list, y_list, "r");

    // 画历史轨迹
    matplotlibcpp::plot(path_x, path_y, "y");

    // 限制横纵范围
    matplotlibcpp::xlim(-5.0, road_len_);
    matplotlibcpp::ylim(-4.0, 4.0);

    // 设置横纵比例
    matplotlibcpp::set_aspect(1);

    // 若还未到终点，则每一次执行结束后停顿0.1s
    if (to_goal_dist_flag == false)
    {
        matplotlibcpp::show(false); // 阻塞标志位置为false
        matplotlibcpp::pause(0.1);
    }
    else
    {
        matplotlibcpp::show(true);
    }

// 保存图片
matplotlibcpp::save("./dwa.png");

}

void DynWinAlgorithm::GetEgoPoint(std::vector<double> *x_list, std::vector<double> *y_list)
{
    x_list->clear();
    y_list->clear();

    // 车头中心点
    double front_posx = veh_state_.pos_x;
    double front_posy = veh_state_.pos_y;
    double heading = veh_state_.heading;

    // 根据车头中心点位置和航向角计算车尾中心点位置
    double rear_posx = front_posx - veh_len_ * std::cos(heading);
    double rear_posy = front_posy - veh_len_ * std::sin(heading);

    // 根据前后中心点、航向角计算目障碍车四个轮廓点位置（顺时针编号）
    x_list->push_back(rear_posx - veh_width_ / 2 * std::sin(heading));
    y_list->push_back(rear_posy + veh_width_ / 2 * std::cos(heading));
    x_list->push_back(front_posx - veh_width_ / 2 * std::sin(heading));
    y_list->push_back(front_posy + veh_width_ / 2 * std::cos(heading));
    x_list->push_back(front_posx + veh_width_ / 2 * std::sin(heading));
    y_list->push_back(front_posy - veh_width_ / 2 * std::cos(heading));
    x_list->push_back(rear_posx + veh_width_ / 2 * std::sin(heading));
    y_list->push_back(rear_posy - veh_width_ / 2 * std::cos(heading));
}
