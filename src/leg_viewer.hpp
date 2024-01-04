/*
 * @author BusyBox
 * @date 2024/01/05
 * @brief 机器狗腿部可视化
 */

#pragma once

#include <opencv2/opencv.hpp>
#include "leg.hpp"

class LegViewer
{
public:
    LegViewer(cv::Size window_size, struct LegParam leg_param);
    ~LegViewer();

    cv::Mat get_view();


private:
    cv::Size m_window_size;
    struct LegParam m_leg_param;
    double x_scale_factor; //水平方向缩放系数
    double y_scale_facotr; //垂直方向缩放系数

    double calc_x_scale_factor();
    double calc_y_scale_facotr();

    cv::Point2d xy_coord_to_screen_coord(cv::Point2d xy_coord);

    void draw_line(cv::Mat view, cv::Point2d start, cv::Point2d end, cv::Scalar color, int thickness = 1);
    void draw_xy_coord(cv::Mat view);

};

LegViewer::LegViewer(cv::Size window_size, struct LegParam leg_param)
{
    this->m_window_size = window_size;
    this->m_leg_param = leg_param;
    x_scale_factor = calc_x_scale_factor();
    y_scale_facotr = calc_y_scale_facotr();
    x_scale_factor = y_scale_facotr = std::min(x_scale_factor, y_scale_facotr);
}

LegViewer::~LegViewer()
{

}

/*
 * @brief 获取腿的可视化图像，正运动学
 */
cv::Mat LegViewer::get_view()
{
    cv::Mat view(m_window_size, CV_8UC3);
    draw_xy_coord(view);

    //求解C点坐标
    cv::Point2d coord_b(0, m_leg_param.ab);
    cv::Point2d coord_c(coord_b.x + m_leg_param.l1 * std::cos(m_leg_param.r_b), 
                        coord_b.y + m_leg_param.l1 * std::sin(m_leg_param.r_b));
    //draw l1
    draw_line(view, coord_b, coord_c, cv::Scalar(255, 0, 0), 3);

    //求解E点坐标
    cv::Point2d coord_a(0, 0);
    cv::Point2d coord_e(coord_a.x + m_leg_param.l3 * std::cos(m_leg_param.r_a), 
                        coord_a.y + m_leg_param.l3 * std::sin(m_leg_param.r_a));
    //draw l3
    draw_line(view, coord_a, coord_e, cv::Scalar(255, 0, 0), 3);

    double l_ce = std::hypot(coord_e.x - coord_c.x, coord_e.y - coord_c.y);
    double l_be = std::hypot(coord_b.x - coord_e.x, coord_b.y - coord_e.y);
    //无解
    if (l_ce >= m_leg_param.l2 + m_leg_param.l4) {
        throw std::runtime_error{"l_ce >= m_leg_param.l2 + m_leg_param.l4"};
    }

    //求解D点坐标
    double alpha = std::acos((std::pow(m_leg_param.l1, 2) + std::pow(l_ce, 2) - std::pow(l_be, 2)) / (2 * m_leg_param.l1 * l_ce));
    double beta = std::acos((std::pow(m_leg_param.l2, 2) + std::pow(l_ce, 2) - std::pow(m_leg_param.l4, 2)) / (2 * m_leg_param.l2 * l_ce));
    double l_bd = std::sqrt(std::pow(m_leg_param.l1, 2) + std::pow(m_leg_param.l2, 2) - 2 * m_leg_param.l1 * m_leg_param.l2 * std::cos(alpha + beta));
    double gamma = std::asin(m_leg_param.l2 * std::sin(alpha + beta) / l_bd);
    cv::Point2d coord_d(coord_b.x + l_bd * std::cos(m_leg_param.r_b + gamma), 
                        coord_b.y + l_bd * std::sin(m_leg_param.r_b + gamma));
    //draw l2
    draw_line(view, coord_c, coord_d, cv::Scalar(255, 0, 0), 3);
    //draw l4
    draw_line(view, coord_d, coord_e, cv::Scalar(255, 0, 0), 3);

    //求解F点坐标
    double dx = coord_e.x - coord_d.x;
    double dy = coord_e.y - coord_d.y;
    double scale_factor = m_leg_param.l5 / m_leg_param.l4;
    cv::Point2d coord_f(coord_e.x + dx * scale_factor, coord_e.y + dy * scale_factor);
    //draw l5
    draw_line(view, coord_e, coord_f, cv::Scalar(255, 0, 0), 3);

    //显示F点坐标
    char hint_f[64];
    snprintf(hint_f, 64, "(x:%.2f, y:%.2f)", coord_f.x, coord_f.y);
    cv::putText(view, hint_f, xy_coord_to_screen_coord(coord_f), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    //显示关节A角度
    char hint_a[64];
    snprintf(hint_a, 64, "(degree:%.2f)", m_leg_param.r_a / CV_PI * 180);
    cv::putText(view, hint_a, xy_coord_to_screen_coord(coord_a), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    //显示关节B角度
    char hint_b[64];
    snprintf(hint_b, 64, "(degree:%.2f)", m_leg_param.r_b / CV_PI * 180);
    cv::putText(view, hint_b, xy_coord_to_screen_coord(coord_b), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);


    return view;
}

double LegViewer::calc_x_scale_factor()
{
    return (m_window_size.width / 2) / (m_leg_param.l4 + m_leg_param.l5);
}

double LegViewer::calc_y_scale_facotr()
{
    return (m_window_size.height / 2) / (m_leg_param.ab + m_leg_param.l3 + m_leg_param.l5);
}

/*
 * @brief 将以关节A为原点的XY坐标系转换成屏幕坐标系
 */
cv::Point2d LegViewer::xy_coord_to_screen_coord(cv::Point2d xy_coord)
{
    cv::Point2d screen_coord;
    screen_coord.x = xy_coord.x * x_scale_factor + m_window_size.width / 2;
    screen_coord.y = -xy_coord.y * y_scale_facotr + m_window_size.height / 2;
    return screen_coord;
}

void LegViewer::draw_line(cv::Mat view, cv::Point2d start, cv::Point2d end, cv::Scalar color, int thickness)
{
    start = xy_coord_to_screen_coord(start);
    end = xy_coord_to_screen_coord(end);

    cv::line(view, start, end, color, thickness);
}

void LegViewer::draw_xy_coord(cv::Mat view)
{
    draw_line(view, cv::Point2d(0, 0), cv::Point2d(50, 0), cv::Scalar(0, 0, 255), 2);
    draw_line(view, cv::Point2d(0, 0), cv::Point2d(0, 50), cv::Scalar(0, 255, 0), 2);
}