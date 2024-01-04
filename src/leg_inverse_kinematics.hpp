/*
 * @author BusyBox
 * @date 2024/01/05
 * @brief 机器狗腿部逆运动学
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <utility>
#include "leg.hpp"


class LegSolver
{
public:
    LegSolver(struct LegParam leg_param);
    ~LegSolver();

    std::pair<double, double> solve(cv::Point2d coord_f);

private:
    struct LegParam m_leg_param;

};

LegSolver::LegSolver(struct LegParam leg_param)
{
    this->m_leg_param = leg_param;
}

LegSolver::~LegSolver()
{

}

/*
 * @brief 机器狗腿部逆运动学
 * @param coord_f 接触地面的脚部坐标
 * @return [关节A角度, 关节B角度]
 */
std::pair<double, double> LegSolver::solve(cv::Point2d coord_f)
{
    double beta = std::atan2(-coord_f.y, coord_f.x);
    double l_af = std::hypot(coord_f.x, coord_f.y);
    double alpha = std::acos((std::pow(l_af, 2) + std::pow(m_leg_param.l3, 2) - std::pow(m_leg_param.l5, 2)) / (2 * l_af * m_leg_param.l3));
    double r_a = 2 * CV_PI - alpha - beta; //关节A角度

    cv::Point2d coord_a(0, 0);
    cv::Point2d coord_b(0, m_leg_param.ab);

    //求解E点坐标
    cv::Point2d coord_e(coord_a.x + m_leg_param.l3 * std::cos(r_a),
                        coord_a.y + m_leg_param.l3 * std::sin(r_a));
    //求解D点坐标
    cv::Point2d coord_d = coord_e + (coord_e - coord_f) * (m_leg_param.l4 / m_leg_param.l5);

    double l_ab = m_leg_param.ab;
    double l_ad = std::hypot(coord_a.x - coord_d.x, coord_a.y - coord_d.y);
    double l_bd = std::hypot(coord_b.x - coord_d.x, coord_b.y - coord_d.y);

    double alpha2 = std::acos((std::pow(l_ab, 2) + std::pow(l_bd, 2) - std::pow(l_ad, 2)) / (2 * l_ab * l_bd));
    double beta2 = std::acos((std::pow(l_bd, 2) + std::pow(m_leg_param.l1, 2) - std::pow(m_leg_param.l2, 2)) / (2 * l_bd * m_leg_param.l1));
    double r_b = CV_PI * 3 / 2 - alpha2 - beta2; //关节B角度

    return std::make_pair(r_a, r_b);
}
