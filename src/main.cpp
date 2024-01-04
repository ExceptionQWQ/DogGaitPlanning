/*
 * @author BusyBox
 * @date 2024/01/05
 */

#include <iostream>
#include <opencv2/opencv.hpp>

#include "leg_viewer.hpp"
#include "leg_inverse_kinematics.hpp"

int main()
{
    LegParam leg_param = {.l1 = 26, .l2 = 56, .l3 = 56, .l4 = 26, .l5 = 56, .ab = 25, .r_a = CV_PI / 180 * 200, .r_b = CV_PI / 180 * 150};
    LegViewer leg_viewer(cv::Size(700, 700), leg_param);
    LegSolver leg_solver(leg_param);
    while (true) {
        cv::Point2d p1(30, -100);
        for (int i = 0; i < 25; ++i) {
            p1.y += 1;
            auto ret = leg_solver.solve(p1);
            leg_viewer.set_r_ab(ret.first, ret.second);
            cv::Mat view = leg_viewer.get_view();
            cv::imshow("LegInverseKinematicsDemo", view);
            cv::waitKey(1);
        }
        cv::Point2d p2(30, -75);
        for (int i = 0; i < 60; ++i) {
            p2.x -= 1;
            auto ret = leg_solver.solve(p2);
            leg_viewer.set_r_ab(ret.first, ret.second);
            cv::Mat view = leg_viewer.get_view();
            cv::imshow("LegInverseKinematicsDemo", view);
            cv::waitKey(1);
        }
        cv::Point2d p3(-30, -75);
        for (int i = 0; i < 25; ++i) {
            p3.y -= 1;
            auto ret = leg_solver.solve(p3);
            leg_viewer.set_r_ab(ret.first, ret.second);
            cv::Mat view = leg_viewer.get_view();
            cv::imshow("LegInverseKinematicsDemo", view);
            cv::waitKey(1);
        }
        cv::Point2d p4(-30, -100);
        for (int i = 0; i < 60; ++i) {
            p4.x += 1;
            auto ret = leg_solver.solve(p4);
            leg_viewer.set_r_ab(ret.first, ret.second);
            cv::Mat view = leg_viewer.get_view();
            cv::imshow("LegInverseKinematicsDemo", view);
            cv::waitKey(1);
        }
    }

    return 0;
}