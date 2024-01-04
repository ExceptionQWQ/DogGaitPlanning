/*
 * @author BusyBox
 * @date 2024/01/05
 */

#include <iostream>
#include <opencv2/opencv.hpp>

#include "leg_viewer.hpp"

int main()
{
    LegParam leg_param = {.l1 = 26, .l2 = 56, .l3 = 56, .l4 = 26, .l5 = 56, .ab = 25, .r_a = CV_PI / 180 * 200, .r_b = CV_PI / 180 * 150};
    LegViewer leg_viewer(cv::Size(512, 512), leg_param);
    while (true) {
        cv::Mat view = leg_viewer.get_view();
        cv::imshow("view", view);
        cv::waitKey(1);
    }

    return 0;
}