#include <opencv2/highgui.hpp>
#include"armor_finder.h"
#include "show_image.h"

ArmorFinder::ArmorFinder(uint8_t &color) :
	enemy_color(color),
	state(SEARCHING_STATE),
	anti_switch_cnt(0),
	contour_area(0) {}

void ArmorFinder::run(cv::Mat &src) {
	switch (state) {
	case SEARCHING_STATE:
		//cout << "进入searching state" << endl;
		if (stateSearchingTarget(src)) {
			find_armor = 1;
			armor_angle = atan((target_box.getCenter().y - last_box.getCenter().y) / (target_box.getCenter().x - last_box.getCenter().x)) * 180 / 3.1416;
			last_box = target_box;
			state = TRACKING_STATE;
			tracking_cnt = 0;
		}
		else {
			find_armor = 0;
		}
		break;
	case TRACKING_STATE:
		if (stateTrackingTarget(src)) {
			find_armor = 1;
			armor_angle = atan((target_box.getCenter().y - last_box.getCenter().y) / (target_box.getCenter().x - last_box.getCenter().x)) * 180 / 3.1416;
			last_box = target_box;
		}
		else {
			find_armor = 0;
			state = SEARCHING_STATE;
		}
		if (++tracking_cnt > 500) {    // 最多追踪100帧图像
			state = SEARCHING_STATE;
			cout << "退出tracking state" << endl;
		}
		break;
	}
}

