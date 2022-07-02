#include "armor_finder.h"
#include "show_image.h"

double getPointLength(const cv::Point2f& p) {
    return sqrt(p.x * p.x + p.y * p.y);
}
bool ArmorFinder::stateSearchingTarget(cv::Mat& src) {
    if (findArmorBox(src, target_box))
	{ // 在原图中寻找目标，并返回是否找到
        return true;
    }
    else 
	{
        target_box = ArmorBox();
        return false;
    }
}