//
// 图片元素计算，装甲板参数测算，获取距离
//

#include "armor_finder.h"

ArmorBox::ArmorBox(const cv::Rect &pos, const LightBlobs &blobs, uint8_t color, int i) :
	rect(pos), light_blobs(blobs), box_color(color), id(i) {};

// 获取装甲板中心点
cv::Point2f ArmorBox::getCenter() const {
	return cv::Point2f(
		rect.x + rect.width / 2,
		rect.y + rect.height / 2
		);
}

// 获取两个灯条中心点的间距
double ArmorBox::getBlobsDistance() const {
	if (light_blobs.size() == 2) {
		auto &x = light_blobs[0].rect.center;
		auto &y = light_blobs[1].rect.center;
		return sqrt((x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y));
	}
	else {
		return 0;
	}
}

// 获取灯条长度和间距的比例
double ArmorBox::lengthDistanceRatio() const {
	if (light_blobs.size() == 2) {
		return max(light_blobs[0].length, light_blobs[1].length)
			/ getBlobsDistance();
	}
	else {
		return 100;
	}
}
