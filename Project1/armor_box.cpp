//
// ͼƬԪ�ؼ��㣬װ�װ�������㣬��ȡ����
//

#include "armor_finder.h"

ArmorBox::ArmorBox(const cv::Rect &pos, const LightBlobs &blobs, uint8_t color, int i) :
	rect(pos), light_blobs(blobs), box_color(color), id(i) {};

// ��ȡװ�װ����ĵ�
cv::Point2f ArmorBox::getCenter() const {
	return cv::Point2f(
		rect.x + rect.width / 2,
		rect.y + rect.height / 2
		);
}

// ��ȡ�����������ĵ�ļ��
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

// ��ȡ�������Ⱥͼ��ı���
double ArmorBox::lengthDistanceRatio() const {
	if (light_blobs.size() == 2) {
		return max(light_blobs[0].length, light_blobs[1].length)
			/ getBlobsDistance();
	}
	else {
		return 100;
	}
}
