//
//��������Ѱ���ж��Ƿ�Ϊװ�װ�
//

#include "armor_finder.h"
#include "show_image.h"
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
using namespace Eigen;
// �ж����������ĽǶȲ�
static bool angelJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
		light_blob_i.rect.angle - 90;
	float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
		light_blob_j.rect.angle - 90;
	//cout << "LIGHTBLOB_ANGLE:" << abs(angle_i - angle_j) << endl;
	return abs(angle_i - angle_j) < LIGHTBLOB_ANGLE;
}
// �ж����������ĸ߶Ȳ�
static bool heightJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
	//cout << "LIGHTBLOB_HEIGHT" << abs(centers.y) << endl;
	return abs(centers.y) < LIGHTBLOB_HEIGHT;
}
// �ж����������ļ��
static bool lengthJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	double side_length;
	cv::Point2f centers = light_blob_i.rect.center - light_blob_j.rect.center;
	side_length = sqrt(centers.ddot(centers));
	//cout << " LIGHTBLOB_GAP" << side_length / light_blob_i.length << endl;
	return (side_length / light_blob_i.length < LIGHTBLOB_GAP_MAX && side_length / light_blob_i.length > LIGHTBLOB_GAP_MIN);
}
// �ж����������ĳ��ȱ�
static bool lengthRatioJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	//cout << " LIGHTBLOB_RATIO" << light_blob_i.length / light_blob_j.length << endl;
	return (light_blob_i.length / light_blob_j.length < LIGHTBLOB_RATIO_MAX
		&& light_blob_i.length / light_blob_j.length > LIGHTBLOB_RATIO_MIN);
}

// �ж����������Ĵ�λ�� 
static bool CuoWeiDuJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
		light_blob_i.rect.angle - 90;
	float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
		light_blob_j.rect.angle - 90;
	float angle = (angle_i + angle_j) / 2.0 / 180.0 * 3.14159265459;
	if (abs(angle_i - angle_j) > 90) {
		angle += 3.14159265459 / 2;
	}
	Vector2f orientation(cos(angle), sin(angle));
	Vector2f p2p(light_blob_j.rect.center.x - light_blob_i.rect.center.x,
		light_blob_j.rect.center.y - light_blob_i.rect.center.y);
	return abs(orientation.dot(p2p)) < LIGHTBLOB_STAGGER;
}
// �ж�װ�װ巽��
static bool boxAngleJudge(const LightBlob &light_blob_i, const LightBlob &light_blob_j) {
	float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle :
		light_blob_i.rect.angle - 90;
	float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle :
		light_blob_j.rect.angle - 90;
	float angle = (angle_i + angle_j) / 2.0;
	if (abs(angle_i - angle_j) > 90) {
		angle += 90.0;
	}
	return (-120.0 < angle && angle < -60.0) || (60.0 < angle && angle < 120.0);
}
// �ж����������Ƿ����ƥ��
static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color) {
	return light_blob_i.blob_color == enemy_color &&
		light_blob_j.blob_color == enemy_color &&
		lengthRatioJudge(light_blob_i, light_blob_j) &&
		lengthJudge(light_blob_i, light_blob_j) &&
		heightJudge(light_blob_i, light_blob_j) &&
		angelJudge(light_blob_i, light_blob_j) &&
		CuoWeiDuJudge(light_blob_i, light_blob_j);

}
// ƥ�����е������ó�װ�װ��ѡ��
bool ArmorFinder::matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes) {
	armor_boxes.clear();
	for (int i = 0; i < light_blobs.size() - 1; ++i) {
		for (int j = i + 1; j < light_blobs.size(); ++j) {
			if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j), enemy_color)) {
				continue;
			}
			cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i)).rect.boundingRect();    //����С����(б��)Ȧ����lyx��
			cv::Rect2d rect_right = light_blobs.at(static_cast<unsigned long>(j)).rect.boundingRect();
			double min_x, min_y, max_x, max_y;
			min_x = fmin(rect_left.x, rect_right.x) - 4;
			max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 4;
			min_y = fmin(rect_left.y, rect_right.y) - 0.5 * (rect_left.height + rect_right.height) / 2.0;
			max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) +
				0.5 * (rect_left.height + rect_right.height) / 2.0;
			if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
				continue;
			}
			if (state == SEARCHING_STATE && (max_y + min_y) / 2 < 120) continue;
			if ((max_x - min_x) / (max_y - min_y) < 0.8) continue;
			LightBlobs pair_blobs = { light_blobs.at(i), light_blobs.at(j) };
			armor_boxes.emplace_back(
				cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y),
				pair_blobs,
				enemy_color
				);
		}
	}
	return !armor_boxes.empty();
}
// �ڸ�����ͼ����Ѱ��װ�װ�
bool ArmorFinder::findArmorBox(const cv::Mat &src, ArmorBox &box) {
	LightBlobs light_blobs; // �洢���п��ܵĵ���
	ArmorBoxes armor_boxes; // װ�װ��ѡ��

	box.rect = cv::Rect2d(0, 0, 0, 0);
	box.id = -1;
	// Ѱ�����п��ܵĵ���
	if (!findLightBlobs(src, light_blobs)) {
		return false;
	}
	if (show_light_blobs && state == SEARCHING_STATE) {
		showLightBlobs("light_blobs", src, light_blobs);
		cv::waitKey(1);
	}
	// �Ե�������ƥ��ó�װ�װ��ѡ��
	if (!matchArmorBoxes(src, light_blobs, armor_boxes)) {
		//showArmorBoxes("boxes", src, armor_boxes);
		return false;
	}
	if (show_armor_boxes && state == SEARCHING_STATE) {//show_armor_boxes && state == SEARCHING_STATE
		showArmorBoxes("boxes", src, armor_boxes);
	}
	//classifier�� //
	box = armor_boxes[0];
	armor_boxes.clear();
	return true;
}


