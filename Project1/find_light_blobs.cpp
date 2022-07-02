//
//Ѱ�ҵ������жϵ����Ϸ���
//

#include "armor_finder.h"
#include "constants.h"
#include <opencv2/highgui.hpp>
// ��ת���εĳ����
static double lw_rate(const cv::RotatedRect &rect) {
	return rect.size.height > rect.size.width ?
		rect.size.height / rect.size.width :
		rect.size.width / rect.size.height;
}
// �������������С��Ӿ������֮��
static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
	return cv::contourArea(contour) / rect.size.area();
}
// �ж������Ƿ�Ϊһ������
static bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
	return (1.2 < lw_rate(rect) && lw_rate(rect) < 10) &&
		((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
			(rect.size.area() >= 50 && areaRatio(contour, rect) > 0.5));
}
// �жϵ�����ɫ(�˺��������������Ż���
static uint8_t get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) {
	auto region = blobPos.boundingRect();
	region.x -= fmax(3, region.width * 0.1);
	region.y -= fmax(3, region.height * 0.05);
	region.width += 2 * fmax(3, region.width * 0.1);
	region.height += 2 * fmax(3, region.height * 0.05);
	region &= cv::Rect(0, 0, src.cols, src.rows);   //cols�У�rows��
	cv::Mat roi = src(region);
	int red_cnt = 0, blue_cnt = 0;
	for (int row = 0; row < roi.rows; row++) {
		for (int col = 0; col < roi.cols; col++) {
			red_cnt += roi.at<cv::Vec3b>(row, col)[2];
			blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
		}
	}
	if (red_cnt > blue_cnt) {
		return BLOB_RED;
	}
	else {
		return BLOB_BLUE;
	}
}
// �ж���������������ͬһ������
static bool isSameBlob(LightBlob blob1, LightBlob blob2) {
	auto dist = blob1.rect.center - blob2.rect.center;
	return (dist.x * dist.x + dist.y * dist.y) < 9;
}
// ��̬ѧ����
static void imagePreProcess(cv::Mat &src) {
	//double erode01=(double)erode0 /100.0*6;
	//double erode02=(double)erode1 /100.0*6;
	static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	dilate(src, src, kernel_dilate);

	static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	erode(src, src, kernel_erode);

	static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	erode(src, src, kernel_erode2);

	static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	dilate(src, src, kernel_dilate2);
	
}
// �ڸ���ͼ����Ѱ�����п��ܵĵ���
bool ArmorFinder::findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs) {
	cv::Mat color_channel;
	cv::Mat src_bin_light, src_bin_dim;
	std::vector<cv::Mat> channels;       // ͨ�����
	cv::split(src, channels); 
	/*char TrackbarName[50];
	namedWindow("����������", 1);
	createTrackbar("��ֵlight 100", "����������", &light_threshold_slider_light, 100);
	createTrackbar("��ֵdim 100", "����������", &light_threshold_slider_dim, 100);*/
	//createTrackbar("��ֵdim 100", "bin_light", &light_threshold_slider_dim, 100);
	if (enemy_color == ENEMY_BLUE) {         /*                      */
		color_channel = channels[0];        /* ����Ŀ����ɫ����ͨ����ȡ */
	}
	else if (enemy_color == ENEMY_RED) {    /*                      */
		color_channel = channels[2];        /************************/
	}
	static double light_threshold = (120 / 100.0)*255;
	static double light_threshold0 = (80 / 100.0) *255;
	cv::threshold(color_channel, src_bin_light, light_threshold, 255, CV_THRESH_BINARY); // ��ֵ����Ӧͨ��
	if (src_bin_light.empty()) return false;
	imagePreProcess(src_bin_light);                                  // ��̬ѧ����

	cv::threshold(color_channel, src_bin_dim, light_threshold0, 255, CV_THRESH_BINARY); // ��ֵ����Ӧͨ������ֵ������ѡ�񣿣�
	if (src_bin_dim.empty()) return false;
	imagePreProcess(src_bin_dim);                                  // ��̬ѧ����

	if (show_light_blobs) {//src_bin_light.size() == cv::Size(640, 480) && 
		imshow("bin_light", src_bin_light);
		imshow("bin_dim", src_bin_dim);
	}
	// ʹ��������ͬ�Ķ�ֵ����ֵͬʱ���е�����ȡ�����ٻ������նԶ�ֵ�����������Ӱ�졣
	// ͬʱ�޳��ظ��ĵ������޳�������㣬���������ҳ����ĵ���ȡ������
	std::vector<std::vector<cv::Point>> light_contours_light, light_contours_dim;
	LightBlobs light_blobs_light, light_blobs_dim;
	std::vector<cv::Vec4i> hierarchy_light, hierarchy_dim;
	cv::findContours(src_bin_light, light_contours_light, hierarchy_light, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	cv::findContours(src_bin_dim, light_contours_dim, hierarchy_dim, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	for (int i = 0; i < light_contours_light.size(); i++) {
		if (hierarchy_light[i][2] == -1) {
			cv::RotatedRect rect = cv::minAreaRect(light_contours_light[i]);
			if (isValidLightBlob(light_contours_light[i], rect)) {
				light_blobs_light.emplace_back(
					rect, areaRatio(light_contours_light[i], rect), get_blob_color(src, rect)
					);
			}
		}
	}
	for (int i = 0; i < light_contours_dim.size(); i++) {
		if (hierarchy_dim[i][2] == -1) {
			cv::RotatedRect rect = cv::minAreaRect(light_contours_dim[i]);
			if (isValidLightBlob(light_contours_dim[i], rect)) {
				light_blobs_dim.emplace_back(
					rect, areaRatio(light_contours_dim[i], rect), get_blob_color(src, rect)   //����
					);
			}
		}
	}
	vector<int> light_to_remove, dim_to_remove;
	for (int l = 0; l != light_blobs_light.size(); l++) {
		for (int d = 0; d != light_blobs_dim.size(); d++) {
			if (isSameBlob(light_blobs_light[l], light_blobs_dim[d])) {
				if (light_blobs_light[l].area_ratio > light_blobs_dim[d].area_ratio) {
					dim_to_remove.emplace_back(d);
				}
				else {
					light_to_remove.emplace_back(l);
				}
			}
		}
	}
	sort(light_to_remove.begin(), light_to_remove.end(), [](int a, int b) { return a > b; });   //�Ӵ�С����
	sort(dim_to_remove.begin(), dim_to_remove.end(), [](int a, int b) { return a > b; });
	for (auto x : light_to_remove) {
		light_blobs_light.erase(light_blobs_light.begin() + x);
	}
	for (auto x : dim_to_remove) {
		light_blobs_dim.erase(light_blobs_dim.begin() + x);
	}
	for (const auto &light : light_blobs_light) {
		light_blobs.emplace_back(light);
	}
	for (const auto &dim : light_blobs_dim) {
		light_blobs.emplace_back(dim);
	}
	return light_blobs.size() >= 2;
}
