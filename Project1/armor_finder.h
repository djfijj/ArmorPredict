#pragma once
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "constants.h"

using namespace std;

#define BLOB_RED    ENEMY_RED  //����о���ɫ
#define BLOB_BLUE   ENEMY_BLUE

#define BOX_RED     ENEMY_RED
#define BOX_BLUE    ENEMY_BLUE

#define IMAGE_CENTER_X      (320)
#define IMAGE_CENTER_Y      (240-20)

extern float armor_speed;
extern float armor_angle;
extern cv::Mat src;
extern bool find_armor;
extern bool show_armor_box;
extern bool show_armor_boxes;
extern bool show_light_blob;
extern bool show_light_blobs;
extern bool save_video;
extern float last_armor_speed;
extern float lastest_armor_speed;
extern float lastests_armor_speed;
extern float lastestss_armor_speed;

/******************* �����ඨ�� ***********************/
class LightBlob {
public:
	cv::RotatedRect rect;   //����λ��
	double area_ratio;
	double length;          //��������
	uint8_t blob_color;      //������ɫ

	LightBlob(cv::RotatedRect &r, double ratio, uint8_t color) : rect(r), area_ratio(ratio), blob_color(color) {
		length = max(rect.size.height, rect.size.width);
	};
	LightBlob() = default;
};

typedef std::vector<LightBlob> LightBlobs;



/******************* װ�װ��ඨ�塡**********************/
class ArmorBox {
public:
	typedef enum {
		FRONT, SIDE, UNKNOWN
	} BoxOrientation;

	cv::Rect2d rect;   //�������ο�
	LightBlobs light_blobs;
	uint8_t box_color;
	int id;

	explicit ArmorBox(const cv::Rect &pos = cv::Rect2d(), const LightBlobs &blobs = LightBlobs(), uint8_t color = 0, int i = 0);

	cv::Point2f getCenter() const; // ��ȡװ�װ�����
	cv::Point2f updateCenter; // �˲����º��װ�װ�����
	cv::Point2f predictCenter; // �˲�Ԥ����װ�װ�����
	double getBlobsDistance() const; // ��ȡ�����������ļ��
	double lengthDistanceRatio() const; // ��ȡ�������ľ�͵������ȵı�ֵ
};

typedef std::vector<ArmorBox> ArmorBoxes;

/********************* �����ඨ�� **********************/
class ArmorFinder {
public:
	ArmorFinder(uint8_t &color);
	~ArmorFinder() = default;
	ArmorBox target_box, last_box;                      // Ŀ��װ�װ�
	int tracking_cnt;                                   // ��¼׷��֡�������ڶ�ʱ�˳�׷��
private:
	//typedef cv::TrackerKCF TrackerToUse;                // Tracker���Ͷ���(KCF��������lyx))

	typedef enum {
		SEARCHING_STATE, TRACKING_STATE, STANDBY_STATE
	} State;                                            // ����״̬ö�ٶ���

	const uint8_t &enemy_color;                         // �з���ɫ�������ⲿ�������Զ��仯
	State state;                                        // ����״̬����ʵ��
	int anti_switch_cnt;                                // ��ֹ����Ŀ�������
	int contour_area;                                   // װ�����������������������ʶ��δ����ʱ�ж��Ƿ�����������ã�
	
	//cv::Ptr<cv::Tracker> tracker;

	bool stateSearchingTarget(cv::Mat& src);            // searching state������
	bool findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs);
	bool findArmorBox(const cv::Mat &src, ArmorBox &box);
	bool stateTrackingTarget(cv::Mat& src);             // tracking state������
	bool matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes);
public:
	void run(cv::Mat &src);                             // ����������
};

cv::Rect getDetectROI(const cv::Rect& pos, int scale, cv::Size src_size);



