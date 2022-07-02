#pragma once
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "constants.h"

using namespace std;

#define BLOB_RED    ENEMY_RED  //定义敌军颜色
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

/******************* 灯条类定义 ***********************/
class LightBlob {
public:
	cv::RotatedRect rect;   //灯条位置
	double area_ratio;
	double length;          //灯条长度
	uint8_t blob_color;      //灯条颜色

	LightBlob(cv::RotatedRect &r, double ratio, uint8_t color) : rect(r), area_ratio(ratio), blob_color(color) {
		length = max(rect.size.height, rect.size.width);
	};
	LightBlob() = default;
};

typedef std::vector<LightBlob> LightBlobs;



/******************* 装甲板类定义　**********************/
class ArmorBox {
public:
	typedef enum {
		FRONT, SIDE, UNKNOWN
	} BoxOrientation;

	cv::Rect2d rect;   //（画矩形框）
	LightBlobs light_blobs;
	uint8_t box_color;
	int id;

	explicit ArmorBox(const cv::Rect &pos = cv::Rect2d(), const LightBlobs &blobs = LightBlobs(), uint8_t color = 0, int i = 0);

	cv::Point2f getCenter() const; // 获取装甲板中心
	cv::Point2f updateCenter; // 滤波更新后的装甲板中心
	cv::Point2f predictCenter; // 滤波预测后的装甲板中心
	double getBlobsDistance() const; // 获取两个灯条中心间距
	double lengthDistanceRatio() const; // 获取灯条中心距和灯条长度的比值
};

typedef std::vector<ArmorBox> ArmorBoxes;

/********************* 自瞄类定义 **********************/
class ArmorFinder {
public:
	ArmorFinder(uint8_t &color);
	~ArmorFinder() = default;
	ArmorBox target_box, last_box;                      // 目标装甲板
	int tracking_cnt;                                   // 记录追踪帧数，用于定时退出追踪
private:
	//typedef cv::TrackerKCF TrackerToUse;                // Tracker类型定义(KCF跟踪器（lyx))

	typedef enum {
		SEARCHING_STATE, TRACKING_STATE, STANDBY_STATE
	} State;                                            // 自瞄状态枚举定义

	const uint8_t &enemy_color;                         // 敌方颜色，引用外部变量，自动变化
	State state;                                        // 自瞄状态对象实例
	int anti_switch_cnt;                                // 防止乱切目标计数器
	int contour_area;                                   // 装甲区域亮点个数，用于数字识别未启用时判断是否跟丢（已弃用）
	
	//cv::Ptr<cv::Tracker> tracker;

	bool stateSearchingTarget(cv::Mat& src);            // searching state主函数
	bool findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs);
	bool findArmorBox(const cv::Mat &src, ArmorBox &box);
	bool stateTrackingTarget(cv::Mat& src);             // tracking state主函数
	bool matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes);
public:
	void run(cv::Mat &src);                             // 自瞄主函数
};

cv::Rect getDetectROI(const cv::Rect& pos, int scale, cv::Size src_size);



