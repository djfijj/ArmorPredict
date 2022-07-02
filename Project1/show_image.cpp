//
//识别可视化处理
//

#include "show_image.h"
#include <opencv2/highgui.hpp>

void drawLightBlobs(cv::Mat &src, const LightBlobs &blobs) {
	for (const auto &blob : blobs) {
		cv::Scalar color(0, 255, 0);
		if (blob.blob_color == BLOB_RED)
			color = cv::Scalar(0, 0, 255);
		else if (blob.blob_color == BLOB_BLUE)
			color = cv::Scalar(255, 0, 0);
		cv::Point2f vertices[4];
		blob.rect.points(vertices);
		for (int j = 0; j < 4; j++) {
			cv::line(src, vertices[j], vertices[(j + 1) % 4], color, 2);
		}
	}
}

/**************************
*     显示多个装甲板区域    *
**************************/
void showArmorBoxes(std::string windows_name, const cv::Mat &src, const ArmorBoxes &armor_boxes) {
	static cv::Mat image2show;
	if (src.type() == CV_8UC1) {// 黑白图像
		cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
	}
	else if (src.type() == CV_8UC3) { //RGB 彩色
		image2show = src.clone();
	}

	for (auto &box : armor_boxes) {
		if (box.box_color == BOX_BLUE) {
			rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 1);
			drawLightBlobs(image2show, box.light_blobs);
		}
		else if (box.box_color == BOX_RED) {
			rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 1);
			drawLightBlobs(image2show, box.light_blobs);
		}

	}
	imshow(windows_name, image2show);
}

/**************************
* 显示多个装甲板区域及其类别 *
**************************/
void showArmorBoxesClass(std::string window_names, const cv::Mat &src, const ArmorBoxes &boxes) {
	static cv::Mat image2show;
	if (src.type() == CV_8UC1) { // 黑白图像
		cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
	}
	else if (src.type() == CV_8UC3) { //RGB 彩色
		image2show = src.clone();
	}
	for (const auto &box : boxes) {
		rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 1);
		drawLightBlobs(image2show, box.light_blobs);
		//putText(image2show, "0", Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1,
		//	Scalar(0, 255, 0));	
		}
	imshow(window_names, image2show);
}

/**************************
* 显示单个装甲板区域及其类别 *
**************************/
void showArmorBox(std::string windows_name, const cv::Mat &src, const ArmorBox &box, float predict_x) {
    static cv::Mat image2show;
	if (src.type() == CV_8UC1) { // 黑白图像
		cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
	}
	else if (src.type() == CV_8UC3) { //RGB 彩色
		image2show = src.clone();
	}
	drawLightBlobs(image2show, box.light_blobs);
	//    static FILE *fp = fopen(PROJECT_DIR"/ratio.txt", "w");
	//    if(box.light_blobs.size() == 2)
	//        fprintf(fp, "%lf %lf %lf\n", box.light_blobs[0].length, box.light_blobs[1].length, box.getBlobsDistance())
	//    cout << box.lengthDistanceRatio() << endl;
	if (box.rect != cv::Rect2d())
	{
		rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 1);
		char num[10];
		sprintf_s(num, "%.2f", armor_angle);
		string str1 = num;
		putText(image2show, str1 + "'", cv::Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 255, 0));
		float mean_v = (lastestss_armor_speed + lastests_armor_speed + lastest_armor_speed + last_armor_speed) / 4;
		char num1[10];
		sprintf_s(num1, "%.2f", mean_v);
		string str2 = num1;
		float k = 50;//500/abs(lastests_dv-dv)/6;
		line(image2show, cv::Point(0, 250), cv::Point(300, 250), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
		line(image2show, cv::Point(0, 250 + mean_v * k), cv::Point(300, 250 + mean_v * k), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
		putText(image2show, str2, cv::Point(310, 250 + (mean_v)*k), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 0, 0));
		line(image2show, cv::Point(60, 250 + lastestss_armor_speed * k), cv::Point(140, 250 + lastests_armor_speed * k), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
		line(image2show, cv::Point(140, 250 + lastests_armor_speed * k), cv::Point(210, 250 + lastest_armor_speed * k), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
		line(image2show, cv::Point(210, 250 + lastest_armor_speed * k), cv::Point(300, 250 + last_armor_speed * k), cv::Scalar(0, 0, 255), 2, cv::LINE_8);
		circle(image2show, box.getCenter(), 2, cv::Scalar(0, 255, 0), -1);
		circle(image2show, box.predictCenter, 2, cv::Scalar(255, 255, 255), -1);
		circle(image2show, box.updateCenter, 2, cv::Scalar(0, 0, 255), -1);
		circle(image2show, cv::Point(predict_x, box.predictCenter.y), 4, cv::Scalar(255, 255, 0), -1);
	}
	imshow(windows_name, image2show);
	if (save_video)
	{
		static bool init = 0;
		static cv::VideoWriter writer;
		if (!init)
		{
			init = 1;
			int coder = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');  //选择编码格式
			string filename = "result.avi";  //保存的视频文件名称
			writer.open(filename, coder, 25, cv::Size(1280, 1024), 1);  //创建保存视频文件的视频流
		}
		else
		{
			writer.write(image2show);
		}
	}
}

/**************************
*      显示多个灯条区域     *
**************************/
void showLightBlobs(std::string windows_name, const cv::Mat &src, const LightBlobs &light_blobs) {
	static cv::Mat image2show;

	if (src.type() == CV_8UC1) { // 黑白图像
		cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
	}
	else if (src.type() == CV_8UC3) { //RGB 彩色
		image2show = src.clone();
	}

	for (const auto &light_blob : light_blobs) {
		cv::Scalar color(0, 255, 0);
		if (light_blob.blob_color == BLOB_RED)
			color = cv::Scalar(0, 0, 255);
		else if (light_blob.blob_color == BLOB_BLUE)
			color = cv::Scalar(255, 0, 0);
		cv::Point2f vertices[4];
		light_blob.rect.points(vertices);
		for (int j = 0; j < 4; j++) {
			cv::line(image2show, vertices[j], vertices[(j + 1) % 4], color, 2);
		}
	}
	imshow(windows_name, image2show);
}


void showTrackSearchingPos(std::string window_names, const cv::Mat &src, const cv::Rect2d pos) {
	static cv::Mat image2show;
	if (src.type() == CV_8UC1) { // 黑白图像
		cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
	}
	else if (src.type() == CV_8UC3) { //RGB 彩色
		image2show = src.clone();
	}
	rectangle(image2show, pos, cv::Scalar(0, 255, 0), 1);
	imshow(window_names, image2show);
}
