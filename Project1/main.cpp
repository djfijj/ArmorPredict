/*
* ========================================================================
* Copyright(c) 国防科技大学军临战队视觉组, All Rights Reserved.
* ========================================================================
*
* 【装甲板预测效果测试】
*
* 作者：刘宇轩   时间：2021/12/11
* 文件名：$Project1$
* 版本：V2.0.0
* 说明：拥有传统视觉识别性能，用于预测效果测试;
*       依赖环境：opencv3.4.5，eigen
*       附加环境：gnuplot      
* 
* 修改记录：
* 修改者：刘宇轩           时间：2022/06/21
* 修改说明：1、代码规范化处理  
*			2、加入管道调用gnuplot画图
* ========================================================================
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include "armor_finder.h"
#include "cnt_time.h"
#include "kalman.h"
#include "armor_finder.h"
#include "show_image.h"
#include "para.h"

uint8_t enemy_color = ENEMY_BLUE;  //敌方装甲板颜色
bool show_armor_box = 1;  //显示单个装甲板
bool show_armor_boxes = 0;  //显示多个装甲板
bool show_light_blobs = 0;  //显示灯条
bool save_video = 1;  //保存视频

cv::Mat src;  //待识别图像
ArmorFinder armor_finder(enemy_color); //装甲板识别实例
Kalman kf;
void kfinit(float singer[5]);  //kalman初始化
float* ReadFile();  //kalman参数读取，参数依次为：a T P Q R

int main(int argc, char* argv[])
{
	parseParameter(argc, argv, "enemy_color", enemy_color);
	parseParameter(argc, argv, "show_armor_box", show_armor_box);
	parseParameter(argc, argv, "show_armor_boxes", show_armor_boxes);
	parseParameter(argc, argv, "show_light_blobs", show_light_blobs);
	parseParameter(argc, argv, "save_video", save_video);
	ofstream myfile("result.txt");  //预测结果写入文件
	CntTime time;
	double t1;
	if (!myfile.is_open())
	{
		cout << "无法打开数据记录文件" << endl;
		return 0;
	}
	cv::VideoCapture capture("live.avi");  //读入视频
	float *singer = ReadFile();
	kfinit(singer);
	while (cv::waitKey(1) != 27)
	{
		time.end();
		t1 = time.interval;
		time.start();
		if (t1 < 0.02 && t1 >= 0)  //控制帧率为50
		{
			Sleep((0.02-t1)*1000);
		}
		if (!capture.read(src))  //获取当前帧图像
		{
			break;
		}
		resize(src, src, cv::Size(1280, 1024), 0, 0, cv::INTER_LINEAR);//统一图像大小
		armor_finder.run(src);  //开启识别
		if (find_armor)
		{
			//根据识别信息对滤波器进行预测、更新
			if (armor_finder.tracking_cnt == 0) {
				kf.stateOpt << armor_finder.target_box.getCenter().x,0,0;
			}
			MatrixXf measurement(1, 1);
			measurement << armor_finder.target_box.getCenter().x;
			kf.predict();
			kf.correct(measurement);
			armor_finder.target_box.updateCenter.x = kf.stateOpt(0, 0);
			armor_finder.target_box.updateCenter.y = armor_finder.target_box.getCenter().y;
			armor_finder.target_box.predictCenter.x = kf.statePre(0, 0);
			armor_finder.target_box.predictCenter.y = armor_finder.target_box.getCenter().y;
			armor_speed = kf.stateOpt(1, 0);
			lastestss_armor_speed = lastests_armor_speed;
			lastests_armor_speed = lastest_armor_speed;
			lastest_armor_speed = last_armor_speed;
			last_armor_speed = armor_speed;
			//模拟电控端延时预测效果，将时间间隔放大5倍
			static double alpha = singer[0];
			static double t = 5 * singer[1];
			MatrixXd transPre(3, 3);
			transPre << 1, t, (alpha* t - 1 + exp(-alpha * t)) / alpha / alpha,
				0, 1, (1 - exp(-alpha * t)) / alpha,
				0, 0, exp(-alpha * t);
			MatrixXd control(3, 1);
			control << 1 / alpha * (-t + alpha * t * t / 2 + (1 - exp(-alpha * t) / alpha)), t - (1 - exp(-alpha * t) / alpha), 1 - exp(-alpha * t);
			MatrixXd predictState(3, 1);
			MatrixXd State(3, 1);
			State << kf.stateOpt(0, 0), kf.stateOpt(1, 0), kf.stateOpt(2, 0);
			predictState = transPre * State +control * State(2, 0);
			//发散判据计算与判定
			MatrixXf xl(1, 1);
			xl = kf.measureMat * kf.errorCovpre * kf.measureMat.transpose() + kf.measureNosiseCov;
			MatrixXf xx(1, 1);
			xx = measurement - kf.measureMat * kf.statePre;
			MatrixXf xx2(1, 1);
			xx2 = xx.transpose() * xx;
			if (xx2(0, 0) >=  10 * xl(0, 0)) {
				//cout << "判定发散" << endl;
				myfile << armor_finder.target_box.getCenter().x << "\t" <<armor_finder.target_box.predictCenter.x << "\t" << armor_finder.target_box.updateCenter.x << "\t" << 0 << "\t" << predictState(0, 0) << "\n";
			}
			else {
				//cout << "判定收敛" << endl;
				myfile << armor_finder.target_box.getCenter().x << "\t" << armor_finder.target_box.predictCenter.x  << "\t" << armor_finder.target_box.updateCenter.x << "\t" << predictState(0, 0) << "\t" << 0 << "\n";
			}
			if (show_armor_box)
			{
				showArmorBox("box", src, armor_finder.target_box, predictState(0, 0));
			}
		}
		else
		{
			if (show_armor_box)
			{
				showArmorBox("box", src, armor_finder.target_box, 0);
			}
		}
	}
	myfile.close();
	//调用gnuplot画图
	FILE* pipe = _popen("gnuplot","w");
	fprintf_s(pipe, "datafile='result.txt'\n");
	fprintf_s(pipe, "set title 'singer kalman'\n");
	fprintf_s(pipe, "set xtics format '% .3f'\n");
	fprintf_s(pipe, "set ytics format '% .3f'\n");
	fprintf_s(pipe, "plot datafile using 1 title 'real' lw 1 with p,datafile using 2 title 'predict' lw 1 with p, datafile using 3 title 'update' lw 1 with p, datafile using 4 title 'long true predict' lw 1 with p, datafile using 5 title 'long false predict' lw1 with p\n");
	fprintf_s(pipe, "pause mouse\n");
	_pclose(pipe);
	return 0;
}

void kfinit(float singer[5])
{
	kf.init(3, 1, 1);  //kalman初始化
	float alpha = singer[0];  //singer参数机动频率
	float dt = singer[1];  //singer参数时间间隔
	kf.transMat << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  //singer状态转移矩阵
		0, 1, (1 - exp(-alpha * dt)) / alpha,
		0, 0, exp(-alpha * dt);
	kf.measureMat << 1, 0, 0;  //测量矩阵 
	kf.contrMat << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)), dt - (1 - exp(-alpha * dt) / alpha), 1 - exp(-alpha * dt); //控制矩阵
	float p = singer[2];
	kf.errorCovOpt << p, 0, 0,  //误差传递矩阵
		0, p, 0,
		0, 0, p;
	float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
	float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
	float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
	float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
	float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
	float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
	float k = singer[3];
	kf.processNoiseCov << k * alpha * q11, k* alpha* q12, k* alpha* q13,  //过程噪声矩阵
		k* alpha* q12, k* alpha* q22, k* alpha* q23,
		k* alpha* q13, k* alpha* q23, k* alpha* q33;
	float r = singer[4];
	kf.measureNosiseCov << r;  //测量噪声矩阵
}

float* ReadFile()
{
	ifstream singerFile("read.txt");  //kalman参数读取文件，参数依次为：a T P Q R
	float *singer = new float[5];
	singerFile >> singer[0];
	singerFile >> singer[1];
	singerFile >> singer[2];
	singerFile >> singer[3];
	singerFile >> singer[4];
	singerFile.close();
	return singer;
}

