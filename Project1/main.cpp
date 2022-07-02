/*
* ========================================================================
* Copyright(c) �����Ƽ���ѧ����ս���Ӿ���, All Rights Reserved.
* ========================================================================
*
* ��װ�װ�Ԥ��Ч�����ԡ�
*
* ���ߣ�������   ʱ�䣺2021/12/11
* �ļ�����$Project1$
* �汾��V2.0.0
* ˵����ӵ�д�ͳ�Ӿ�ʶ�����ܣ�����Ԥ��Ч������;
*       ����������opencv3.4.5��eigen
*       ���ӻ�����gnuplot      
* 
* �޸ļ�¼��
* �޸��ߣ�������           ʱ�䣺2022/06/21
* �޸�˵����1������淶������  
*			2������ܵ�����gnuplot��ͼ
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

uint8_t enemy_color = ENEMY_BLUE;  //�з�װ�װ���ɫ
bool show_armor_box = 1;  //��ʾ����װ�װ�
bool show_armor_boxes = 0;  //��ʾ���װ�װ�
bool show_light_blobs = 0;  //��ʾ����
bool save_video = 1;  //������Ƶ

cv::Mat src;  //��ʶ��ͼ��
ArmorFinder armor_finder(enemy_color); //װ�װ�ʶ��ʵ��
Kalman kf;
void kfinit(float singer[5]);  //kalman��ʼ��
float* ReadFile();  //kalman������ȡ����������Ϊ��a T P Q R

int main(int argc, char* argv[])
{
	parseParameter(argc, argv, "enemy_color", enemy_color);
	parseParameter(argc, argv, "show_armor_box", show_armor_box);
	parseParameter(argc, argv, "show_armor_boxes", show_armor_boxes);
	parseParameter(argc, argv, "show_light_blobs", show_light_blobs);
	parseParameter(argc, argv, "save_video", save_video);
	ofstream myfile("result.txt");  //Ԥ����д���ļ�
	CntTime time;
	double t1;
	if (!myfile.is_open())
	{
		cout << "�޷������ݼ�¼�ļ�" << endl;
		return 0;
	}
	cv::VideoCapture capture("live.avi");  //������Ƶ
	float *singer = ReadFile();
	kfinit(singer);
	while (cv::waitKey(1) != 27)
	{
		time.end();
		t1 = time.interval;
		time.start();
		if (t1 < 0.02 && t1 >= 0)  //����֡��Ϊ50
		{
			Sleep((0.02-t1)*1000);
		}
		if (!capture.read(src))  //��ȡ��ǰ֡ͼ��
		{
			break;
		}
		resize(src, src, cv::Size(1280, 1024), 0, 0, cv::INTER_LINEAR);//ͳһͼ���С
		armor_finder.run(src);  //����ʶ��
		if (find_armor)
		{
			//����ʶ����Ϣ���˲�������Ԥ�⡢����
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
			//ģ���ض���ʱԤ��Ч������ʱ�����Ŵ�5��
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
			//��ɢ�оݼ������ж�
			MatrixXf xl(1, 1);
			xl = kf.measureMat * kf.errorCovpre * kf.measureMat.transpose() + kf.measureNosiseCov;
			MatrixXf xx(1, 1);
			xx = measurement - kf.measureMat * kf.statePre;
			MatrixXf xx2(1, 1);
			xx2 = xx.transpose() * xx;
			if (xx2(0, 0) >=  10 * xl(0, 0)) {
				//cout << "�ж���ɢ" << endl;
				myfile << armor_finder.target_box.getCenter().x << "\t" <<armor_finder.target_box.predictCenter.x << "\t" << armor_finder.target_box.updateCenter.x << "\t" << 0 << "\t" << predictState(0, 0) << "\n";
			}
			else {
				//cout << "�ж�����" << endl;
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
	//����gnuplot��ͼ
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
	kf.init(3, 1, 1);  //kalman��ʼ��
	float alpha = singer[0];  //singer��������Ƶ��
	float dt = singer[1];  //singer����ʱ����
	kf.transMat << 1, dt, (alpha * dt - 1 + exp(-alpha * dt)) / alpha / alpha,  //singer״̬ת�ƾ���
		0, 1, (1 - exp(-alpha * dt)) / alpha,
		0, 0, exp(-alpha * dt);
	kf.measureMat << 1, 0, 0;  //�������� 
	kf.contrMat << 1 / alpha * (-dt + alpha * dt * dt / 2 + (1 - exp(-alpha * dt) / alpha)), dt - (1 - exp(-alpha * dt) / alpha), 1 - exp(-alpha * dt); //���ƾ���
	float p = singer[2];
	kf.errorCovOpt << p, 0, 0,  //���ݾ���
		0, p, 0,
		0, 0, p;
	float q11 = 1 / (2 * pow(alpha, 5)) * (1 - exp(-2 * alpha * dt) + 2 * alpha * dt + 2 * pow(alpha * dt, 3) / 3 - 2 * pow(alpha * dt, 2) - 4 * alpha * dt * exp(-alpha * dt));
	float q12 = 1 / (2 * pow(alpha, 4)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt) + 2 * alpha * dt * exp(-alpha * dt) - 2 * alpha * dt + pow(alpha * dt, 2));
	float q13 = 1 / (2 * pow(alpha, 3)) * (1 - exp(-2 * alpha * dt) - 2 * alpha * dt * exp(-alpha * dt));
	float q22 = 1 / (2 * pow(alpha, 3)) * (4 * exp(-alpha * dt) - 3 - exp(-2 * alpha * dt) + 2 * alpha * dt);
	float q23 = 1 / (2 * pow(alpha, 2)) * (exp(-2 * alpha * dt) + 1 - 2 * exp(-alpha * dt));
	float q33 = 1 / (2 * alpha) * (1 - exp(-2 * alpha * dt));
	float k = singer[3];
	kf.processNoiseCov << k * alpha * q11, k* alpha* q12, k* alpha* q13,  //������������
		k* alpha* q12, k* alpha* q22, k* alpha* q23,
		k* alpha* q13, k* alpha* q23, k* alpha* q33;
	float r = singer[4];
	kf.measureNosiseCov << r;  //������������
}

float* ReadFile()
{
	ifstream singerFile("read.txt");  //kalman������ȡ�ļ�����������Ϊ��a T P Q R
	float *singer = new float[5];
	singerFile >> singer[0];
	singerFile >> singer[1];
	singerFile >> singer[2];
	singerFile >> singer[3];
	singerFile >> singer[4];
	singerFile.close();
	return singer;
}

