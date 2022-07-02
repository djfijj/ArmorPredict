 #include <iostream>
#include <Eigen/Dense>
#include "Kalman.h"
using namespace Eigen;
using namespace std;

void Kalman::init(int DP, int MP, int CP) {
	assert(DP > 0 && MP > 0);
	CP = std::max(CP, 0);

	statePre = MatrixXf::Zero(DP, 1);   //predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
	stateOpt = MatrixXf::Zero(DP, 1);   //corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
	transMat = MatrixXf::Identity(DP, DP); //A
	transMat_slow = MatrixXf::Identity(DP, DP); //A
	processNoiseCov = MatrixXf::Identity(DP, DP); //Q
	processNoiseCov_Slow = MatrixXf::Identity(DP, DP); //Q Slow_speed
	processNoiseCov_Fast = MatrixXf::Identity(DP, DP); //Q Fast_speed
	measureMat = MatrixXf::Zero(MP, DP); //H
	measureNosiseCov = MatrixXf::Identity(MP, MP); //R
	measureNosiseCov_Slow = MatrixXf::Identity(MP, MP); //R Slow_speed
	measureNosiseCov_Fast = MatrixXf::Identity(MP, MP); //R Fast_speed

	errorCovpre = MatrixXf::Zero(DP, DP);   //(P'(k)): P'(k)=A*P(k-1)*At + Q)
	errorCovOpt = MatrixXf::Zero(DP, DP);   //(P(k)) : P(k) = (I - K(k)*H)*P'(k)
	Kgain = MatrixXf::Zero(DP, MP);  //(K(k)) : K(k) = P'(k)*Ht*inv(H*P'(k)*Ht + R)

	//STKF
	Vk = MatrixXf::Zero(MP, MP);
	Nk = MatrixXf::Zero(MP, MP);
	Mk = MatrixXf::Zero(MP, MP);
	rho = 0.55;

	//
	if (CP > 0)
		contrMat = MatrixXf::Zero(DP, CP);

	tmp1 = MatrixXf::Zero(DP, DP);

	//tmp2 = MatrixXf::Zero(MP, DP);
	tmp2 = MatrixXf::Zero(DP, MP);
	tmp3 = MatrixXf::Zero(MP, MP);
	//tmp4 = MatrixXf::Zero(MP, DP);
	tmp4 = MatrixXf::Zero(DP, MP);
	tmp5 = MatrixXf::Zero(MP, 1);
	cP = CP;
}

double Kalman::get_beta(double vx, double vy, double yetax, double yetay) {

	double yeta = abs(vx + vy + yetax + yetay) / 25;
	//cout << yeta << endl;

	double bt1 = 0.001 + 1 / (1 + exp(-(yeta - 5)));
	double bt2 = 0.001 + 1 / (1 + exp(-(yeta - 8)));
	double bt3 = 0.001 + 1 / (1 + exp(-(0.5 * yeta - 5)));
	double bt4 = 0.001 + 1 / (1 + exp(-(0.4 * yeta - 5)));
	double bt5 = 0.001 + 1 / (1 + exp(-(0.85 * yeta - 8)));

	return bt5;
}

double Kalman::get_beta_plus(double vx, double vy) {

	double Vxy = sqrt(vx * vx + vy * vy);
	double bt = 0;
	double tp1 = Vxy - 30;
	double tp2 = pow(Vxy - 20, 0.1);
	if (Vxy <= 30)
		bt = 0.0000006;
	else if (Vxy > 30 && Vxy < 70)
		bt = 0.6 * (0.1 + 100 / (0.6 + exp(-tp1)));
	else if (Vxy >= 70 && Vxy <= 80)
		bt = 2 * (0.01 + 100 / (0.15 + exp(-tp2)));
	else if (Vxy > 80)
		bt = 2 * (0.01 + 100 / (0.1 + exp(-tp2)));
	//	bt = 0.1 * (0.01 + 100 / (0.1 + exp(-tp2)));
	/*if (Vxy <= 30)
		bt = 0.01;
	if (Vxy > 30 && Vxy < 80)
		bt = 0.5;
	if (Vxy >= 80)
		bt = 1.2;*/

	return bt;
}

MatrixXf& Kalman::get_Vk() {

	Vk = (rho * Vk + tmp5 * tmp5.transpose()) / (1 + rho);

	return Vk;
}

MatrixXf& Kalman::get_Mk() {

	Mk = measureMat * transMat * errorCovpre * transMat.transpose() * measureMat.transpose();
	return Mk;
}

MatrixXf& Kalman::get_Nk() {

	Nk = Vk - measureNosiseCov - measureMat * processNoiseCov * measureMat.transpose();
	return Nk;
}

double Kalman::get_lamda(int MP) {
	double tr_Nk = 0;
	double tr_Mk = 0;
	double lamda = 0;
	Vk = get_Vk();
	Nk = get_Nk();
	Mk = get_Mk();
	for (int i = 0; i < MP; ++i) {

		tr_Nk = tr_Nk + Nk(i, i);
		tr_Mk = tr_Mk + Mk(i, i);
	}
	double ck = tr_Nk / (tr_Mk + 0.0000001);
	//cout << ck << endl;
	if (ck >= 1)
		lamda = ck;
	else if (ck < 1)
		lamda = 1;
	return lamda;
}

//Ç¿¸ú×Ù¿¨¶ûÂü
MatrixXf& Kalman::STFK_predict(int MP, double& vx, double& vy, double& yetax, double& yetay) {

	lamda = get_lamda(MP);
	//cout << lamda << endl;

	// update the state: x'(k) = A*x(k)
	statePre = transMat * stateOpt;

	// update error covariance matrices : tmp1 = A*P(k)
	tmp1 = transMat * errorCovOpt;

	// P'(k) = lamda * temp1*At + Q
	beta = get_beta(vx, vy, yetax, yetay);
	cout << beta * 1 << endl;
	errorCovpre = lamda * tmp1 * transMat.transpose() + beta * processNoiseCov;

	//iteration
	stateOpt = statePre;
	errorCovOpt = errorCovpre;

	return statePre;
}



//×ÔÊÊÓ¦¿¨¶ûÂü
MatrixXf& Kalman::AK_predict(double& vx, double& vy, double& yetax, double& yetay) {

	// update the state: x'(k) = A*x(k)
	statePre = transMat * stateOpt;

	// update error covariance matrices : temp1 = A*P(k)
	tmp1 = transMat * errorCovOpt;

	// P'(k) = temp1*At + Q
	beta = get_beta(vx, vy, yetax, yetay);
	//cout << beta *1 << endl;
	//beta = get_beta_plus(vx, vy);
	errorCovpre = tmp1 * transMat.transpose() + beta * processNoiseCov;

	//iteration
	stateOpt = statePre;
	errorCovOpt = errorCovpre;

	return statePre;

}


//ËÙ¶È×ÔÊÊÓ¦¿¨¶ûÂü
MatrixXf& Kalman::AKV_predict(double vx, double vy, double yetax, double yetay, double V_TL, double V_TH) {

	// update the state: x'(k) = A*x(k)
	statePre = transMat * stateOpt;

	// update error covariance matrices : temp1 = A*P(k)
	tmp1 = transMat * errorCovOpt;

	// P'(k) = temp1*At + Q
	beta = get_beta(vx, vy, yetax, yetay);
	//cout << beta<< endl;

	double Vxy = sqrt(vx * vx + vy * vy);

	if (Vxy <= V_TL) {
		errorCovpre = tmp1 * transMat.transpose() + beta * processNoiseCov_Slow;
		cout << "slow" << endl;
	}

	else if (Vxy > V_TL && Vxy < V_TH) {
		errorCovpre = tmp1 * transMat.transpose() + beta * processNoiseCov;
		cout << "median" << endl;
	}
	else if (Vxy >= V_TH) {
		errorCovpre = tmp1 * transMat.transpose() + beta * processNoiseCov_Fast;

	}

	//iteration
	stateOpt = statePre;
	errorCovOpt = errorCovpre;

	return statePre;

}


//±ê×¼¿¨¶ûÂü
MatrixXf& Kalman::predict() {

	// update the state: x'(k) = A*x(k) + B*u
	if (cP > 0)
	{
		statePre = transMat * stateOpt + contrMat * stateOpt(2, 0);
	}
	else
	{
		statePre = transMat * stateOpt;
	}
	//cout << stateOpt(2, 0) << endl;

	// update error covariance matrices : tmp1 = A*P(k)
	tmp1 = transMat * errorCovOpt;

	// P'(k) = temp1*At + Q
	errorCovpre = tmp1 * transMat.transpose() + processNoiseCov;

	//iteration
	stateOpt = statePre;
	errorCovOpt = errorCovpre;

	return statePre;

}


MatrixXf& Kalman::correct(MatrixXf& measurement) {
	// tmp2 = H*P'(k)
	//tmp2 = measureMat * errorCovpre;
	tmp2 = errorCovpre * measureMat.transpose();

	// tmp3 = tmp2*H + R
	//tmp3 = tmp2*measureMat.transpose() + measureNosiseCov;
	tmp3 = measureMat * tmp2 + measureNosiseCov;

	// tmp4 = inv(tmp3)*temp2 = Kt(k) /----/ (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	//tmp4 = tmp3.inverse() * tmp2;
	tmp4 = tmp2 * tmp3.inverse();

	//K(k)
	//Kgain = tmp4.transpose();
	Kgain = tmp4;

	// tmp5 = z(k) - H*x'(k)
	tmp5 = measurement - measureMat * statePre;

	// x(k) = x'(k) + K(k)*temp5
	stateOpt = statePre + Kgain * tmp5;

	// P(k) = P'(k) - K(k)*H*P'(k)       
	//errorCovOpt = errorCovpre - Kgain *tmp2;
	errorCovOpt = errorCovpre - Kgain * measureMat * errorCovpre;

	return stateOpt;
}


MatrixXf& Kalman::AKV_correct(MatrixXf measurement, double vx, double vy) {

	double Vxy = sqrt(vx * vx + vy * vy);

	// tmp2 = H*P'(k)
	tmp2 = errorCovpre * measureMat.transpose();

	// tmp3 = tmp2*Ht + R
	tmp3 = measureMat * tmp2 + measureNosiseCov;

	// tmp4 = inv(tmp3)*temp2 = Kt(k) /----/ (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	tmp4 = tmp2 * tmp3.inverse();

	//K(k)
	Kgain = tmp4;

	// tmp5 = z(k) - H*x'(k)
	tmp5 = measurement - measureMat * statePre;

	// x(k) = x'(k) + K(k)*temp5
	if (Vxy <= 0) {
		stateOpt(0, 0) = measurement(0, 0);//measurement(0, 0);
		stateOpt(3, 0) = measurement(1, 0);//measurement(1, 0);
	}
	else {
		stateOpt = statePre + Kgain * tmp5;
	}

	// P(k) = P'(k) - K(k)*H*P'(k)                                                                                             
	errorCovOpt = errorCovpre - Kgain * measureMat * errorCovpre;

	return stateOpt;
}