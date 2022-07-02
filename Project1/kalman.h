
#ifndef  KF_H
#define  KF_H

#include <Eigen/Dense>
using namespace Eigen;

class Kalman {

public:
	/*
	@breif Standard Kalman filter
	*/
	MatrixXf& predict();

	/*
	@brief AFK: "Tracking Touched Trajectory on Capacitive Touch Panels Using Adjustable Weighted Prediction Covariance Matrix."
	@param  vx Velocity in x-axis direction
	@param  vy Velocity in y-axis direction
	@param  yetax  Error in x-axis direction :z(k)-H*x'(k)
	@param  yetay  Error in y-axis direction :z(k)-H*x'(k)
	*/
	MatrixXf& AK_predict(double& vx, double& vy, double& yetax, double& yetay);

	/*
	@brief Adactive velocit adjust
	@param  vx Velocity in x-axis direction
	@param  vy Velocity in y-axis direction
	@param  yetax  Error in x-axis direction(innovation sequence):z(k)-H*x'(k)
	@param  yetay  Error in y-axis direction(innovation sequence):z(k)-H*x'(k)
	@V_L Low threshold of speed
	@V_H High threshold of speed
	*/
	MatrixXf& AKV_predict(double vx, double vy, double yetax, double yetay, double V_L, double V_H);

	/*
	@brief  STFK: "Position stimation and Smooth Tracking with a Fuzzy Logic-Based Adaptive Strong Tracking Kalman Filter for Capacitive Touch Panels."
	@param  measureParams Dimensionality of the measurement.
	@param  vx Velocity in x-axis direction
	@param  vy Velocity in y-axis direction
	@param  yetax  Error in x-axis direction() :z(k)-H*x'(k)
	@param  yetay  Error in y-axis direction :z(k)-H*x'(k)
	*/
	MatrixXf& STFK_predict(int measureParams, double& vx, double& vy, double& yetax, double& yetay);

	// updates the predicted state from the measurement
	MatrixXf& correct(MatrixXf& measurement);

	// updates the predicted state from the measurement
	MatrixXf& AKV_correct(MatrixXf measurement, double vx, double vy);


	/*
	@brief Re-initializes Kalman filter.
	@param dynamParams Dimensionality of the state.
	@param measureParams Dimensionality of the measurement.
	@param controlParams Dimensionality of the control vector.
	*/
	void init(int dynamParams, int measureParams, int controlParams);

	MatrixXf statePre; //predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
	MatrixXf stateOpt; //corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
	MatrixXf transMat; //state transition matrix (A)
	MatrixXf transMat_slow; //state transition matrix (A)
	MatrixXf contrMat; // control matrix (B) (not used if there is no control)
	MatrixXf measureMat; // measurement matrix (H)
	MatrixXf processNoiseCov; // process noise covariance matrix (Q)
	MatrixXf processNoiseCov_Slow; // process noise covariance matrix (Q)  SLOW speed
	MatrixXf processNoiseCov_Fast; // process noise covariance matrix (Q)  Fast speed
	MatrixXf measureNosiseCov; // measurement noise covariance matrix (R)
	MatrixXf measureNosiseCov_Slow; // measurement noise covariance matrix (R) SLOW speed
	MatrixXf measureNosiseCov_Fast; // measurement noise covariance matrix (R) Fast speed
	MatrixXf errorCovpre; // priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)
	MatrixXf Kgain;        //  Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	MatrixXf errorCovOpt;  //   posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

	//STKF
	MatrixXf Vk;
	MatrixXf Nk;
	MatrixXf Mk;
	double rho; //forgetting factor
	double get_lamda(int MP); // calculation suboptimal scaling factor
	double lamda; //suboptimal scaling factor

	//AKF
	double get_beta(double vx, double vy, double yetax, double yetay); //calculation beta
	double get_beta_plus(double vx, double vy); //calculation beta
	double eps;  //A empirically param determined  0.01
	double beta; //A wight is used to adjust the prediction convariance matrix of AKF

	//temporary Mat
	MatrixXf tmp1;
	MatrixXf tmp2;
	MatrixXf tmp3;
	MatrixXf tmp4;
	MatrixXf tmp5;   //innovation sequence vk

private:
	//STKF
	MatrixXf& get_Vk();
	MatrixXf& get_Mk();
	MatrixXf& get_Nk();
	int cP;
};

#endif
