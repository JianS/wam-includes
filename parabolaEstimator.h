/*
 * Parabola estimator is for given the object pose information from the vision system, estimating the 
 * trajactory of the object for predicting. Ex, estimate the contact point if we want to catch the 
 * object. 
 * 
 * 
 * 	Author: js
 * 	06/30/2014
 */


#ifndef PARABOLAESTIMATOR_H
#define PARABOLAESTIMATOR_H

#include <iostream>
#include <vector>
#include <utility>
#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <gsl/gsl_fit.h>

#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>

#include <boost/circular_buffer.hpp>

using namespace barrett;

BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

class parabolaEstimator : public systems::SingleIO<pose_type, pose_type>
{

#define c_w(i) (gsl_vector_get(c_w,(i)))
#define c_x(i) (gsl_vector_get(c_x,(i)))
#define c_y(i) (gsl_vector_get(c_y,(i)))
#define c_z(i) (gsl_vector_get(c_z,(i)))

#define sizeFOE 7 // buffer size for the first order estimator
#define sizeSOE 50 // buffer size for the second order estimator
#define sizeFOEv 10 // buffer size for the first order estimator velocity

public:
	parabolaEstimator(pose_type startPose, double period, cp_type acc,
						size_t n = 100, const std::string& sysName = "Parabola_Estimator") :
		systems::SingleIO<pose_type,pose_type>(sysName),
		_2ndOutput(this, startPose),
		_n(n), _i(0), _period(period), _IfFreeFly(false),
		_acc(acc),
		_startPose(startPose), _retPose(startPose), _1stret(startPose), _2ndret(startPose),
		_vision_buff(_n),
		_FOE_Vx(sizeFOEv), _FOE_Vy(sizeFOEv), _angularVelocityBuff(20)
	{
		clearBuff();
	}
	virtual ~parabolaEstimator(){ mandatoryCleanUp(); }

//	SingleOutput<pose_type> SecondOrderEsiOutput;


	double getCurrentTime() {
		struct timespec ts;
		double t = 0.0;

		if (!clock_gettime( CLOCK_REALTIME, &ts) )
			t = (double)ts.tv_sec + ts.tv_nsec*1.0e-9;
		else std::cout<< "\nError!...(Getting current time.)\n ::parabolaEstimator::getCurrentTime()\n" << std::endl;

		return t;
	}

	bool clearBuff(){
		bool ret = true;

		_vision_buff.clear();
		_FOE_Vx.clear();
		_FOE_Vy.clear();
		_angularVelocityBuff.clear();

		if (_vision_buff.empty() && _FOE_Vx.empty() && _FOE_Vy.empty() && _angularVelocityBuff.empty())
			printf("\nParabolaEstimator buffer is emptyed.\n");
		else{
			printf("\nParabolaEstimator Failed emptying buffer.\n");
			ret = false;
		}
		return ret;
	}

	bool secondOrderEstimator(){
		static size_t inter = 10;

		static gsl_matrix* X = gsl_matrix_alloc(sizeSOE, 3);
		static gsl_matrix* cov_x = gsl_matrix_alloc(3, 3);
		static gsl_matrix* cov_y = gsl_matrix_alloc(3, 3);
		static gsl_matrix* cov_z = gsl_matrix_alloc(3, 3);

		static gsl_vector* Y_x = gsl_vector_alloc(sizeSOE);
		static gsl_vector* Y_y = gsl_vector_alloc(sizeSOE);
		static gsl_vector* Y_z = gsl_vector_alloc(sizeSOE);

		static gsl_vector* c_x = gsl_vector_alloc(3);
		static gsl_vector* c_y = gsl_vector_alloc(3);
		static gsl_vector* c_z = gsl_vector_alloc(3);

		static gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (sizeSOE, 3);

		if (_vision_buff.size() < sizeSOE*inter){
			return false;
		}

		size_t j = 0;
		for (size_t i = 0; i < sizeSOE; i++) {
			gsl_matrix_set(X, i, 0, 1.0);
			gsl_matrix_set(X, i, 1, _vision_buff[j].second);
			gsl_matrix_set(X, i, 2, _vision_buff[j].second*_vision_buff[j].second);

			gsl_vector_set(Y_x, i, _vision_buff[j].first.get<0>()[0]);
			gsl_vector_set(Y_y, i, _vision_buff[j].first.get<0>()[1]);
			gsl_vector_set(Y_z, i, _vision_buff[j].first.get<0>()[2]);
			j = j + inter;
		}

		double chisq_x,chisq_y,chisq_z;
		gsl_multifit_linear (X, Y_x, c_x, cov_x, &chisq_x, work);
		gsl_multifit_linear (X, Y_y, c_y, cov_y, &chisq_y, work);
		gsl_multifit_linear (X, Y_z, c_z, cov_z, &chisq_z, work);

		_2ndret.get<0>()[0] = c_z(0);
		_2ndret.get<0>()[1] = c_z(1);
		_2ndret.get<0>()[2] = c_z(2);

		_2ndret.get<1>().w() = chisq_z;

		_2ndret.get<1>().x() = c_x(0);
		_2ndret.get<1>().y() = c_x(1);
		_2ndret.get<1>().z() = c_x(2);

		return true;
	}

	double average(boost::circular_buffer<double> input){
		size_t n = input.size();
		double sum = 0;

		for (size_t i = 0; i < n; i++){
			sum += input[i];
		}

		return (sum/n);
	}

	Eigen::Quaterniond averageQ(boost::circular_buffer<Eigen::Quaterniond> input){
		static size_t i = 0;

		Eigen::Quaterniond ret;
		size_t n = input.size();
		Eigen::Vector3d sum_axis = Eigen::Vector3d::Zero();
		double sum_angle = 0.0;
		double angle = 0.0;
		Eigen::AngleAxisd aa;
//		sum_axis.Zero();

		for (size_t i = 0; i < n; i++){
			aa = input[i];
			sum_axis = sum_axis + aa.axis();

			angle = aa.angle();
//			if (angle > M_PI) angle = angle - 2*M_PI;
			if (angle == 2*M_PI)
				std::cout << "\n Weird angle: " << angle << "; aa.angle:" << aa.angle() << "; n:" << n << "; i:"<< i << std::endl;
			else
				sum_angle += angle;
		}

		sum_axis.normalize();

		angle = sum_angle/n;
//		while (angle < 0) angle += 2*M_PI;
//		while (angle >= 2*M_PI) angle -= 2*M_PI;

		if (angle < 0 || angle >= 2*M_PI) std::cout << "\n Weird sum angle: " << angle << std::endl;

		aa = Eigen::AngleAxisd(angle, sum_axis);
		ret = aa;

		/*if ((i+1)%200 == 0) {
			printf("\n Angle: %f rad.", angle);
			std::cout << "Rotation axis: " << sum_axis << std::endl;
		}*/
		i++;
		return ret;
	}

	bool FirstOrderEstimator(){

		static double z_acc = 0;
		static double max_z_acc = 0;
		static double c1_z_last = 0;

		//static double g = 9;
		//static double apex = -10.0;
		//static bool IfFinished = false;

		static double X[sizeFOE];
		static double Y_x[sizeFOE];
		static double Y_y[sizeFOE];
		static double Y_z[sizeFOE];

		double c0_x, c1_x, cov00_x, cov01_x, cov11_x, chisqq_x;
		double c0_y, c1_y, cov00_y, cov01_y, cov11_y, chisqq_y;
		double c0_z, c1_z, cov00_z, cov01_z, cov11_z, chisqq_z;

		size_t nb = _vision_buff.size();

		if (nb < sizeFOE) {
			printf("\n vision buff size < size of FOE ! ::parabolaEstimator::FirstOrderEstimator\n");
			return false;
		}

		for (size_t i = 0; i < sizeFOE; i++){
			X[i] = _vision_buff[nb-i-1].second;
			Y_x[i] = _vision_buff[nb-i-1].first.get<0>()[0];
			Y_y[i] = _vision_buff[nb-i-1].first.get<0>()[1];
			Y_z[i] = _vision_buff[nb-i-1].first.get<0>()[2];
		}

		gsl_fit_linear (X, 1, Y_x, 1, sizeFOE ,&c0_x, &c1_x, &cov00_x, &cov01_x, &cov11_x, &chisqq_x);
		gsl_fit_linear (X, 1, Y_y, 1, sizeFOE ,&c0_y, &c1_y, &cov00_y, &cov01_y, &cov11_y, &chisqq_y);
		gsl_fit_linear (X, 1, Y_z, 1, sizeFOE ,&c0_z, &c1_z, &cov00_z, &cov01_z, &cov11_z, &chisqq_z);

		z_acc = (c1_z - c1_z_last)/_period;
		if (z_acc > max_z_acc) max_z_acc = z_acc;

		// Detecting free fly
		if (max_z_acc > 30 && z_acc < -6 && !_IfFreeFly){
			clearBuff();
			_IfFreeFly = true;
			_apex[0] = -10;_apex[1] = -10;_apex[2] = -10;
			//IfFinished = false;
			return true;
		}

		_FOE_Vx.push_back(c1_x);
		_FOE_Vy.push_back(c1_y);

		double Vx = average(_FOE_Vx);
		double Vy = average(_FOE_Vy);
		double Vz = c1_z;

		double current_x = _vision_buff[nb-1].first.get<0>()[0];
		double current_y = _vision_buff[nb-1].first.get<0>()[1];
		double current_z = _vision_buff[nb-1].first.get<0>()[2];

		/*******************************************/


		pose_type ret = _startPose;
		static pose_type last_ret = _startPose;



		double current_t = _vision_buff[nb-1].second;
		if (current_z > _apex[2]) {
			_apex = _vision_buff[nb-1].first.get<0>();
			_t2apex = current_t;
		}
		//double esti_apex = apex;

		double delta_t2apex = fabs(Vz/_acc[2]);

		if (Vz > 0.5) {
			_apex[2] = current_z + fabs(Vz*Vz/(2*_acc[2]));
			_t2apex = current_t + delta_t2apex;
			_apex[0] = current_x + Vx*delta_t2apex + 0.5*_acc[0]*pow(delta_t2apex,2);
			_apex[1] = current_y + Vy*delta_t2apex + 0.5*_acc[1]*pow(delta_t2apex,2);
		}

		/*double d = 0.3; // The distance from the apex to the catch point
		double t2catch = t2apex + sqrt(2*d/g);


		if (t2catch < 0 || IfFinished) {
			ret = last_ret;
			IfFinished = true;
		}
		else {
			ret.get<0>()[0] = current_x + Vx*t2catch;
			ret.get<0>()[1] = current_y + Vy*t2catch;
			ret.get<0>()[2] = esti_apex - d;
		}

		if (!_IfFreeFly) ret = last_ret;

		_2ndret = ret;*/
		/*******************************************/

		// Angular velocity
		Eigen::Quaterniond delta_ori, ori;

		if (nb >= 2){
			delta_ori =_vision_buff[nb-1].first.get<1>() * _vision_buff[nb-2].first.get<1>().inverse();
		}

		// make sure the angle is between 0~pi; w = cos(theta/2);
		if (delta_ori.w() < 0){
			delta_ori.w() = -delta_ori.w();
			delta_ori.x() = -delta_ori.x();
			delta_ori.y() = -delta_ori.y();
			delta_ori.z() = -delta_ori.z();
		}


		Eigen::AngleAxisd aa = delta_ori;
//		Eigen::Vector3d unitX = Eigen::Vector3d::UnitX();
//		Eigen::Quaterniond unitQ = Eigen::Quaterniond::Identity();


		double angle = aa.angle();
		static double angle_last = angle;
		double D_angle;

/*		if ( angle == 0) {
			angle = angle_last;
		}
*/

		static double period = 0.002;
		D_angle = (angle - angle_last)/(period*period);

		static double D_angle_last = D_angle;
		double DD_angle;
		DD_angle = (D_angle - D_angle_last)/period;

//		if (aa.axis().dot(unitX) < 0.0) {
//		if (delta_ori.dot(unitQ) < 0) {

		// 0, 1st, 2nd order Low-pass filter for angular velocity
		if ( ((angle > 0.05) && (fabs(2*M_PI-angle) > 0.05)) || ( angle == 0) ){ //0.05 rad/cycle = 0.05*500 = 25 rad/sec
//			printf("\nAngular Velocity too big! %f rad/cycle.", angle);
		}
		else {
			if ( abs(D_angle) < 1500 && fabs(D_angle) >= 0){ //1st order (rad/sec^2)
				if ( abs(DD_angle) < 1000000 && fabs(DD_angle) >= 0) { //2nd order (rad/sec^3)
					_angularVelocityBuff.push_back(delta_ori);
				}
			}
		}

		if (_angularVelocityBuff.size() > 0){
			ori = averageQ(_angularVelocityBuff);
		}
		else {
			ori = delta_ori;
		}

		/*******************************************/

		/*_1stret.get<0>()[0] = c1_x;
		_1stret.get<0>()[1] = c1_y;
		_1stret.get<0>()[2] = c1_z;*/
		_1stret.get<0>()[0] = Vx;
		_1stret.get<0>()[1] = Vy;
		_1stret.get<0>()[2] = Vz;

		_1stret.get<1>() = ori;

		_2ndret.get<1>() = delta_ori;
		/*_1stret.get<1>().w() = c1_x;
		_1stret.get<1>().x() = Vx;
		_1stret.get<1>().y() = c1_y;
		_1stret.get<1>().z() = Vy;
*/
		c1_z_last = c1_z;
		last_ret = ret;
		angle_last = angle;
		D_angle_last = D_angle;

		return true;
	}



	class SecondOrderEsiOutput : public SingleOutput<pose_type>
	{
	public:
		SecondOrderEsiOutput(System* parentSys, pose_type init) :
			SingleOutput<pose_type>(parentSys),
			_init(init), _buf(init)
		{
			this->outputValue->setData(&_init);
		}
		~SecondOrderEsiOutput(){}

		void setOutputData(pose_type input){
			_buf = input;
			this->outputValue->setData(&_buf);
		}

		pose_type _init, _buf;
	};

	SecondOrderEsiOutput _2ndOutput;

	cp_type getApex(){
		return _apex;
	}
	double getT2apex(){ // return the time when apex is arrived.
		return _t2apex;
	}

	pose_type getStartPose(){
		return _startPose;
	}

	bool IfFreeFly(){
		return _IfFreeFly;
	}

	bool IfVelBuffFull(){
		return _vision_buff.full();
	}

	cp_type getObjAcc(){
		return _acc;
	}

protected:
	size_t _n; // size of the buffer for fitting the parabola
	size_t _i; // iteration number
	double _period;
	bool _IfFreeFly;
	double _t2apex;
	cp_type _apex, _acc;
	pose_type _startPose, _retPose, _1stret, _2ndret;
//	double _startTime;
	std::pair<pose_type, double> _logBuff;
	boost::circular_buffer< std::pair<pose_type, double> > _vision_buff;
	boost::circular_buffer<double> _FOE_Vx, _FOE_Vy;
	boost::circular_buffer<Eigen::Quaterniond> _angularVelocityBuff;

	virtual void operate() {
		if (this->input.valueDefined() ){
			_logBuff.first = this->input.getValue();
			_logBuff.second = getCurrentTime();

			_vision_buff.push_back(_logBuff);
		}
		else {
			printf("\nNo vision data received!! ::parabolaEstimator::operate()\n");
			return;
		}

		if (_vision_buff.size() >= sizeFOE) {
			FirstOrderEstimator();
			_retPose = _1stret;
		}
		else {
			//_retPose = _startPose;
			//if (_vision_buff.size() == 5)
			//printf("parabolaEstimator's buff is too empty...buff size: %lu. \n", _vision_buff.size());

		}

		if (_vision_buff.size() < 4) {
			printf("PEbuffSize:%lu of %lu;", _vision_buff.size(), _vision_buff.capacity());
			fflush(stdout);
		}

//		if (_vision_buff.full() && _IfFreeFly){
//			secondOrderEstimator();
//			_retPose = _2ndret;
//		}
		else{
//			_2ndret = _startPose;
		}
		_2ndOutput.setOutputData(_2ndret);

		this->outputValue->setData(&_retPose);

		_i++;
	}

private:
	DISALLOW_COPY_AND_ASSIGN(parabolaEstimator);

#undef  c_w
#undef  c_x
#undef  c_y
#undef  c_z
#undef  sizeFOE
#undef  sizeSOE
#undef  sizeFOEv
};

#endif
