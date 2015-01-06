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

#define sizeFOE 25 // buffer size for the first order estimator

public:
	parabolaEstimator(pose_type startPose, double period, size_t n = 250, const std::string& sysName = "Parabola_Estimator") :
		systems::SingleIO<pose_type,pose_type>(sysName),
//		SecondOrderEsiOutput(this),
		_n(n), _i(0), _period(period), _IfFreeFly(false),
		_startPose(startPose), _retPose(startPose),
		_vision_buff(_n)
	{
		clearBuff();
	}
	virtual ~parabolaEstimator(){ mandatoryCleanUp(); }

//	Output<pose_type> SecondOrderEsiOutput;

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

		while (!_vision_buff.empty() ){
			_vision_buff.pop_back();
		}

		if (_vision_buff.empty())
			printf("\nParabolaEstimator buffer is emptyed.\n");
		else{
			printf("\nParabolaEstimator Failed emptying buffer.\n");
			ret = false;
		}
		return ret;
	}

	bool secondOrderEstimator(){
		static gsl_matrix* X = gsl_matrix_alloc(_n, 3);
		static gsl_matrix* cov_x = gsl_matrix_alloc(3, 3);
		static gsl_matrix* cov_y = gsl_matrix_alloc(3, 3);
		static gsl_matrix* cov_z = gsl_matrix_alloc(3, 3);

		static gsl_vector* Y_x = gsl_vector_alloc(_n);
		static gsl_vector* Y_y = gsl_vector_alloc(_n);
		static gsl_vector* Y_z = gsl_vector_alloc(_n);

		static gsl_vector* c_x = gsl_vector_alloc(3);
		static gsl_vector* c_y = gsl_vector_alloc(3);
		static gsl_vector* c_z = gsl_vector_alloc(3);

		static gsl_multifit_linear_workspace * work = gsl_multifit_linear_alloc (_n, 3);

		for (size_t i = 0; i < _vision_buff.size(); i++) {
			gsl_matrix_set(X, i, 0, 1.0);
			gsl_matrix_set(X, i, 1, _vision_buff[i].second);
			gsl_matrix_set(X, i, 2, _vision_buff[i].second*_vision_buff[i].second);

			gsl_vector_set(Y_x, i, _vision_buff[i].first.get<0>()[0]);
			gsl_vector_set(Y_y, i, _vision_buff[i].first.get<0>()[1]);
			gsl_vector_set(Y_z, i, _vision_buff[i].first.get<0>()[2]);
		}

		double chisq_x,chisq_y,chisq_z;
		gsl_multifit_linear (X, Y_x, c_x, cov_x, &chisq_x, work);
		gsl_multifit_linear (X, Y_y, c_y, cov_y, &chisq_y, work);
		gsl_multifit_linear (X, Y_z, c_z, cov_z, &chisq_z, work);

		_retPose.get<0>()[0] = c_z(0);
		_retPose.get<0>()[1] = c_z(1);
		_retPose.get<0>()[2] = c_z(2);

		_retPose.get<1>().x() = c_x(0);
		_retPose.get<1>().y() = c_x(1);
		_retPose.get<1>().z() = c_x(2);

		return true;
	}

	bool FirstOrderEstimator(){

		static double z_acc = 0;
		static double max_z_acc = 0;
		static double c1_z_last = 0;

		static double X[sizeFOE];
		static double Y_x[sizeFOE];
		static double Y_y[sizeFOE];
		static double Y_z[sizeFOE];

		double c0_x, c1_x, cov00_x, cov01_x, cov11_x, chisqq_x;
		double c0_y, c1_y, cov00_y, cov01_y, cov11_y, chisqq_y;
		double c0_z, c1_z, cov00_z, cov01_z, cov11_z, chisqq_z;

		size_t nb = _vision_buff.size();

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

		if (max_z_acc > 15 && z_acc < 0 && !_IfFreeFly){
			clearBuff();
			_IfFreeFly = true;
		}

		_retPose.get<0>()[0] = c0_z;
		_retPose.get<0>()[1] = c1_z;

		_retPose.get<1>().w() = c0_x;
		_retPose.get<1>().x() = c1_x;
		_retPose.get<1>().y() = c0_y;
		_retPose.get<1>().z() = c1_y;

		c1_z_last = c1_z;

		return true;
	}

protected:
	size_t _n; // size of the buffer for fitting the parabola
	size_t _i; // iteration number
	double _period;
	bool _IfFreeFly;
	pose_type _startPose, _retPose;
//	double _startTime;
	std::pair<pose_type, double> _logBuff;
	boost::circular_buffer< std::pair<pose_type, double> > _vision_buff;

	virtual void operate() {
		if (this->input.valueDefined() ){
			_logBuff.first = this->input.getValue();

//			if (_i % 5 == 0){
				_logBuff.second = getCurrentTime();
				_vision_buff.push_back(_logBuff);
//			}
		}
		else {
			printf("\nNo vision data received!! ::parabolaEstimator::operate()\n");
			return;
		}

		if (_vision_buff.size() >= sizeFOE) {
			FirstOrderEstimator();
		}
		else {
			//_retPose = _startPose;
			//if (_vision_buff.size() == 5)
			//printf("parabolaEstimator's buff is too empty...buff size: %lu. \n", _vision_buff.size());
			if (_vision_buff.size() < 15) {
				printf("PEbuffSize:%lu;", _vision_buff.size());
				fflush(stdout);
			}
		}

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
};

#endif
