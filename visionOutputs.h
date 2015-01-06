/*
 * This is a sub-class under OptiTrackVision that is used for outputing the
 * information within the package.
 *
      Author: js
 */

#ifndef VISIONOUTPUTS_H
#define VISIONOUTPUTS_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include <boost/circular_buffer.hpp>
#include <time.h>

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/FrameListener.h>

#include "visionFilter.h"

using namespace barrett;

// Outputing the Cartesain Position of the objects
/************************************************************************************/
class visionCP : public systems::System , public systems::SingleOutput<units::CartesianPosition::type>
{
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	//void setTransferMatrix();

	visionCP( VisionFilter* VF , size_t rb_id, const std::string& sysName = "vision_CartesianPosition") :
		systems::System(sysName),
		systems::SingleOutput<units::CartesianPosition::type>(this),
		_VF(VF),
		_t(0),
		_rb_id(rb_id)
	{
//		setTransferMatrix();
		_cp_t[0] = _cp_t[1] = _cp_t[2] = 0.0;
		this->outputValue->setData(&_cp_t);
	}
	virtual ~visionCP() { mandatoryCleanUp(); }

private:
	::VisionFilter* _VF;
	::MocapFrame _framebuff;
	//boost::circular_buffer< std::pair<MocapFrame, struct timespec> >* _pkgsBuff;
	cp_type _cp, _cp_t;
	int _t;
	std::vector<double> _TM; //Transfer Matrix
	size_t _rb_id;

protected:

	virtual void operate(){

		/*_cp[0] = rbBuff.location().z;
		_cp[1] = rbBuff.location().x;
		_cp[2] = rbBuff.location().y;

		_cp[0] = rbBuff.location().x;
		_cp[1] = rbBuff.location().y;
		_cp[2] = rbBuff.location().z;*/
		_VF->getLatestPkg(); // Try grabbing pkg from receiving thread

		if ( _VF->ReadyFilter(_rb_id) ) {

			_cp = _VF->Positionfiltering(_rb_id);

		}
		else {
//			printf("\nFilter not ready. (::visionCP::operate())\n");
			_cp = _VF->getLatestCP(_rb_id);
		}
		//printf("\n _cp[0] = %f, _cp[1] = %f, _cp[2] = %f\n", _cp[0], _cp[1], _cp[2]);
//		std::cout << _cp <<std::endl;

		_cp_t = _VF->Cam2WamPos(_cp);
//		this->outputValue->setData(&_cp);
		this->outputValue->setData(&_cp_t);

		//double t2 = _VF->getCurrentTime();
		//printf("CP outputs operating time: %f sec.\n", (t2-t1));

	}

private:
	DISALLOW_COPY_AND_ASSIGN(visionCP);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
// Outputing the Cartesain Position and Orientation of the objects
/************************************************************************************/
class visionPose : public systems::System , public systems::SingleOutput<pose_type>
{
//	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	//void setTransferMatrix();

	visionPose( VisionFilter* VF , size_t rb_id, bool IsFiltered, const std::string& sysName = "vision_CartesianPosition") :
		systems::System(sysName),
		systems::SingleOutput<pose_type>(this),
		_VF(VF),
		_t(0),
		_rb_id(rb_id), _IsFiltered(IsFiltered)
	{
//		setTransferMatrix();
		_pose.get<0>()[0] = _pose.get<0>()[1] = _pose.get<0>()[2] = 0.f;
		this->outputValue->setData(&_pose);
	}
	virtual ~visionPose() { mandatoryCleanUp(); }

private:
	::VisionFilter* _VF;
	::MocapFrame _framebuff;
	//boost::circular_buffer< std::pair<MocapFrame, struct timespec> >* _pkgsBuff;
	pose_type _pose, _pose_t;
	cp_type _cp, _cp_t;
	::Eigen::Quaterniond _ori, _ori_t;
	int _t;
	std::vector<double> _TM; //Transfer Matrix
	size_t _rb_id;
	bool _IsFiltered;

protected:

	virtual void operate(){

		/*_cp[0] = rbBuff.location().z;
		_cp[1] = rbBuff.location().x;
		_cp[2] = rbBuff.location().y;

		_cp[0] = rbBuff.location().x;
		_cp[1] = rbBuff.location().y;
		_cp[2] = rbBuff.location().z;*/

		_VF->getLatestPkg(); // Try grabbing pkg from receiving thread

		if ( _VF->ReadyFilter(_rb_id) && _IsFiltered) {

			_pose = _VF->getLatestPose(_rb_id);
			_pose.get<0>() = _VF->Positionfiltering(_rb_id);
			_pose.get<1>() = _VF->Orientationfiltering(_rb_id);

		}
		else {
//			printf("\nFilter not ready. (::visionPose::operate())\n");
//			_cp = _VF->getLatestCP(_rb_id);
			_pose = _VF->getLatestPose(_rb_id);
		}
		//printf("\n _cp[0] = %f, _cp[1] = %f, _cp[2] = %f\n", _cp[0], _cp[1], _cp[2]);
//		std::cout << _cp <<std::endl;

		_pose_t.get<0>() = _VF->Cam2WamPos(_pose.get<0>());
		_pose_t.get<1>() = _VF->Cam2WamOri(_pose.get<1>());

		this->outputValue->setData(&_pose_t);

		//double t2 = _VF->getCurrentTime();
		//printf("CP outputs operating time: %f sec.\n", (t2-t1));

	}

private:
	DISALLOW_COPY_AND_ASSIGN(visionPose);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



// Outputing the Time information of the frame
/************************************************************************************/
struct PkgTimeInfo{
	double Cap_t;  // time when a frame is captured. (In the clock time of this mechine)
	double Recv_t; // time when a frame data is received in this com. (In the clock time of this mechine)
};

std::ostream& operator<< ( std::ostream& os, const struct PkgTimeInfo& pkg )
{
	os << pkg.Cap_t << ", " << pkg.Recv_t << std::endl;
	return os;
}


class visionTimeOutput : public systems::System , public systems::SingleOutput< math::Vector<7>::type >
{
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

#define SMPTEperiod 0.0399984
#define multi 10.0
#define ExpTime 0.0006
//#define SecsBeforeToday (1392595200)

public:

	visionTimeOutput( VisionFilter* VF , const std::string& sysName = "vision_TimeInfo") :
		systems::System(sysName), systems::SingleOutput< math::Vector<7>::type >(this), _VF(VF), _t(0)
	{
		//_pkgsBuff = PkgsBuff;
		_t_delay = (SMPTEperiod/multi)/2;

		struct timespec ts;
		clock_gettime( CLOCK_REALTIME, &ts);
		_t_secsBeforeStart = ts.tv_sec;
		_SecsBeforeToday = (ts.tv_sec/(3600*24))*(3600*24);

		std::cout << "t_secsBeforeStart = " << _t_secsBeforeStart << " ; SecsBeforeToday = " << _SecsBeforeToday << std::endl;

	}
	virtual ~visionTimeOutput() { mandatoryCleanUp(); }


private:
	::VisionFilter* _VF;
	struct PkgTimeInfo _t_info;
	::MocapFrame _framebuff;
	//boost::circular_buffer< std::pair<MocapFrame, struct timespec> >* _pkgsBuff;
	struct timespec _tsBuff;
	int _hour, _min, _sec, _fnum, _sub_fnum;
	double _t_delay;
	int _t_secsBeforeStart;
	int _t;
	int _SecsBeforeToday;

	math::Vector<7>::type outBuff;

	pose_type _pose;


protected:

	virtual void operate() {

//		t1 = _VF->getCurrentTime();

		std::pair<MocapFrame, struct timespec> ret = _VF->getLatestPkg();
		_framebuff = ret.first;
		_tsBuff = ret.second;

		_framebuff.timecode(_hour,_min,_sec,_fnum,_sub_fnum);
		_t_info.Cap_t = (double) (_hour*3600 + _min*60 + _sec) + (((double) (_fnum)) * SMPTEperiod)
					+ (((double)(_sub_fnum))*SMPTEperiod/multi) + _t_delay - (_t_secsBeforeStart-_SecsBeforeToday);
		_t_info.Recv_t = (double)_tsBuff.tv_sec - _t_secsBeforeStart + (double)_tsBuff.tv_nsec*1.0e-9;

		outBuff[0] = _t_info.Cap_t;
		outBuff[1] = _t_info.Recv_t;
		/*outBuff[2] = _hour;
		outBuff[3] = _min;
		outBuff[4] = _sec;
		outBuff[5] = _fnum;
		outBuff[6] = _sub_fnum;*/

//		if (_VF->ReadyFilter(2))
			_pose = _VF->getLatestPose(2);
			_pose.get<0>() = _VF->Cam2WamPos(_pose.get<0>());
			_pose.get<1>() = _VF->Cam2WamOri(_pose.get<1>());
//			cp = _VF->Cam2Wam(_VF->Positionfiltering(1));
//		else
//			cp = _VF->Cam2Wam(_VF->getLatestCP(1));

		outBuff[0] = _pose.get<0>()[0];
		outBuff[1] = _pose.get<0>()[1];
//		outBuff[2] = 0; // '0' is just for space different infomation
		outBuff[2] = _pose.get<0>()[2];
		outBuff[3] = _pose.get<1>().w();
		outBuff[4] = _pose.get<1>().x();
		outBuff[5] = _pose.get<1>().y();
		outBuff[6] = _pose.get<1>().z();

		this->outputValue->setData(&outBuff);
		_t++;

//		t2 = _VF->getCurrentTime();
//		printf("Vision Time outputs operating time: %f sec.\n", (t2-t1));
	}

private:
	DISALLOW_COPY_AND_ASSIGN(visionTimeOutput);
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

#undef SMPTEperiod
#undef multi
#undef ExpTime
//#undef SecsBeforeToday

};


#endif //include guard
