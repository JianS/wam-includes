/*
 * This is a sub-class under OptiTrackVision that is used for dealing with the 
 * vision thread, including grabing information and filering the object pose information
 * 
 *
      Author: js
 */

#ifndef VISIONFILTER_H
#define VISIONFILTER_H

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/FrameListener.h>

#include <boost/circular_buffer.hpp>
#include <vector>
#include <utility>
#include <time.h>

#include <gsl/gsl_multifit.h>

#include <barrett/units.h>

#include "../include/ObjectSet.h"
/*
* Filtering vision object information.
* This is a second-order filter, or constant acceleration filter
*
* \param
* 'n_rb' is the number of rigidboies we tracking in the Mocapframe data;
* 'n' is the size of the filter
*/
class VisionFilter
{
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

#define c_x(i) (gsl_vector_get(c_x,(i)))
#define c_y(i) (gsl_vector_get(c_y,(i)))
#define c_z(i) (gsl_vector_get(c_z,(i)))

public:

	/*
	 * Constructer: n_rb is for the number of rigidbodies;
	 * 				n is the length of the filter
	 * 				'mulRB_obj' flag is telling if there is an object defined with multible rigidbodies in Motive
	 * 				'mrbo_start' & 'mrbo_end' only effect when mulRB_obj is true and
	 * 				they are showing the range of the id() for these rigidbodies
	 */
	VisionFilter(FrameListener* FL, size_t n_rb , size_t n, ObjectSet objectSet) :
		_FL(FL), _n_rb(n_rb), _n(n), _objectSet(objectSet),
		_isPkgbuffDef(false), _loggedPos_vec(n_rb), _loggedRecv_t(n)
	{
		setTransferMatrix();
		for (size_t i = 0; i<n_rb; i++)
			_loggedPos_vec[i].resize(n);
	}

	~VisionFilter(){};

	double TimeSpec2double(struct timespec ts) {
		double t = (double)ts.tv_sec + ts.tv_nsec*1.0e-9;
		return t;
	}

	double getCurrentTime() {
		struct timespec ts;
		double t = 0.0;

		if (!clock_gettime( CLOCK_REALTIME, &ts) )
			t = (double)ts.tv_sec + ts.tv_nsec*1.0e-9;
		else std::cout<< "Error!...(Getting current time.)" << std::endl;

		return t;
	}

	void setTransferMatrix() {
			double tm[] = {-0.0026771,-0.010778,1.0087,-0.29405,
					1.0034,0.014631,0.00328,-0.78748,
					-0.011528,1.0051,0.0042434,-1.0574,
					0,0,0,1
				 };
			_TM.assign(tm,tm+12);
	}

	std::vector<double> getTransferMatrix(){
		return _TM;
	}

	cp_type Cam2Wam(cp_type cp) {
		cp_type cp_t;

		cp_t[0] = _TM[0]*cp[0] + _TM[1]*cp[1] + _TM[2]*cp[2] + _TM[3];
		cp_t[1] = _TM[4]*cp[0] + _TM[5]*cp[1] + _TM[6]*cp[2] + _TM[7];
		cp_t[2] = _TM[8]*cp[0] + _TM[9]*cp[1] + _TM[10]*cp[2] + _TM[11];

		return cp_t;
	}

	void ClearFilterBuff() {
		bool success = true;

		for (int i = 0; i<_n_rb; i++) {
//		for (int i = 0; i<_loggedPos_vec.size(); i++) {
			while (!_loggedPos_vec[i].empty() )
				_loggedPos_vec[i].pop_back();
		}

		while (!_loggedRecv_t.empty() )
			_loggedRecv_t.pop_back();

		for (int i = 0; i<_n_rb; i++) {
			if (!_loggedPos_vec[i].empty())
				success = false;
		}
		if ( success && _loggedRecv_t.empty() )
			printf("Filter buffer is emptyed.\n");
		else
			printf("Failed emptying filter buffer.\n");
	}


	Point3f getObjCenter(MocapFrame framebuff, int start_id, int end_id, bool * success = 0){
		int rb_id;
		std::vector<Point3f> pos_buff;
		Point3f ret;
		double sumX = 0,sumY = 0,sumZ = 0;
		bool success_flag = false;
		size_t i;

		for (i = 0; i < framebuff.rigidBodies().size(); i++) {
			rb_id = framebuff.rigidBodies()[i].id();
			if ( (rb_id>=start_id)&&(rb_id<=end_id) && framebuff.rigidBodies()[i].err() != 0 ) {
				pos_buff.push_back( framebuff.rigidBodies()[i].location() );
			}
		}

		if ( pos_buff.size() > 0 ) {
			for (i = 0; i < pos_buff.size(); i++){
				sumX += pos_buff[i].x;
				sumY += pos_buff[i].y;
				sumZ += pos_buff[i].z;
			}

			ret.x = sumX/pos_buff.size();
			ret.y = sumY/pos_buff.size();
			ret.z = sumZ/pos_buff.size();

			success_flag = true;
//			printf("\nCurrent object is tracked.");
		}
		else {
//			printf("\nAll the rigidbodies of the object is untracked! (::VisionFilter::getMulRB_ObjCenter)");
			fflush(stdout);
		}

		if (success) *success = success_flag;

		return (ret);
	}

	bool grabPkg() {
//		double t1 = getCurrentTime();

		bool ifsuccess = false, ret = false, track_flag;
		_grabBuff = _FL->tryPop(&ifsuccess);

		if ( (ifsuccess) && (_grabBuff.first.rigidBodies().size() > 0)) {
		//if ( ifsuccess) {

			if (!_isPkgbuffDef)
			{
//				printf("\nPkg buff undef! rb size: %ld. (::VisionFilter::grabPkg)", _grabBuff.first.rigidBodies().size());
//				fflush(stdout);
				ClearFilterBuff();
			}

			_pkgBuff = _grabBuff;
			_isPkgbuffDef = true;

			MocapFrame framebuff = _pkgBuff.first;

			int rb_id = 0;
//			for (size_t i = 0; i < framebuff.rigidBodies().size(); i++) {
			for (int i = 0; i < _n_rb; i++) {
				rb_id = framebuff.rigidBodies()[i].id();
				track_flag = false;

//				if ( rb_id <= _n_rb ) {

//					_loggedPos_vec[rb_id-1].push_back(_pkgBuff.first.rigidBodies()[i].location());


						//if ( (rb_id>=_mrbo_start)&&(rb_id<=_mrbo_end) ) {
//						if ( rb_id == _mrbo_start){
				Point3f objCoord = getObjCenter(framebuff, _objectSet.Objects()[i].first, _objectSet.Objects()[i].second, &track_flag);
//				printf("Is rb_id:%d tracked? %d.\n", rb_id, track_flag);
				if (track_flag)
					_loggedPos_vec[rb_id-1].push_back(objCoord);

//					i = _mrbo_end - 1;// TODO
//						}
//						else
//							_loggedPos_vec[rb_id-1].push_back(_pkgBuff.first.rigidBodies()[i].location());
//				}
//				else{
//					printf("\n rb_id is bigger than the total number of rigidbodies! (::VisionFilter::grabPkg) rb_id: %d, n_rb: %d.", rb_id, _n_rb);
//					fflush(stdout);
//				}
			}

			_loggedRecv_t.push_back( TimeSpec2double(_pkgBuff.second) );

			ret = true;
		}

//		double t2 = getCurrentTime();
//		printf("Grabbing time: %f sec.\n", (t2-t1));

		return (ret);
	}

	std::pair<MocapFrame, struct timespec> getLatestPkg(){

		if (!_isPkgbuffDef) {
			printf("\nPkg buff undef! (::VisionFilter::getLatestPkg)\n");
			fflush(stdout);
		}
		//int i=0;
		do{
			//if (i > 0) printf("\n i is too big!!: %d", i);
			grabPkg();
			//i++;
		}
		while ( !_isPkgbuffDef );

		return _pkgBuff;
	}

	/*
	 * Return the unfiltered Catesian position of rigidbody, indicated by its ID(this ID starts at 1)
	 */
	cp_type getLatestCP(size_t rb_id) {
		getLatestPkg();
		cp_type ret;
		if ( _loggedPos_vec[rb_id-1].size() > 0) {
			ret[0] = _loggedPos_vec[rb_id-1].back().x;
			ret[1] = _loggedPos_vec[rb_id-1].back().y;
			ret[2] = _loggedPos_vec[rb_id-1].back().z;
		}
		else
			printf("\n Do not have any logged info for rb_id: %lu. (::VisionFilter::getLatestCP)", rb_id);

		return ret;
	}

/*
* Start second-order filtering here.
* 'rb_id' is the ID of targeting rigidbody
*/
	cp_type Positionfiltering(size_t rb_id) {

//		double t1 = getCurrentTime();
		//printf("Entered filtering...\n"); fflush(stdout);

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

		for (size_t i = 0; i < _n; i++){
			gsl_matrix_set(X, i, 0, 1.0);
			gsl_matrix_set(X, i, 1, _loggedRecv_t[i]);
			gsl_matrix_set(X, i, 2, _loggedRecv_t[i]*_loggedRecv_t[i]);

			gsl_vector_set(Y_x, i, _loggedPos_vec[rb_id-1][i].x);
			gsl_vector_set(Y_y, i, _loggedPos_vec[rb_id-1][i].y);
			gsl_vector_set(Y_z, i, _loggedPos_vec[rb_id-1][i].z);
		}

		double chisq_x,chisq_y,chisq_z;
		gsl_multifit_linear (X, Y_x, c_x, cov_x, &chisq_x, work);
		gsl_multifit_linear (X, Y_y, c_y, cov_y, &chisq_y, work);
		gsl_multifit_linear (X, Y_z, c_z, cov_z, &chisq_z, work);
		//gsl_multifit_linear_free (work);

		double ss = getCurrentTime();

		_cp[0] = c_x(0) + c_x(1)*ss + c_x(2)*(ss*ss);
		_cp[1] = c_y(0) + c_y(1)*ss + c_y(2)*(ss*ss);
		_cp[2] = c_z(0) + c_z(1)*ss + c_z(2)*(ss*ss);

//		double t2 = getCurrentTime();
//		printf("Filtering time: %f sec.\n", (t2-t1));

		return _cp;
	}

	bool ReadyFilter(size_t rb_id) {
//		if (!_loggedPos_vec[rb_id-1].full())
//			printf("\nFilter not ready. Size of the buffer[%lu]:%lu.(::visionFilter::ReadyFilter())\n", rb_id-1, _loggedPos_vec[rb_id-1].size());
		return _loggedPos_vec[rb_id-1].full();
	}

	bool IsPkgbuffDef() { return _isPkgbuffDef; }

	void ResetPkgbuff() { _isPkgbuffDef = false; }



private:
	FrameListener* _FL;
	int _n_rb;
	size_t _n;
//	bool _mulRB_obj;
//	size_t _mrbo_start, _mrbo_end;
	ObjectSet _objectSet;
	cp_type _cp;
	std::pair<MocapFrame, struct timespec> _pkgBuff;
	std::pair<MocapFrame, struct timespec> _grabBuff;
	bool _isPkgbuffDef;
	//boost::circular_buffer<Point3f> _loggedPosition;

	/*
	 * The rule of linking rigidbody ID and the vector index is :
	 * array index = rigidbody_ID - 1
	 */
	std::vector< boost::circular_buffer<Point3f> > _loggedPos_vec;
	boost::circular_buffer<double> _loggedRecv_t;

	std::vector<double> _TM; //Transfer Matrix



#undef  c_x
#undef  c_y
#undef  c_z
};

#endif
