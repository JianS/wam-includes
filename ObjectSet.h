/*
 * ObjectSet is the class defining the relationship about how many rigidbodies in the Motive
 * defining a real object. Each element of the vector is a pair of start id and end id for rigidbodies 
 * which are defining this object. If there is only one rigidbody defining, the start and end gonna be 
 * the same. 
 * 
 * NOTE: the id() of rigidbodies always start at 1.
 * 
 * 	Author: js
 * 	04/21/2014
 */


#ifndef OBJECTSET_H
#define OBJECTSET_H

#include <iostream>
#include <vector>
#include <utility>
#include <unistd.h>

class ObjectSet{

public:
	ObjectSet() : _n(2), _objs(_n)  // Define the range in the constructor.
									 //'first' and 'second' means the start and end rigidbody ID# in Motive.
	{
		_objs[0].first = 1; _objs[0].second = 1; // Assuming rigidbody id 1 is always the end-effector
//		_objs[1].first = 2; _objs[1].second = 7;
//		_objs[1].first = 8; _objs[1].second = 8;
		_objs[1].first = 100; _objs[1].second = 100; // 100 means tracking only one markers, this marker will be unidentified.
													 // used when tracking a ball
	}
	virtual ~ObjectSet(){}

	size_t NumOfObjs() {return _n;}
	size_t NumOfRBs() {
		return _objs[_n-1].second;
	}
	std::vector< std::pair<size_t,size_t> > Objects() {return _objs;}

protected:
	size_t _n;
	std::vector< std::pair<size_t,size_t> > _objs;
	
} objectSet;  //Define the unique global varible

#endif
