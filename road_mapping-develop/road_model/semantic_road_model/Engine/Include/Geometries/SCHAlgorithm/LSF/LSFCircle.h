#ifndef _H_CIRCLE_
#define _H_CIRCLE_

//
//						 circle.h
//
/************************************************************************
			DECLARATION OF THE CLASS CIRCLE
************************************************************************/
// Class for Circle
// A circle has 7 fields: 
//     a, b, r (of type reals), the circle parameters
//     s (of type reals), the estimate of sigma (standard deviation)
//     g (of type reals), the norm of the gradient of the objective function
//     i and j (of type int), the iteration counters (outer and inner, respectively)

class LSFCircle
{
public:

	// The fields of a Circle
	reals a, b, r, s, g, Gx, Gy;
	int i, j;

	// constructors
	LSFCircle();
	LSFCircle(reals aa, reals bb, reals rr);

	// routines
	void print(void);

	// no destructor we didn't allocate memory by hand.
};

#endif
