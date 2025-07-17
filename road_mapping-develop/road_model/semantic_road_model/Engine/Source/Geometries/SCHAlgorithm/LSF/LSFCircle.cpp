#include "Geometries/SCHAlgorithm/LSF/mystuff.h"
#include "Geometries/SCHAlgorithm/LSF/LSFCircle.h"

using namespace std;
/************************************************************************
BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor

LSFCircle::LSFCircle()
{
	a = 0.; b = 0.; r = 1.; s = 0.; i = 0; j = 0;
}

// Constructor with assignment of the circle parameters only

LSFCircle::LSFCircle(reals aa, reals bb, reals rr)
{
	a = aa; b = bb; r = rr;
}

// Printing routine

void LSFCircle::print(void)
{
	cout << endl;
	cout << setprecision(10) << "center (" << a << "," << b << ")  radius "
		<< r << "  sigma " << s << "  gradient " << g << "  iter " << i << "  inner " << j << endl;
}