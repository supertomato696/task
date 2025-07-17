#include "Geometries/SCHAlgorithm/LSF/mystuff.h"

reals pythag(reals a, reals b)
{
	reals absa = abs(a), absb = abs(b);
	if (absa > absb) return absa*sqrt(One + SQR(absb / absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(One + SQR(absa / absb)));
}