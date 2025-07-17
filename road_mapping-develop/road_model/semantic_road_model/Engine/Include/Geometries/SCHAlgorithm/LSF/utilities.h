#ifndef _H_LSF_UTILITIES
#define  _H_LSF_UTILITIES

reals OptimalRadius(LSFData& data, LSFCircle& circle);
void RandomNormalPair(reals& x, reals& y);
reals Sigma(LSFData& data, LSFCircle& circle);
reals SigmaReduced(LSFData& data, LSFCircle& circle);
reals SigmaReducedForCenteredScaled(LSFData& data, LSFCircle& circle);
reals SigmaReducedNearLinearCase(LSFData& data, LSFCircle& circle);
void SimulateArc(LSFData& data, reals a, reals b, reals R, reals theta1, reals theta2, reals sigma);
void SimulateRandom(LSFData& data, reals Window);

#endif //_H_LSF_UTILITIES
