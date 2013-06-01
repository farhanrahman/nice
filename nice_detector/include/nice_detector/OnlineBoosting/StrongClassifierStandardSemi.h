#ifndef STRONG_CLASSIFIER_STANDARD_SEMI_H
#define STRONG_CLASSIFIER_STANDARD_SEMI_H

#include "StrongClassifier.h"

namespace nice_detector{
class StrongClassifierStandardSemi  : public StrongClassifier
{
public:

	StrongClassifierStandardSemi(int numBaseClassifier, int numWeakClassifier, 
		Size patchSize,  bool useFeatureExchange = false, int iterationInit = 0);

	virtual ~StrongClassifierStandardSemi();
	
	bool updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence);
	void getPseudoValues(ImageRepresentation *image, Rect ROI, float priorConfidence, float* pseudoLambdaInOut, int* pseudoTargetInOut);
	
private:

	bool *m_errorMask;
	float *m_errors;
	float* m_pseudoLambda;
	int* m_pseudoTarget;

};
}

#endif
