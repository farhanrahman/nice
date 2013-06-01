#ifndef WEAK_CLASSIFIER_H
#define WEAK_CLASSIFIER_H

#include "ImageRepresentation.h"

namespace nice_detector{
class WeakClassifier  
{

public:

	WeakClassifier();
	virtual ~WeakClassifier();

	virtual bool update(ImageRepresentation* image, Rect ROI, int target);

	virtual int eval(ImageRepresentation* image, Rect ROI);

	virtual float getValue (ImageRepresentation* image, Rect  ROI);

	virtual int getType();

};
}

#endif
