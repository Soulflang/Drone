
//  structs.h
//  opencv_test2
//
//  Created by Morten Due Christiansen on 31/03/2017.
//  Copyright © 2017 Morten Due Christiansen. All rights reserved.
//

#ifndef structs_h
#define structs_h
#include "opencv2/core.hpp"
#include <string.h>
using namespace cv;
struct Circle {
	Point2f center;
	float radius;
};

struct QrCode {
	Point screenCoordinate;
	Point2f worldCoordinate;
	String data;
};

#endif /* structs_h */
