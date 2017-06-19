








#include "ardrone/ardrone.h"
#include <zbar.h>
#include <Windows.h>
#include <math.h>
// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
using namespace cv;
using namespace zbar;


const long BOOTUP_TIME = 3000;
const long TAKEOFF_TIME = 7000;
const int CIRCLE_COUNTER = 8;
ARDrone ardrone;

zbar::ImageScanner scanner;

// center point in image
const int imgCenterX = 640 / 2;
const int imgCenterY = 360 / 2;

// Timemanagement
long deltaTime = 0;
long lastTime = 0;

long qrCodeTime = 0;
// storing 3 circles for a average circle center and average
Vec3f circle1, circle2, circle3, circle4, circle5, circle6, circle7, circle8;
bool circleFound;
Point2f QrCodeCenter;
float QrCodeWith;
bool qrCodeFound;
int circleCounter = 0;

cv::Mat frame;

enum STATES { TAKEOFF, SEARCH, ALIGN, PENETRATE, FINDCIRCLE };
STATES currentState;

enum commands { left, right, down, up, NoCommand, hoover, forward, circlePenetration };

commands currentCommand;
struct QrCode {
	Point screenCoordinate;
	Point2f worldCoordinate;
	String data;
};

bool readQrCode(cv::Mat frame);
Point2f findCircleCenter();
float findCircleRadius();
bool readCircle(cv::Mat frame);
void takeoff();
void initTime();
void updateTime();
void search();
void align();
void penetrate();
void findCircle();
void alignWithCircle(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint);
void alignWithQrCode(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint);
void GammaCorrection(Mat& src, Mat& dst, float fGamma);
int calcMarginOfError();
int marginOfError = 0;

int main(int argc, char *argv[])
{
	// ***************** INIT DRONE *****************
	if (!ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}
	// Battery
	std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;
	ardrone.setFlatTrim();
	
	currentState = TAKEOFF;
	std::cout << "STATE: takeoff" << std::endl;
	initTime();
	// MAIN LOOP
	while (1) {
		updateTime();
		circleFound = false;

		qrCodeTime += deltaTime;
		if (qrCodeFound > 2000) {
			qrCodeTime = 0;
			qrCodeFound = false;
		}
		// check for keys
		int key = cv::waitKey(20);
		if (key != -1) {
			if (key == 'q')  ardrone.move3D(0, 0, 1, 0);
			if (key == 'a')  ardrone.move3D(0, 0, -1, 0);
			if (key == CV_VK_UP) ardrone.move3D(1, 0, 0, 0);
			if (key == ' ') { if (ardrone.onGround()) { currentState = TAKEOFF;  ardrone.takeoff(); } else ardrone.landing(); }
			//if (key == 'c') ardrone.close(); exit(1);
		}
		
		// record a frame from camera
		frame = ardrone.getImage();
		Mat gammaCorrected1;
		Mat gammaCorrected2;
		GammaCorrection(frame, gammaCorrected1, 2.0f);
		GammaCorrection(frame, gammaCorrected2, 0.5f);
		if (readCircle(frame)) {
			circleFound = true;
		}
		else if (readCircle(gammaCorrected1)) {
			circleFound = true;
		}
		else if (readCircle(gammaCorrected2)) {
			circleFound = true;
		}
		//else {
		//	circleFound = false;
		//}
		qrCodeFound = readQrCode(frame);
		 

		// STATE CONTROL
		switch (currentState) {
			case TAKEOFF:	/*std::cout << "takeoff" << std::endl;*/		takeoff(); break;
			case SEARCH:	/*std::cout << "search" << std::endl;	*/		search(); break;
			case ALIGN:		/*std::cout << "align" << std::endl;*/			align(); break;
			case FINDCIRCLE:/*std::cout << "findcircle" << std::endl;*/		findCircle(); break;
			case PENETRATE: /*std::cout << "penetrate" << std::endl;*/		penetrate(); break;
		}
		cv::line(frame, cv::Point(0, imgCenterY), cv::Point(640, imgCenterY), Scalar(155, 40, 50));
		cv::line(frame, cv::Point(imgCenterX, 0), cv::Point(imgCenterX, 360), Scalar(155, 40, 50));
		cv::circle(frame, cv::Point(imgCenterX, imgCenterY), marginOfError, Scalar(155, 60, 80), 2);
		cv::imshow("camera", frame);
	}
}


int state = 0;
long bootupTime = 0;
long takeoffTime = 0;

void takeoff() {
	
	if (state == 0) {
		bootupTime += deltaTime;
		if (bootupTime > BOOTUP_TIME) {
			ardrone.takeoff();
			state = 1;
		}
	}
	// takeoff
	if (state == 1) {
		takeoffTime += deltaTime;
		//std::cout << "takeoff time: " << takeoffTime << std::endl;
		if (takeoffTime > TAKEOFF_TIME)
			state = 2;
	}
	if (state == 2) {
		if (ardrone.getAltitude() >= 1.2) {
			ardrone.move3D(0, 0, 0, 0);
			currentState = SEARCH;
			takeoffTime = 0;
			bootupTime = 0;
			state = 0;
			std::cout << "currentState: SEARCH" << std::endl;
		}
		ardrone.move3D(0, 0, 0.3, 0);
	}
}

long searchTimer = 0;
int searchState = 0;
long SEARCHTIME_FIRST = 3000;
long SEARCHTIME = 1000;
long searchWaitTime = 0;

void search() {

	if (circleCounter >= 4 || qrCodeFound) {
		currentState = ALIGN;
		std::cout << "currentState: ALIGN" << std::endl;

		searchState = 0;
		searchTimer = 0;
		searchWaitTime = 0;
		SEARCHTIME = 1000;
		return;
	}

	if (ardrone.getAltitude() > 2.0f || ardrone.getAltitude() < 1.0f) {
		double vz = 0;
		//std::cout << "Altitude: " << ardrone.getAltitude() << std::endl;
		if (ardrone.getAltitude() > 2.0f) {
			vz = -0.1;
			//std::cout << "To high" << std::endl;
		}
		else {
			vz = 0.1;
			//std::cout << "To low" << std::endl;
		}
		ardrone.move3D(0, 0, vz, 0);
		return;
	}

	searchTimer += deltaTime;
	searchWaitTime += deltaTime;

	if (searchWaitTime < 1500) {
		return;
	}
	double vr = 0.0;

	if ( searchState % 2 == 0) {
		vr = -0.8;
	}
	else {
		vr = 0.5;
	}


	if (searchState == 0 && searchTimer > SEARCHTIME/2) {
		searchState++;
		searchTimer = 0;
	}
	else if (searchTimer > SEARCHTIME) {
		searchState++;
		if (searchState % 2 == 0) {
			std::cout << "SEARCH RIGHT" << std::endl;
		}else
			std::cout << "SEARCH LEFT" << std::endl;
		searchTimer = 0;
		SEARCHTIME += SEARCHTIME;
	}


	ardrone.move3D(0, 0, 0, vr);

}


long circleCounterTime = 0;
long alignMoveTime = 0;

void align() {
	//std::cout << "align" << std::endl;
	alignMoveTime += deltaTime;
	double vx = 0.0;
	double vy = 0.0;
	double vz = 0.0;
	double vr = 0.0;
	if (circleFound)
		circleCounterTime = 0;
	else {
		circleCounterTime += deltaTime;
	}
	if (circleCounterTime > 1000) {
		circleCounter = 0;
		currentState = SEARCH;
		std::cout << "FROM ALIGN: NO CIRCLE to currentState: SEARCH" << std::endl;

	}


	// Control drone
	Point2f workingPoint;

	if (circleCounter >= 8)
	{
		workingPoint = findCircleCenter();
		cv::circle(frame, workingPoint, 10, Scalar(155, 40, 50), -1, 8);
		alignWithCircle(vx, vy, vz, vr, workingPoint);
	}
	else if (qrCodeFound) {
		workingPoint = QrCodeCenter;
		cv::circle(frame, workingPoint, 10, Scalar(155, 40, 50), -1, 8);
		alignWithQrCode(vx, vy, vz, vr, workingPoint);
	}

	if(alignMoveTime > 100){
		//std::cout << "moving drone: vx" << vx << ", vy: " << vy << ", vz: " << vz << ", vr: " << vr << std::endl;
		ardrone.move3D(vx, vy, vz, vr);
		alignMoveTime = 0;
	}
}

void alignWithCircle(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint) {
	//std::cout << "circle align: " << std::endl;
	calcMarginOfError();
	commands currentCommand = hoover;
	/*int startMargin = 40;
	float constant = 150;
	 = (constant / findCircleRadius())*startMargin;
	std::cout << "marginOfErrror: " << marginOfError << std::endl;

	
	*/
	
	
	
	bool centerV = false;
	bool centerH = false;
	long penetrateWaitTimer = 0;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
		//vy = 0.15;
	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
		//vy = -0.15;
	}
	else {
		centerH = true;
	}

	if (workingPoint.y > imgCenterY + marginOfError) {
		currentCommand = down;
	}
	else if (workingPoint.y < imgCenterY - marginOfError) {
		currentCommand = up;
	}
	else {
		centerV = true;
	}

	// Radius was 190: Casper
	if (centerH && centerV && findCircleRadius() > 170) {
		penetrateWaitTimer += deltaTime;
		
			currentState = PENETRATE;
			circleCounter = 0;
			std::cout << "currentState: PENETRATE" << std::endl;
			
			
		return;
	}
	else if (centerH && centerV) {
		currentCommand = forward;
		if (imgCenterX > workingPoint.x + marginOfError) {
			vr = 0.1;
		}
		else if (imgCenterX < workingPoint.x - marginOfError) {
			currentCommand = right;
			vr = -0.1;
		}
		if (workingPoint.y > imgCenterY + marginOfError) {
			vz = -0.1;
		}
		else if (workingPoint.y < imgCenterY - marginOfError) {
			vz = 0.1;
		}
	}
	penetrateWaitTimer = 0;
	switch (currentCommand) {
	case left: vr = 0.25;  break;
	case right: vr = -0.25; break;
	case up: vz = 0.25; break;
	case down: vz = -0.25; break;
	case hoover:  break;
	case forward: vx = 0.2; break;

	}
}

void alignWithQrCode(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint) {
	commands currentCommand = hoover;
	marginOfError = 100;


	bool centerV = false;
	bool centerH = false;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
		vr = 0.35;  
		//vy = 0.2;

	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
		vr = -0.35;
		//vy = -0.2;
	}
	else {
		centerH = true;
	}

	if (workingPoint.y > imgCenterY + marginOfError) {
		currentCommand = down;
		vz = -0.35;
	}
	else if (workingPoint.y < imgCenterY - marginOfError) {
		currentCommand = up;
		vz = 0.35;
	}
	else {
		centerV = true;
	}	
	if (centerH && centerV) {
		currentCommand = forward;
		if (QrCodeWith > 33) {
		currentState = FINDCIRCLE;
		std::cout << "FROM ALIGN_QR_CODE to currentState: FINDCIRCLE" << std::endl;

			return;
		}
		vx = 0.5;
	}

	//std::cout << "center-x: " << workingPoint.x << ", center-y: " << workingPoint.y << std::endl;
	//std::cout << "imgcenter-x: " << imgCenterX << ", imgcenter-y: " << imgCenterY << std::endl;
	//switch (currentCommand) {
	//case left: vr = 0.3; std::cout << "left" << std::endl;  break;
	//case right: vr = -0.3; std::cout << "right" << std::endl; break;
	//case up: vz = 0.3; std::cout << "up" << std::endl; break;
	//case down: vz = -0.3; std::cout << "down" << std::endl; break;
	//case hoover: std::cout << "Middle found Go Forward" << std::endl; break;
	//case forward: vx = 0.3; std::cout << "forward" << std::endl; break;

	//}
}

long findCircleTime = 0;
void findCircle() {
	std::cout << "altitude " << ardrone.getAltitude() <<std::endl;

	if (circleCounter >= 8) {
		currentState = ALIGN;
		std::cout << "FROM FINDICRLCE: CIRCLEFOUND to currentState: ALIGN" << std::endl;

		return;
	}
	std::cout << "Find circle" << std::endl;
	findCircleTime += deltaTime; 
	ardrone.move3D(0, 0, 0.2, 0);
		
	if (ardrone.getAltitude() > 2.0f) {
		currentState = SEARCH;
		std::cout << "FROM FINDCIRLCE HIGH ALTITUDE CHANGE to currentState: SEARCH" << std::endl;
	}
}

long penetrationTime = 0;
void penetrate() {

	ardrone.move3D(1, 0, 0, 0);
	
	penetrationTime += deltaTime;
	if (penetrationTime > 2000) {
		ardrone.move3D(0, 0, 0, 0);
		penetrationTime = 0;
		currentState = SEARCH;
		std::cout << "FROM PENETRATE to currentState: SEARCH" << std::endl;

	}
}
void initTime() {
	SYSTEMTIME st;
	GetSystemTime(&st);
	lastTime = st.wMilliseconds;
	deltaTime = 60;
}

void updateTime() {
	SYSTEMTIME st;
	GetSystemTime(&st);
	long currentTime = st.wMilliseconds;

	long newDelta = currentTime - lastTime;

	if (newDelta < 300 && newDelta > 0)
		deltaTime = newDelta;
	lastTime = currentTime;
}

bool readQrCode(cv::Mat frame) {
	cv::Mat grey;  // transformed image
	cv::cvtColor(frame, grey, CV_BGR2GRAY);   // convert image to grey
	int width = frame.cols;
	int height = frame.rows;
	
	uchar *raw = (uchar *)grey.data;
	zbar::Image image(width, height, "Y800", raw, width * height);
	scanner.scan(image);
	Point centerOfQR;
	centerOfQR.x = -1;
	centerOfQR.y = -1;
	// iterate over results
	for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
		std::vector<Point> vp;
		//std::cout << "decoded " << symbol->get_type_name() << " symbol " << symbol->get_data() << '"' << " " << std::endl;
		//std::cout << "QRCODE DATA: " << symbol->get_data() << std::endl;
		//std::cout << "QRCODE DATA at position 0: " << symbol->get_data().at(0) << std::endl;
		if (symbol->get_data().at(0) != 'P') {
			continue;
		}
		int n = symbol->get_location_size();
		for (int i = 0; i<n; i++) {
			vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));
		}

		// lets get the rect of the qr code,,, the angeled rect
		RotatedRect r = minAreaRect(vp);
		Point2f pts[4];

		// get middle of qr code
		r.points(pts);
		Point2f point;
		for (int i = 0; i < 4; i++) {
			point.x += pts[i].x;
			point.y += pts[i].y;
		}
		QrCode code;
		
		code.screenCoordinate.x = point.x / 4;
		code.screenCoordinate.y = point.y / 4;
		code.data = symbol->get_data();
		QrCodeWith = abs(pts[0].x - code.screenCoordinate.x);
		cv::circle(frame, code.screenCoordinate, 10, Scalar(255, 20, 20), -1, 8);
		for (int i = 0; i<4; i++) { cv::line(frame, pts[i], pts[(i + 1) % 4], Scalar(255, 20, 20), 3); }
		
		
		centerOfQR.x = code.screenCoordinate.x;
		centerOfQR.y = code.screenCoordinate.y;

		

	}
	QrCodeCenter.x = centerOfQR.x;
	QrCodeCenter.y = centerOfQR.y;
	return QrCodeCenter.x >= 0;
}

bool readCircle(cv::Mat cFrame) {
	cv::Mat imgGrayscale;
	cv::Mat imgBlurred;
	cv::Mat imgCanny;

	cv::Mat hsv_image;
	cv::cvtColor(cFrame, hsv_image, cv::COLOR_BGR2HSV);

	cv::Mat lower_red_hue_range;
	cv::Mat upper_red_hue_range;
	//cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	//cv::inRange(hsv_image, cv::Scalar(150, 100, 100), cv::Scalar(160, 255, 255), upper_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(150, 100, 100), cv::Scalar(160, 255, 255), upper_red_hue_range);



	// Combine the above two images
	cv::Mat red_hue_image;
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

	cv::GaussianBlur(red_hue_image,              // input image
		imgBlurred,                // output image
		cv::Size(5, 5),            // smoothing window width and height in pixels
		2.0);                      // sigma value, determines how much the image will be blurred

	Canny(imgBlurred,                       // input image
		imgCanny,                         // output image
		300,                               // low threshold
		450);                             // high threshold

	std::vector<cv::Vec3f> circle;
	HoughCircles(imgBlurred, circle, CV_HOUGH_GRADIENT, 2, 1000, 120, 50, 60, 500);
	//cv::namedWindow("blurred", CV_WINDOW_NORMAL);          // or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	//cv::namedWindow("higher", CV_WINDOW_NORMAL);
	//cv::namedWindow("lower", CV_WINDOW_NORMAL);
	//cv::namedWindow("canny", CV_WINDOW_NORMAL);
	bool circleFound = false;
	//std::cout << circle.size() << std::endl;
	for (size_t i = 0; i < circle.size(); i++) { // This is here if we need to find more than 1 circle
		Vec3f c = circle[i];

			cv::circle(frame,
				Point(c[0], c[1]),
				c[2],
				Scalar(0, 255, 0),
				3);

			cv::circle(frame,
				Point(c[0], c[1]),
				1,
				Scalar(0, 255, 0),
				3);

			Point center(cvRound(circle[i][0]), cvRound(circle[i][1]));
			if (circleCounter % 8 == 0) {
				circle1 = c;
			}
			if (circleCounter % 8 == 1) {
				circle2 = c;
			}
			if (circleCounter % 8 == 2) {
				circle3 = c;
			}
			if (circleCounter % 8 == 3) {
				circle4 = c;
			}
			if (circleCounter % 8 == 4) {
				circle5 = c;
			}
			if (circleCounter % 8 == 5) {
				circle6 = c;
			}
			if (circleCounter % 8 == 6) {
				circle7 = c;
			}
			if (circleCounter % 8 == 7) {
				circle8 = c;
			}
			circleCounter++;
			circleFound = true;

	}

	//cv::imshow("lower", lower_red_hue_range);
	//cv::imshow("higher", upper_red_hue_range);
	//cv::imshow("blurred", imgBlurred);
	//cv::imshow("canny", imgCanny);
	return circleFound;
}

Point2f findCircleCenter() {
	Vec3f circleCombined = (circle1 + circle2 + circle3 + circle4 + circle5 + circle6 + circle7 + circle8) / 8;
	Point center(cvRound(circleCombined[0]), cvRound(circleCombined[1]));

	return center;
}

float findCircleRadius() {

	Vec3f circleCombined = (circle1 + circle2 + circle3 + circle4 + circle5 + circle6 + circle7 + circle8) / 8;
	return circleCombined[2];
}

int calcMarginOfError() {
	marginOfError = -3.0f / 5.0f*findCircleRadius() + 160.0f;
	if (marginOfError > 100)
		marginOfError = 100;
	else if (marginOfError < 30)
		marginOfError = 30;
	return marginOfError;
}

void GammaCorrection(Mat& src, Mat& dst, float fGamma)
{

	unsigned char lut[256];

	for (int i = 0; i < 256; i++)

	{

		lut[i] = saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);

	}

	dst = src.clone();

	const int channels = dst.channels();

	switch (channels)

	{

	case 1:

	{

		MatIterator_<uchar> it, end;

		for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)

			*it = lut[(*it)];

		break;

	}

	case 3:

	{

		MatIterator_<Vec3b> it, end;

		for (it = dst.begin<Vec3b>(), end = dst.end<Vec3b>(); it != end; it++)

		{

			(*it)[0] = lut[((*it)[0])];

			(*it)[1] = lut[((*it)[1])];

			(*it)[2] = lut[((*it)[2])];

		}

		break;

	}

	}

}

  