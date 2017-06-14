








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
const long TAKEOFF_TIME = 4000;

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
Vec3f circle1, circle2, circle3;
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
	initTime();
	// MAIN LOOP
	while (1) {
		
		updateTime();

		circleFound = false;

		qrCodeTime += deltaTime;
		if (qrCodeFound > 1000) {
			qrCodeTime = 0;
			qrCodeFound = false;
		}
		// check for keys
		int key = cv::waitKey(33);
		if (key != -1) {
			if (key == 'q')  ardrone.move3D(0, 0, 1, 0);
			if (key == 'a')  ardrone.move3D(0, 0, -1, 0);
			if (key == CV_VK_UP) ardrone.move3D(1, 0, 0, 0);
			if (key == ' ') if (ardrone.onGround()) ardrone.takeoff(); else ardrone.landing();
		}

		// record a frame from camera
		frame = ardrone.getImage();
		circleFound = readCircle(frame);
		qrCodeFound = readQrCode(frame);
		

		// STATE CONTROL
		switch (currentState) {
			case TAKEOFF:	std::cout << "takeoff" << std::endl;		takeoff(); break;
			case SEARCH:	std::cout << "search" << std::endl;			search(); break;
			case ALIGN:		std::cout << "align" << std::endl;			align(); break;
			case FINDCIRCLE:std::cout << "findcircle" << std::endl;		findCircle(); break;
			case PENETRATE: std::cout << "penetrate" << std::endl;		penetrate(); break;
		}
		cv::imshow("camera", frame);
	}


}



int state = 0;
long bootupTime = 0;
long takeoffTime = 0;

void takeoff() {
	std::cout << "takeoff" << std::endl;
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
		std::cout << "takeoff time: " << takeoffTime << std::endl;
		if (takeoffTime > TAKEOFF_TIME)
			currentState = SEARCH;
	}
	
}

void search() {
	if(circleCounter >= 3 || qrCodeFound)
		currentState = ALIGN;
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
	}


	// Control drone
	Point2f workingPoint;

	if (circleCounter >= 3)
	{
		workingPoint = findCircleCenter();
		circle(frame, workingPoint, 10, Scalar(155, 40, 50), -1, 8);
		alignWithCircle(vx, vy, vz, vr, workingPoint);
	}
	else if (qrCodeFound) {
		workingPoint = QrCodeCenter;
		circle(frame, workingPoint, 10, Scalar(155, 40, 50), -1, 8);
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
	int marginOfError = 0;
	commands currentCommand = hoover;
	/*int startMargin = 40;
	float constant = 150;
	 = (constant / findCircleRadius())*startMargin;
	std::cout << "marginOfErrror: " << marginOfError << std::endl;

	
	*/
	if (findCircleRadius() < 100) marginOfError = 70;
	else marginOfError = 30;
	bool centerV = false;
	bool centerH = false;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
		vy = 0.2;
	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
		vy = -0.2;
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
	if (centerH && centerV && findCircleRadius() > 200) { 
		currentState = PENETRATE;
		return;
	}
	else if (centerH && centerV) {
		currentCommand = forward;
	}

	switch (currentCommand) {
	case left: vr = 0.3;  break;
	case right: vr = -0.3; break;
	case up: vz = 0.3; break;
	case down: vz = -0.3; break;
	case hoover:  break;
	case forward: vx = 0.3; break;

	}
}

void alignWithQrCode(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint) {
	commands currentCommand = hoover;
	int marginOfError = 40;


	bool centerV = false;
	bool centerH = false;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
		vr = 0.3;  
		vy = 0.3;

	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
		vr = -0.3;
		vy = -0.3;
	}
	else {
		centerH = true;
	}

	if (workingPoint.y > imgCenterY + marginOfError) {
		currentCommand = down;
		vz = -0.3;
	}
	else if (workingPoint.y < imgCenterY - marginOfError) {
		currentCommand = up;
		vz = 0.3;
	}
	else {
		centerV = true;
	}	
	if (centerH && centerV) {
		std::cout << "forward and QRWidth is: " << QrCodeWith << std::endl;
		currentCommand = forward;
		if (QrCodeWith > 33) {
		currentState = FINDCIRCLE;
			return;
		}
		vx = 0.8;
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
	std::cout << "circleCounter " << circleCounter <<std::endl;

	if (circleCounter >= 3) {
		currentState = SEARCH;
		return;
	}
	std::cout << "Find circle" << std::endl;
	findCircleTime += deltaTime; 
	ardrone.move3D(0, 0, 0.5, 0);
		
	if (findCircleTime > 7000)
		currentState = SEARCH;
}

long penetrationTime = 0;
void penetrate() {

	ardrone.move3D(0.8, 0, 0, 0);
	penetrationTime += deltaTime;
	if (penetrationTime > 2000) {
		penetrationTime = 0;
		currentState = SEARCH;
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
		circle(frame, code.screenCoordinate, 10, Scalar(255, 20, 20), -1, 8);
		for (int i = 0; i<4; i++) { line(frame, pts[i], pts[(i + 1) % 4], Scalar(255, 20, 20), 3); }
		
		
		centerOfQR.x = code.screenCoordinate.x;
		centerOfQR.y = code.screenCoordinate.y;

		
	}
	QrCodeCenter.x = centerOfQR.x;
	QrCodeCenter.y = centerOfQR.y;
	return QrCodeCenter.x >= 0;
}

bool readCircle(cv::Mat frame) {
	cv::Mat imgGrayscale;
	cv::Mat imgBlurred;
	cv::Mat imgCanny;

	cv::Mat hsv_image;
	cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

	cv::Mat lower_red_hue_range;
	cv::Mat upper_red_hue_range;
	cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
	cv::inRange(hsv_image, cv::Scalar(150, 100, 100), cv::Scalar(160, 255, 255), upper_red_hue_range);

	// Combine the above two images
	cv::Mat red_hue_image;
	cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

	cv::GaussianBlur(red_hue_image,              // input image
		imgBlurred,                // output image
		cv::Size(5, 5),            // smoothing window width and height in pixels
		2.5);                      // sigma value, determines how much the image will be blurred

	Canny(imgBlurred,                       // input image
		imgCanny,                         // output image
		300,                               // low threshold
		450);                             // high threshold

	std::vector<cv::Vec3f> circle;
	HoughCircles(imgBlurred, circle, CV_HOUGH_GRADIENT, 2, 1000, 120, 50, 30, 500);
	cv::namedWindow("blurred", CV_WINDOW_NORMAL);          // or CV_WINDOW_AUTOSIZE for a fixed size window matching the resolution of the image
	cv::namedWindow("higher", CV_WINDOW_NORMAL);
	cv::namedWindow("lower", CV_WINDOW_NORMAL);
	cv::namedWindow("canny", CV_WINDOW_NORMAL);
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
			if (circleCounter % 3 == 0) {
				circle1 = c;
			}
			if (circleCounter % 3 == 1) {
				circle2 = c;
			}
			if (circleCounter % 3 == 2) {
				circle3 = c;
			}
			circleCounter++;
			circleFound = true;

	}

	cv::imshow("lower", lower_red_hue_range);
	cv::imshow("higher", upper_red_hue_range);
	cv::imshow("blurred", imgBlurred);
	cv::imshow("canny", imgCanny);
	return circleFound;
}

Point2f findCircleCenter() {
	Vec3f circleCombined = (circle1 + circle2 + circle3)/3;	
	Point center(cvRound(circleCombined[0]), cvRound(circleCombined[1]));

	return center;
}

float findCircleRadius() {

	Vec3f circleCombined = (circle1 + circle2 + circle3) / 3;
	return circleCombined[2];
}

//bool isItFlySafe(int screenMargin) {
//
//	float relativeX = (imgCenterX)*(screenMargin*(imgheight / imgWitdh)); //Adjusting for screen ratio
//	float relativeY = (imgCenterY*screenMargin);
//
//	bool isObjectCloseEnough = false;
//
//	Point2f circleCenter = findCircleCenter();
//	float circleRadius = findCircleRadius();
//
//	if (QrCodeCenter.x + relativeX < QRcodeRadius + QrCodeCenter.x && QrCodeCenter.x - relativeX > QRcodeRadius - QrCodeCenter.x) {
//		if (QrCodeCenter.y + relativeY < QRcodeRadius + QrCodeCenter.y && QrCodeCenter.y - relativeY > QRcodeRadius - QrCodeCenter.y) {
//			isObjectCloseEnough = true;
//		}
//		else {
//			isObjectCloseEnough = false;
//		}
//
//	}
//	else if (circleCenter.x + relativeX < circleRadius + circleCenter.x && circleCenter.x - relativeX > circleRadius - circleCenter.x) {
//		if (circleCenter.y + relativeY < circleRadius + circleCenter.y && circleCenter.y - relativeY > circleRadius - circleCenter.y) {
//			isObjectCloseEnough = true;
//		}
//		else {
//			isObjectCloseEnough = false;
//		}
//	}
//
//	return false;
//}
