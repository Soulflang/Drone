








#include "ardrone/ardrone.h"
#include <zbar.h>
#include <Windows.h>
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

// storing 3 circles for a average circle center and average
Vec3f circle1, circle2, circle3;
bool circleFound;
Point2f QrCodeCenter;
bool qrCodeFound;
int circleCounter = 0;

cv::Mat frame;

enum STATES { TAKEOFF, SEARCH, ALIGN, PENETRATE };
STATES currentState;

enum commands { left, right, down, up, NoCommand, hoover, forward };


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

	currentState = TAKEOFF;
	initTime();
	// MAIN LOOP
	while (1) {
		updateTime();
		std::cout << "whileloop: " << deltaTime << std::endl;
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
			case TAKEOFF:	takeoff(); break;
			case SEARCH:	search(); break;
			case ALIGN:		align(); break;
			case PENETRATE: penetrate(); break;
		}

	}

}


int state = 0;
long bootupTime = 0;
long takeoffTime = 0;

void takeoff() {
	
	std::cout << deltaTime << std::endl;
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
		return;
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
		ardrone.move3D(vx, vy, vz, vr);
		alignMoveTime = 0;
	}
}

void alignWithCircle(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint) {
	commands currentCommand = hoover;
	int marginOfError = 40;


	bool centerV = false;
	bool centerH = false;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
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

	std::cout << "center-x: " << workingPoint.x << ", center-y: " << workingPoint.y << std::endl;
	std::cout << "imgcenter-x: " << imgCenterX << ", imgcenter-y: " << imgCenterY << std::endl;
	switch (currentCommand) {
	case left: vr = 0.3; std::cout << "left" << std::endl;  break;
	case right: vr = -0.3; std::cout << "right" << std::endl; break;
	case up: vz = 0.3; std::cout << "up" << std::endl; break;
	case down: vz = -0.3; std::cout << "down" << std::endl; break;
	case hoover: std::cout << "Middle found Go Forward" << std::endl; break;
	case forward: vx = 0.3; std::cout << "forward" << std::endl; break;

	}
}

void alignWithQrCode(double &vx, double &vy, double &vz, double &vr, Point2f workingPoint) {
	commands currentCommand = hoover;
	int marginOfError = 40;


	bool centerV = false;
	bool centerH = false;
	if (imgCenterX > workingPoint.x + marginOfError) {
		currentCommand = left;
	}
	else if (imgCenterX < workingPoint.x - marginOfError) {
		currentCommand = right;
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
	if (centerH && centerV) {
		currentCommand = forward;
	}

	std::cout << "center-x: " << workingPoint.x << ", center-y: " << workingPoint.y << std::endl;
	std::cout << "imgcenter-x: " << imgCenterX << ", imgcenter-y: " << imgCenterY << std::endl;
	switch (currentCommand) {
	case left: vr = 0.3; std::cout << "left" << std::endl;  break;
	case right: vr = -0.3; std::cout << "right" << std::endl; break;
	case up: vz = 0.3; std::cout << "up" << std::endl; break;
	case down: vz = -0.3; std::cout << "down" << std::endl; break;
	case hoover: std::cout << "Middle found Go Forward" << std::endl; break;
	case forward: vx = 0.3; std::cout << "forward" << std::endl; break;

	}
}

long penetrationTime = 0;
void penetrate() {

	ardrone.move3D(0.5, 0, 0, 0);
	penetrationTime += deltaTime;
	if (penetrationTime > 1000)
		currentState = SEARCH;
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
		1.8);                      // sigma value, determines how much the image will be blurred

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

