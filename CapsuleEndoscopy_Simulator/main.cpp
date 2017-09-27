#define _USE_MATH_DEFINES
#include <cmath>  
#include <direct.h>
#include <iostream>
#include "opencv_set_3.2.0.h"

using namespace std;
using namespace cv;

const double scale = 20.0;    // pathImg 확대를 위한 scale factor
Point2d pOffset(0., 0.);
const char* window01 = "Capsule01";
const char* pathWindow = "Path";

string numbering(int n, int digit);
Point2d rotateVector(Point2d vec, double theta);

class capsule
{
public:
	Point2d pCur;    // Current position
	Point2d pPre;    // Previous position
	Point2d vDirec;    // Direction vector

	const double height = 1.0;    // TODO: 더 작게?
	const double FOV_angle = M_PI / 180 * 55;     // 55도
	const double FOV_length = height * 0.5;
	Point2d pFOV_left;
	Point2d pFOV_right;
	Point2d pFOV_Contact;

	int t, s, a;
	String path;    // Image file path
	Mat curImg;    // 현재 캡슐이 보고 있는 이미지: holw 찾는 알고리즘에 제공, 조향 및 병진 후 업데이트
	Mat blackImg;    // FOV에서 holw이 벗어났을 때 업데이트되는 이미지

	// 병진
	double step;
	// 조향
	const double theta = M_PI / 90;    // 2도
	const double cosTheta = cos(theta);
	const double sinTheta = sin(theta);

public:
	capsule(const string path) {
		this->pCur = Point2d(-20.0, -20.0);
		this->pPre = Point2d(-20.0, -20.0 - (height * 0.5));
		this->vDirec = Point2d(pCur - pPre) / norm(pCur - pPre);    // 방향벡터의 크기는 항상 1로 유지

		this->pFOV_left = Point2d(pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), FOV_angle));
		this->pFOV_right = Point2d(pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), -FOV_angle));
		this->pFOV_Contact = Point2d((pFOV_left + pFOV_right) / 2);

		this->t = 0, this->s = 0, this->a = 0;
		this->path = path;
		this->curImg = imread(path + "\\c_t" + numbering(t, 2) + "_s" + to_string(s) + "_a" + numbering(a, 2) + ".0.jpg");
		// TODO: 디스플레이를 위한 깊은 복사
		putText(curImg, "#c_t" + numbering(t, 2) + "_s" + to_string(s) + "_a" + numbering(a, 2) + ".0", Point(20, 40), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar::all(255));
		this->blackImg = Mat::zeros(curImg.size(), curImg.type());

		this->step = height * 0.25;
	}

	/** @brief FOV 직선의 방정식이 path와 만나는 좌표 계산
	* 단, FOV가 직선 path와 평행한 경우와 FOV가 곡선 path와의 교점이 하나도 없는 경우(D < 0)는 처리되어 있지 않음. (Hole을 완전히 잘못 찾은 경우)
	* @param inputNum 방정식에 대입할 x 또는 y
	* @param flag 0이면 x에, 1이면 y에 input을 대입. 2이면 곡선 path와의 교점을 구함.
	*/
	void calculateLineEq (double inputNum, int flag = 0) {
		if (flag == 0) {
			pFOV_Contact.x = inputNum;
			pFOV_Contact.y = (pFOV_right.y - pFOV_left.y) / (pFOV_right.x - pFOV_left.x) * (pFOV_Contact.x - pFOV_left.x) + pFOV_left.y;
		}
		else if (flag == 1) {
			pFOV_Contact.y = inputNum;
			pFOV_Contact.x = (pFOV_right.x - pFOV_left.x) / (pFOV_right.y - pFOV_left.y) * (pFOV_Contact.y - pFOV_left.y) + pFOV_left.x;
		}
		else {

		}
	}

	void rotate(int dgree) {
		double vDirecL2N = norm(vDirec);
		vDirec /= vDirecL2N;

		if (dgree == 2) {    // CCW
			vDirec.x = (vDirec.x * cosTheta) + (vDirec.y * -sinTheta);
			vDirec.y = (vDirec.x * sinTheta) + (vDirec.y * cosTheta);
			vDirec /= norm(vDirec);
			pCur = pPre + (vDirec * (height * 0.5));
		}
		else if (dgree == -2) {    // CW
			vDirec.x = (vDirec.x * cosTheta) + (vDirec.y * sinTheta);
			vDirec.y = (vDirec.x * -sinTheta) + (vDirec.y * cosTheta);
			vDirec /= norm(vDirec);
			pCur = pPre + (vDirec * (height * 0.5));
		}

		pFOV_left = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), FOV_angle);
		pFOV_right = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), -FOV_angle);
	}


	void move() {    // TODO: Step이 일정하지 않게 하는 parameter 추가
		double vDirecL2N = norm(vDirec);
		vDirec /= vDirecL2N;

		pPre.x += vDirec.x * step;
		pPre.y += vDirec.y * step;
		pCur.x += vDirec.x * step;
		pCur.y += vDirec.y * step;
		vDirec = pCur - pPre;
		vDirec /= norm(vDirec);

		pFOV_left = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), FOV_angle);
		pFOV_right = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), -FOV_angle);
	}

	// TODO: 구역 넘어갔을 때 뒤로 이동 (수정 중)
	void move(int flag) {
		double vDirecL2N = norm(vDirec);
		vDirec /= vDirecL2N;

		pPre.x += vDirec.x * step;
		pPre.y += vDirec.y * step;
		pCur.x += vDirec.x * step;
		pCur.y += vDirec.y * step;
		vDirec = pCur - pPre;
		vDirec /= norm(vDirec);

		pFOV_left = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), FOV_angle);
		pFOV_right = pCur + rotateVector(vDirec * FOV_length / cos(FOV_angle), -FOV_angle);
	}

	void updateA() {
		// TODO: Interpolation: round() 빼고 적용

		double ccw = norm(pFOV_Contact - pFOV_left);
		double cw = norm(pFOV_Contact - pFOV_right);
		double l = ccw + cw;
		ccw = 60 * ccw / l;
		cw = 60 * cw / l;
		if (round(ccw) == round(cw)) {
			s = 0;
			a = 0;
		}
		else if (ccw > cw) {
			s = 0;
			if ((round(ccw) - 30) > 29) {
				a = 30;
			}
			else {
				a = round(ccw) - 30;
			}
		}
		else {
			s = 1;
			if ((round(cw) - 30) > 29) {
				a = 30;
			}
			else {
				a = round(cw) - 30;
			}
		}
	}

	void updateImg() {
		// t가 0~27 안에서 반복되게 함.
		t++;
		if (t == 28) {
			t = 0;
		}

		// 업데이트
		if (a == -1) {
			curImg = blackImg.clone();
		}
		else {
			curImg = imread(path + "\\c_t" + numbering(t, 2) + "_s" + to_string(s) + "_a" + numbering(a, 2) + ".0.jpg");
			// TODO: 디스플레이를 위한 깊은 복사
			putText(curImg, "#c_t" + numbering(t, 2) + "_s" + to_string(s) + "_a" + numbering(a, 2) + ".0", Point(20, 40), CV_FONT_HERSHEY_SIMPLEX, 1, Scalar::all(255));
		}
	}

	void drawCapsule(Mat pathImg, Mat displayImg) {
		displayImg = pathImg.clone();
		line(displayImg, (pPre + pOffset) * scale, (pCur + pOffset) * scale, Scalar(0, 0, 0));    // 캡슐 본체
		circle(displayImg, (pCur + pOffset) * scale, 0, Scalar(0, 0, 255), 3);    // 캡슐 머리
		line(displayImg, (pFOV_left + pOffset) * scale, (pFOV_right + pOffset) * scale, Scalar(255, 0, 0));    // FOV
		circle(displayImg, (pFOV_Contact + pOffset) * scale, 0, Scalar(0, 0, 255), 3);    // FOV와 path가 만나는 교점
		flip(displayImg, displayImg, 0);
		imshow(pathWindow, displayImg);
		moveWindow(pathWindow, 50, 50);
	}
	
	~capsule() {

	}
};

void simulator(capsule* capsule, double xStart = -25., double xEnd = 25., double yStart = -25., double yEnd = 25., double deltaX = 0.25, double deltaY = 0.);
//void writeData();

// 내시경 카메라로 촬영 시 좌우에 생기는 검은 부분(좌 5px, 우 10px)을 제거
void resize();

int main(int argc, char* argv[]) {
	// 1) Set image path
	const int bufMax = 150;
	char cwd[bufMax] = "";
	getcwd(cwd, bufMax);
	const string path = string(cwd) + "\\Phantom";

	// 2) Initialize capsule endoscopy
	capsule capsule01 = capsule(path);

	// 3) Run simulator
	simulator(&capsule01);
	
	return 0;
}

/** @brief Generate a serial number with zeros and convert it to string
* From two digits to five digits: 00~99, 000~999, 0000 ~ 9999 or 00000 ~ 99999
* @author Taejin Kim
* @date 7 Apr 2017
* @param n An original number
* @param digit Digit of serial number to be returned
* @return A serial number with zeros
*/
string numbering(int n, int digit) {
	switch (digit) {
	case 2:
		if (n >= 0 && n < 10) {
			return "0" + to_string(n);
		}
		else if (n >= 10 && n < 100) {
			return to_string(n);
		}
		else {
			return "Error generating serial number";
		}

		break;
	case 3:
		if (n >= 0 && n < 10) {
			return "00" + to_string(n);
		}
		else if (n >= 10 && n < 100) {
			return "0" + to_string(n);
		}
		else if (n >= 100 && n < 1000) {
			return to_string(n);
		}
		else {
			return "Error generating serial number";
		}

		break;
	case 4:
		if (n >= 0 && n < 10) {
			return "000" + to_string(n);
		}
		else if (n >= 10 && n < 100) {
			return "00" + to_string(n);
		}
		else if (n >= 100 && n < 1000) {
			return "0" + to_string(n);
		}
		else if (n >= 1000 && n < 10000) {
			return to_string(n);
		}
		else {
			return "Error generating serial number";
		}

		break;
	case 5:
		if (n >= 0 && n < 10) {
			return "0000" + to_string(n);
		}
		else if (n >= 10 && n < 100) {
			return "000" + to_string(n);
		}
		else if (n >= 100 && n < 1000) {
			return "00" + to_string(n);
		}
		else if (n >= 1000 && n < 10000) {
			return "0" + to_string(n);
		}
		else if (n >= 10000 && n < 100000) {
			return to_string(n);
		}
		else {
			return "Error generating serial number";
		}

		break;
	default:
		break;
	}

	return "Error generating serial number";
}

Point2d rotateVector(Point2d vec, double theta) {
	Point2d vRotated(0.0, 0.0);
	const double cosTheta = cos(theta);
	const double sinTheta = sin(theta);

	vRotated.x = (vec.x * cosTheta) + (vec.y * -sinTheta);
	vRotated.y = (vec.x * sinTheta) + (vec.y * cosTheta);
	
	return vRotated;
}

void simulator(capsule* capsule, double xStart, double xEnd, double yStart, double yEnd, double deltaX, double deltaY) {
	if (deltaY == 0) {
		deltaY = deltaX;
	}

	double pathWidth_half = capsule->height; 
	Point2d pEnd(0., 0.);

	Mat displayImg;

	// Show a correct path and a capsule endoscopy
	Point2d pDraw(0.0, 0.0);
	pOffset = Point2d(-xStart, -yStart);    // (0, 0)을 pathImg의 가운데로 맞춰주기 위한 offset
	Size2d pathImgSize(xEnd - xStart + 1, yEnd - yStart + 1);    // 51 x 51
	Mat pathImg = Mat(pathImgSize * scale, CV_64FC3, Scalar(255, 255, 255));
	line(pathImg, Point2d(0, pOffset.y) * scale, Point2d(pathImg.cols, pOffset.y) * scale, Scalar(0, 0, 0));    // x-axis
	line(pathImg, Point2d(pOffset.x, 0) * scale, Point2d(pOffset.x, pathImg.rows) * scale, Scalar(0, 0, 0));    // y-axis
	// 구간 1
	line(pathImg, (Point2d(-20., -25.) + pOffset) * scale, (Point2d(-20., -10.) + pOffset) * scale, Scalar(0, 255, 0));
	line(pathImg, (Point2d(-20. - pathWidth_half, -25.) + pOffset) * scale, (Point2d(-20. - pathWidth_half, -10.) + pOffset) * scale, Scalar(51, 51, 0));
	line(pathImg, (Point2d(-20. + pathWidth_half, -25.) + pOffset) * scale, (Point2d(-20. + pathWidth_half, -10.) + pOffset) * scale, Scalar(51, 51, 0));
	// 구간 2
	circle(pathImg, (Point2d(-10, -10) + pOffset) * scale, 10 * scale, Scalar(0, 255, 0));
	circle(pathImg, (Point2d(-10, -10) + pOffset) * scale, (10 - pathWidth_half) * scale, Scalar(51, 51, 0));
	circle(pathImg, (Point2d(-10, -10) + pOffset) * scale, (10 + pathWidth_half) * scale, Scalar(51, 51, 0));
	// 구간 3
	line(pathImg, (Point2d(-10, 0) + pOffset) * scale, (Point2d(0, 0) + pOffset) * scale, Scalar(0, 255, 0));
	line(pathImg, (Point2d(-10, 0 - pathWidth_half) + pOffset) * scale, (Point2d(0, 0 - pathWidth_half) + pOffset) * scale, Scalar(51, 51, 0));
	line(pathImg, (Point2d(-10, 0 + pathWidth_half) + pOffset) * scale, (Point2d(0, 0 + pathWidth_half) + pOffset) * scale, Scalar(51, 51, 0));

	//capsule->drawCapsule(displayImg);

	// TODO: 병진 거리와 회전각
	int step = 1;    // 캡슐 내부에 고정해놓음. (현재 이 값은 필요 없는 값)
	int rotation = 0;    // Radian으로 변경?

	while (true) {
		// 목표지점 근처에 도달하면 종료
		if (norm(capsule->pCur - pEnd) < 1) {
			break;
		}

		// 현재 캡슐이 보고 있는 화면을 가져오고 path 위에 캡슐의 위치와 FOV를 그림.
		imshow(window01, capsule->curImg);
		moveWindow(window01, 1050, 50);
		capsule->drawCapsule(pathImg, displayImg);

		/*
		for (double y = yStart; y <= yEnd; y += deltaY) {
			for (double x = xStart; x <= xEnd; x += deltaX) {
			}
		}
		*/

		// TODO: Hole 찾는 알고리즘에 의해 step과 rotation이 업데이트 되고 그 값에 따라 병진 및 회전


		// 구간별로 캡슐의 위치와 방향, FOV 분석
		// 구간 1: x = -20
		if (capsule->pCur.x > (-20 - pathWidth_half) && capsule->pCur.x < (-20 + pathWidth_half)
			&& capsule->pCur.y > -25 && capsule->pCur.y < -10) {
			// [Debugging] Key 입력으로 캡슐 조종
			while (true) {
				int key = waitKey(0);
				if (key == 'a') {    /// 'a' key: CCW
					capsule->rotate(2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 'd') {    /// 'd' key: CW
					capsule->rotate(-2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 32) {    /// 'space bar' key: 병진
					capsule->move();
					capsule->drawCapsule(pathImg, displayImg);
					break;
				}
			}

			// Hole이 FOV를 벗어났는 지(pFOV_Contact가 FOV의 연장된 직선이 path와 만나는 교점일 경우)인지 확인
			capsule->calculateLineEq(-20., 0);
			if (capsule->pFOV_Contact.x > capsule->pFOV_left.x && capsule->pFOV_Contact.x < capsule->pFOV_right.x) {    // FOV에 hole이 있는 경우
				capsule->updateA();
			}
			else {    // FOV에서 hole이 벗어난 경우
				capsule->a = -1;    // 까만 화면
			}

		}
		// 구간 2: (x + 10)^2 + (y + 10)^2 = 10^2
		else if (capsule->pCur.x <= -10 && capsule->pCur.y >= -10
			&& pow(capsule->pCur.x - (-10), 2) + pow(capsule->pCur.y - (-10), 2) > pow(10 - pathWidth_half, 2)
			&& pow(capsule->pCur.x - (-10), 2) + pow(capsule->pCur.y - (-10), 2) < pow(10 + pathWidth_half, 2)) {
			// [Debugging] Key 입력으로 캡슐 조종
			while (true) {
				int key = waitKey(0);
				if (key == 'a') {    /// 'a' key: CCW
					capsule->rotate(2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 'd') {    /// 'd' key: CW
					capsule->rotate(-2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 32) {    /// 'space bar' key: 병진
					capsule->move();
					capsule->drawCapsule(pathImg, displayImg);
					break;
				}
			}
			
			// TODO: 곡선(CW)의 FOV
			//capsule->calculateLineEq(, 2);
		}
		// 구간 3: y = 0
		else if (capsule->pCur.x > -10 && capsule->pCur.x < 5
			&& capsule->pCur.y >(0 - pathWidth_half) && capsule->pCur.y < (0 + pathWidth_half)) {
			// [Debugging] Key 입력으로 캡슐 조종
			while (true) {
				int key = waitKey(0);
				if (key == 'a') {    /// 'a' key: CCW
					capsule->rotate(2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 'd') {    /// 'd' key: CW
					capsule->rotate(-2);
					capsule->drawCapsule(pathImg, displayImg);
				}
				else if (key == 32) {    /// 'space bar' key: 병진
					capsule->move();
					capsule->drawCapsule(pathImg, displayImg);
					break;
				}
			}
				
			capsule->calculateLineEq(0., 1);
			if (capsule->pFOV_Contact.y > capsule->pFOV_right.y && capsule->pFOV_Contact.y < capsule->pFOV_left.y) {    // FOV에 hole이 있는 경우
				capsule->updateA();
			}
			else {    // FOV에서 hole이 벗어난 경우
				capsule->a = -1;    // 까만 화면
			}
		}
		else {
			// TODO: 캡슐이 구역을 넘어갔을 때 (애초에 못 넘어가게? 회전 및 이동 하는 부분에서 검사해서 구역 넘어가면 되돌리기)
			capsule->move(-1);    // 다시 구역 안으로 들어올 때까지 뒤로 이동
		}

		// 캡슐 화면 업데이트
		capsule->updateImg();
	}
}

void resize() {
	int t = 0;    // Time: 0~27 (iteration)
	int s = 0;    // Sign: 0 (counterclockwise) or 1 (clockwise)
	int a = 0;    // Angle: -30 (counterclockwise) ~ 0 (center) ~ 30 (clockwise)

	const int bufMax = 150;
	char cwd[bufMax] = "";
	getcwd(cwd, bufMax);
	string imgName;
	string imgPath;

	cout << "Start resize" << endl;
	for (t = 0; t < 28; t++) {
		cout << t << " / 28" << endl;
		for (s = 0; s < 2; s++) {
			for (a = 0; a < 31; a++) {
				if (s == 1 && a == 0) { continue; }

				imgName = "c_t" + numbering(t, 2) + "_s" + to_string(s) + "_a" + numbering(a, 2) + ".0.jpg";
				imgPath = string(cwd) + "\\Phantom" + "\\" + imgName;

				Mat img = imread(imgPath);
				//imshow("Debugging", img);
				//waitKey(0);
				Mat crop = img(Rect(5, 0, img.cols - 15, img.rows));    // Image size: 640 x 480 -> 625 x 480

				imgPath = string(cwd) + "\\Phantom" + "\\Resize" + "\\" + imgName;
				imwrite(imgPath, crop);
			}
		}
	}
}