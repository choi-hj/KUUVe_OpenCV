/* Test */

#pragma warning(disable: 4819)

#include "opencv2/opencv.hpp"
#include <iostream>
#include "ros/ros.h"

using namespace cv;
using namespace std;

/* 화면 사이즈 조절 상수 */
#define WIDTH 960/2
#define HEIGHT 540/2

/* 하프라인 함수 파라매토 */
#define THRESHOLD 50  

int main(int argc, char** argv)
{
	/* 만일 웹캠을 이용하고 싶다면 이 문장 사용
	VideoCapture cap(1);
	*/


	/*비디오를 저장하는 변수 cap 선언하고, 그 변수에 영상 넣기*/
	VideoCapture cap;
	cap.open("/home/choi/catkin_ws/src/lane_detection_pkg/src/cameraimage_color_camera3.mp4"); 

	if (!cap.isOpened())
	{
		cout << "영상이 열리지 않았습니다." << endl;
		return -1;
	}

	int fps = 1000;   // Å«°Ô ÁÁŽÙ
	double sum = 0;
	int temp = 0;
	double avg = 0;


	/* 차선인식을 위한 기본 화면들 정의 */
	Mat frame, gray, blured, Roi, edge;   
	
	/*직선들 각을 저장하는 변수*/
	double Left_angle;
	double Right_angle;

	/* 검출된 직선의 정보가 저장되는 변수... (x1,y1,x2,y2)순서로 저장된다. */
	vector<Vec4i> lines;  

	/* 영상 -> 수많은 사진들 -> 무한루프로 계속 처리 */
	for (;;)
	{
		int64 t1 = getTickCount(); // 초시계 스타트!!
		temp++;

		/* cap이라는 변수안에 영상이 들어있는데 이를 frame이라는 변수 안에 사진으로 넣기 */
		cap >> frame; 
		
		/* frame이라는 사진의 크기를 바꿔 frame이라는 변수에 다시 대입 */
		resize(frame, frame, Size(WIDTH, HEIGHT));
		
		/* 만일 frame이 빈 화면이라면... 오류 메시지 띄어준다. */
		if (frame.empty())
		{
			cout << "Empty frame!!!!!!" << endl;
			break;
		}
		
		/* 관심영역 자르기 */
		Roi = frame(Rect(0, HEIGHT / 2, WIDTH, HEIGHT / 2)); 
		/* 관심영역을 흑백으로 바꾸기 */
		cvtColor(Roi, gray, COLOR_BGR2GRAY);
		/* 흑백영상의 노이즈 제거하기 */ 
		bilateralFilter(gray, blured, -1, 50, 3); 
		/* 노이즈가 제거된 사진으로 edge 검출하기 */
		Canny(blured, edge, 100, 150); 
		/* edge영상을 토대로 직선검출하기 */
		HoughLinesP(edge, lines, 1, CV_PI / 180, THRESHOLD, 50, 200); 


		/* 누가 왼쪽 차선이고 누가 오른쪽 차선인지 판별을 위한 변수 */
		int left_x_center = WIDTH;
		int right_x_center = 0;
		
		/*오른쪽 차선, 왼쪽 차선 정보 담는 변수 선언 */
		Vec4i left_line(WIDTH, 0, WIDTH, HEIGHT);
		Vec4i right_line(0, 0, 0, HEIGHT);

		

		/* 왼쪽차선, 오른쪽선 구분하고 정보 저장하기.*/
		int v_size = lines.size();
		for (int i = 0; i < v_size ; i++)
		{
			/*직선의 기울기 계산*/
			int dx = lines.at(i)[2] - lines.at(i)[0];
			int dy = lines.at(i)[3] - lines.at(i)[1];

			float angle = atan2f(dy, dx) * 180 / CV_PI;
			
			/*만약 직선의 기울기가 10보다 작다면 그것은 차선이*/
			if (fabs(angle) <= 10) continue;

			if (angle < 0)
			{
				if (left_x_center > (lines.at(i)[2] + lines.at(i)[0]) / 2)
				{
					left_line = lines.at(i);
					right_x_center = (lines.at(i)[2] + lines.at(i)[0]) / 2;
				}
				Right_angle = abs(angle);
			}
			else
			{
				if (right_x_center < (lines.at(i)[2] + lines.at(i)[0]) / 2)
				{
					right_line = lines.at(i);
					right_x_center = (lines.at(i)[2] + lines.at(i)[0] / 2);
				}
				Right_angle = abs(angle);
			}
	
		}
		
		/* 원본 영상에 차선인식한 결과를 반영한다. */
		cv::line(frame, Point(left_line[0], left_line[1] + HEIGHT / 2), Point(left_line[2], left_line[3] + HEIGHT / 2), Scalar(0, 0, 255), 2);
		cv::line(frame, Point(right_line[0], right_line[1] + HEIGHT / 2), Point(right_line[2], right_line[3] + HEIGHT / 2), Scalar(0, 255, 0), 2);
		
		/* 평균 처리시간 계산, 화면에 띄어주기 */
		int64 t2 = getTickCount();

		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / temp;
		
		cout << "1. it took " << ms << "ms.    " << "2. avarage_time : " << avg <<"ms.    "<< "fps : " << 1000 / avg << endl;
		/* 결과화면 출력 */
		imshow("Roi", Roi);
		imshow("src", frame);
		imshow("dst", edge);
		
		if (waitKey(1000 / fps) >= 0) break; //
		

	}
			
	return 0;
}
