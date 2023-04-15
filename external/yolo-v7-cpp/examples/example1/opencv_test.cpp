#include <yolo_opencv.h>

int main()
{
    YOLO yolov7_kpt("/home/xiaoliu/下载/yolo-v7-cpp/config/yolov7-k6.yaml");
    yolov7_kpt.YOLO_INIT();

	cv::VideoCapture cap(0);
	cv::Mat img;

	while(1)
    {
        cap >> img;
        yolov7_kpt.YOLO_GET_KEY_POINTS(img);
	}
	return 0;
}
