#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

#ifndef YOLOV5
#define YOLOV5 false  //true:Yolov5, false:yolov7
#endif

#ifndef YOLO_P6
#define YOLO_P6 false //是否使用P6模型
#endif

/**
 * @brief 对pytorch生成的onnx的使用
 * @note 声明类时，YOLO(false)即可关闭cuda,默认使用YOLO则会打开cuda！
 */
class YOLO {
public:
    /**
     * @brief 构造函数
     * @param filepath yaml配置文件路径
     * @param use_cuda 是否使用CUDA
     */
    YOLO(std::string filepath, bool use_cuda = true);

    YOLO();
    /**
     * @brief 析构函数
     */
    ~YOLO(){};
private:
    static const int class_num = 1;  //种类数
    static const int key_points_num = 6; //关键点数
    /**
     * @param class_name 目标检测的种类
     * @param key_points_num 关键点数目
     */
//    std::vector<std::string> class_name = { "None", "Red", "Blue"};
    std::vector<std::string> class_name = { "cylinder"}; //种类名
//    std::vector<std::string> class_name = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
//                                           "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
//                                           "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
//                                           "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
//                                           "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
//                                           "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
//                                           "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
//                                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
//                                           "hair drier", "toothbrush" };
//    std::vector<std::string> class_name = { "person"};
    std::vector<std::string> key_points_name = { "l1", "r1", "l2", "r2", "l3", "r3"};   //关键点名
    struct key_points
    {
        cv::Point kpt[key_points_num];
    };

public:
    struct yolo_output
    {
        int id;
        float confidence;
        cv::Rect bbox;
        key_points kpts;

        float height =0;
        float distance = 0;
        float x =0;
        float y =0;
        float horizontal_angle =0;
        cv::Mat image;

        int index = 0;

        cv::Mat roi;

        char tag = 'N';

    };

    std::vector<yolo_output> results;
private:
    /* onnx 输入图片的高度和宽度 */
    int net_width = 960;
    int net_height = 960;
    bool if_use_cuda = true;
    cv::dnn::Net yolo_net;
    std::string model_path;
    std::vector<cv::Scalar> color;


    /**
     * @brief 激活函数
     * @note 用户忽略此函数
     * @param x 输入值
     * @return sigmoid(x)
     */
    float sigmoid_x(float x)
    {
        return static_cast<float>(1.f / (1.f + exp(-x)));
    }
    /* yolov7-P5 anchors */
    const float net_anchors[3][6] =
    {
            {12, 16, 19, 36, 40, 28},
            {36, 75, 76, 55, 72, 146},
            {142, 110, 192, 243, 459, 401}
    };

    const float kpt_net_anchors[4][6] =
    {
            {19,   27,  44, 40, 38, 94},
            {96,  68,  86, 152,180,137},
            {140, 301, 303,264,238,542},
            {436, 615, 739,380,925,792}
    };

    /* stride size */
    const int kpt_stride_size = 4;
    const int stride_size = 3;
    const float net_stride[4] = {8, 16.0,32,64};
    float bbox_threshold = 0.25;
    float class_threshold = 0.25;
    float nms_threshold = 0.45;
    float kpt_threshold = 0.5;
    float nms_score_threshold = bbox_threshold * class_threshold;
public:
    /**
     * @brief 初始化
     */
    void YOLO_INIT();

    /**
     * @brief 读取onnx模型
     */
	void YOLO_READ_MODEL();

    void YOLO_READ_MODEL(std::string modle);
    /**
     * @brief 物体检测
     * @param src 待检测图片
     */
	void YOLO_OBJECT_DETECT(cv::Mat &src);

    /**
     * @brief 物体检测 + 关键点检测
     * @param src 待检测图片
     */
    void YOLO_KEY_POINTS_DETECT(cv::Mat &src);

    /**
     * @brief 物体检测结果可视化
     * @param src 待可视化图片
     */
	void YOLO_OBJECT_DRAW_PRED(cv::Mat &src);

    /**
     * @brief 物体检测 + 关键点检测可视化
     * @param src 待可视化图片
     */
    void YOLO_KEY_POINTS_DRAW_PRED(cv::Mat &src);

    /**
     * @brief 物体检测的接口
     * @param src
     */
    void YOLO_GET_OBJECT_DETECTION(cv::Mat &src);

    /**
     * @brief 物体检测 + 关键点检测的接口
     * @param src
     */
    void YOLO_GET_KEY_POINTS(cv::Mat &src);
};
