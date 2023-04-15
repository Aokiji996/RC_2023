#include "yolo_opencv.h"

YOLO::YOLO(){

}

/**
 * @brief 构造函数
 * @param filepath yaml配置文件路径
 * @param use_cuda 是否使用CUDA
 */
YOLO::YOLO(std::string filepath, bool use_cuda)
{
    cv::FileStorage file(filepath, cv::FileStorage::READ);
    if(file.isOpened())
    {
        file["model_path"] >> this->model_path;
        file["class_names"] >> this->class_name;
        file["new_width"] >> this->net_width;
        file["new_height"] >> this->net_height;
        file["bbox_threshold"] >> this->bbox_threshold;
        file["class_threshold"] >> this->class_threshold;
        file["nms_threshold"] >> this->nms_threshold;
        file["kpt_threshold"] >> this->kpt_threshold;
    }
    file.release();
    this->if_use_cuda = use_cuda;
    if(this->if_use_cuda)
    {
        printf("Use CUDA to accelerate calculation!\n");
    }
    else
    {
        printf("CUDA is prohibited!\n");
    }
}

/**
 * @brief 读取onnx模型
 */
void YOLO::YOLO_READ_MODEL()
{
    try
    {
        this->yolo_net = cv::dnn::readNet(this->model_path);
    }
    catch(const std::exception&)
    {
        std::cout << "Error in read model!Check if use YOLO::your_object_name.YOLO_SET_MODEL_PATH(your model path) to set model path correctly!" <<std::endl;
        exit(0);
    }
    if(this->if_use_cuda)
    {
        this->yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        this->yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        this->yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        this->yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    printf("Read model success!\n");
}



/**
 * @brief 读取onnx模型
 */
void YOLO::YOLO_READ_MODEL(std::string modle)
{
    try
    {
        this->yolo_net = cv::dnn::readNet(modle);
    }
    catch(const std::exception&)
    {
        std::cout << "Error in read model!Check if use YOLO::your_object_name.YOLO_SET_MODEL_PATH(your model path) to set model path correctly!" <<std::endl;
        exit(0);
    }
    if(this->if_use_cuda)
    {
        this->yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        this->yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        this->yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        this->yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    printf("Read model success!\n");
}





/**
 * @brief 初始化
 */
void YOLO::YOLO_INIT()
{
    srand(time(0));
    for(size_t i = 0; i < 120;i++)
    {
        int b = rand() % 256;
		int g = rand() % 256;
		int r = rand() % 256;
        this->color.emplace_back(cv::Scalar(b, g, r));
    }
    this->YOLO_READ_MODEL();
}

/**
 * @brief 物体检测
 * @param src 待检测图片
 */
void YOLO::YOLO_OBJECT_DETECT(cv::Mat &src)
{
	cv::Mat blob;
	int col = src.cols;
	int row = src.rows;
	int maxLen = MAX(col, row);
	cv::Mat net_input_img = src.clone();
	if (maxLen > 1.2 * col || maxLen > 1.2 * row)
    {
		cv::Mat resize_img = cv::Mat::zeros(maxLen, maxLen, CV_8UC3);
		src.copyTo(resize_img(cv::Rect(0, 0, col, row)));
        net_input_img = resize_img;
	}
	std::vector<cv::Ptr<cv::dnn::Layer>> layer;
	std::vector<std::string> layer_names ;
	layer_names = this->yolo_net.getLayerNames();
	cv::dnn::blobFromImage(net_input_img, blob, 1 / 255.0, cv::Size(this->net_width, this->net_height), cv::Scalar(0, 0, 0), true, false);
    /* 如果在其他设置没有问题的情况下但是结果偏差很大，可以尝试下用下面两句语句 */
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
    this->yolo_net.setInput(blob);
	std::vector<cv::Mat> net_output_img;

	this->yolo_net.forward(net_output_img, this->yolo_net.getUnconnectedOutLayersNames());
#if CV_VERSION_MAJOR==4&&CV_VERSION_MINOR==6
    /* OpenCV4.6.0 */
	std::sort(net_output_img.begin(), net_output_img.end(), [](cv::Mat &A, cv::Mat &B) {return A.size[2] > B.size[2];});
#endif
    /* 结果id数组 */
    std::vector<int> class_ids;
    /* 结果每个id对应置信度数组 */
    std::vector<float> confidences;
    /* 每个id矩形框 */
    std::vector<cv::Rect> bboxes;

	float ratio_h = (float)net_input_img.rows / this->net_height;
	float ratio_w = (float)net_input_img.cols / this->net_width;
    /* 输出的网络宽度是类别数+5 */
	int net_channels = this->class_name.size() + 5;
    for(size_t stride = 0; stride < this->stride_size; stride++)
    {
        float *ptr_data = (float*)net_output_img[stride].data;
        int grid_x = (int)(this->net_width / this->net_stride[stride]);
        int grid_y = (int)(this->net_height / this->net_stride[stride]);
        for(int anchor = 0; anchor < 3; anchor++)
        {
            const float anchor_w = this->net_anchors[stride][anchor * 2];
            const float anchor_h = this->net_anchors[stride][anchor * 2 + 1];
            for(size_t i = 0; i < grid_y; i++)
            {
                for(size_t j = 0; j < grid_x; j++)
                {
                    /* 获取每一行的box框中含有某个物体的概率 */
                    float bbox_score = sigmoid_x(ptr_data[4]);
                    if(bbox_score >= this->bbox_threshold)
                    {
                        cv::Mat scores(1, class_name.size(), CV_32FC1, ptr_data+5);
                        cv::Point class_id_point;
                        double max_class_score;
                        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
                        max_class_score = sigmoid_x(max_class_score);
                        if(max_class_score >= this->class_threshold)
                        {
                            float x = (sigmoid_x(ptr_data[0]) * 2.f - 0.5f + j) * this->net_stride[stride];
                            float y = (sigmoid_x(ptr_data[1]) * 2.f - 0.5f + i) * this->net_stride[stride];
                            float w = powf(sigmoid_x(ptr_data[2]) * 2.f, 2.f) * anchor_w;
                            float h = powf(sigmoid_x(ptr_data[3]) * 2.f, 2.f) * anchor_h;
                            int left = (int)(x - 0.5 * w) * ratio_w + 0.5;
                            int top  = (int)(y - 0.5 * h) * ratio_h + 0.5;
                            class_ids.push_back(class_id_point.x);
                            confidences.push_back(max_class_score*bbox_score);
                            bboxes.push_back(cv::Rect(left, top, int(w*ratio_w), int(h*ratio_h)));
                        }
                    }
                    /* 下一行 */
                    ptr_data += net_channels;
                }
            }
        }
    }

	/* 执行非最大抑制以消除具有较低置信度的冗余重叠框（NMS）*/
	std::vector<int> nms_result;
	cv::dnn::NMSBoxes(bboxes, confidences, this->nms_score_threshold, this->nms_threshold, nms_result);
    for(size_t i = 0; i < nms_result.size(); i++)
    {
        int idx = nms_result[i];
        yolo_output result;
        result.id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = bboxes[idx];
        this->results.push_back(result);
    }
}

/**
 * @brief 物体检测 + 关键点检测
 * @param src 待检测图片
 */
void YOLO::YOLO_KEY_POINTS_DETECT(cv::Mat &src)
{
    cv::Mat blob;
    int col = src.cols;
    int row = src.rows;
    int maxLen = MAX(col, row);
    cv::Mat net_input_img = src.clone();
    if (maxLen > 1.2 * col || maxLen > 1.2 * row)
    {
        cv::Mat resize_img = cv::Mat::zeros(maxLen, maxLen, CV_8UC3);
        src.copyTo(resize_img(cv::Rect(0, 0, col, row)));
        net_input_img = resize_img;
    }
    std::vector<cv::Ptr<cv::dnn::Layer>> layer;
    std::vector<std::string> layer_names ;
    layer_names = this->yolo_net.getLayerNames();
    cv::dnn::blobFromImage(net_input_img, blob, 1 / 255.0, cv::Size(this->net_width, this->net_height), cv::Scalar(0, 0, 0), true, false);
    /* 如果在其他设置没有问题的情况下但是结果偏差很大，可以尝试下用下面两句语句 */
    //blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
    //blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
    this->yolo_net.setInput(blob);
    std::vector<cv::Mat> net_output_img;

    this->yolo_net.forward(net_output_img, this->yolo_net.getUnconnectedOutLayersNames());
#if CV_VERSION_MAJOR==4&&CV_VERSION_MINOR==6
    /* OpenCV4.6.0 */
    std::sort(net_output_img.begin(), net_output_img.end(), [](cv::Mat &A, cv::Mat &B) {return A.size[2] > B.size[2];});
#endif

    /* 结果id数组 */
    std::vector<int> class_ids;
    /* 结果每个id对应置信度数组 */
    std::vector<float> confidences;
    /* 每个id矩形框 */
    std::vector<cv::Rect> bboxes;
    std::vector<key_points> key_points_all;

    float ratio_h = (float)net_input_img.rows / this->net_height;
    float ratio_w = (float)net_input_img.cols / this->net_width;
    float pad_x = (float)((this->net_width - (float)net_input_img.cols / ratio_w) / 2);
    float pad_y = (float)((this->net_height - (float)net_input_img.rows / ratio_h) / 2);

    /* 输出的网络宽度是类别数+5 */
    //int padh = (float)this->net_height*(float)row/(float)col*0.5;
    int net_channels = this->class_num + 5 + this->key_points_num * 3;
    for(size_t stride = 0; stride < this->kpt_stride_size; stride++)
    {
        float *ptr_data = (float*)net_output_img[stride].data;
        int grid_x = (int)(this->net_width / this->net_stride[stride]);
        int grid_y = (int)(this->net_height / this->net_stride[stride]);
        for(int anchor = 0; anchor < 3; anchor++){
            const float anchor_w = this->kpt_net_anchors[stride][anchor*2];
            const float anchor_h = this->kpt_net_anchors[stride][anchor*2 + 1];
            for(size_t i = 0; i < grid_y; i++)
            {
                for(size_t j = 0; j < grid_x; j++)
                {
                    /* 获取每一行的box框中含有某个物体的概率 */
                    float bbox_score = ptr_data[4];
                    if(bbox_score >= this->bbox_threshold)
                    {
                        cv::Mat scores(1, this->class_num, CV_32FC1, ptr_data + 5);
                        cv::Point class_id_point;
                        double max_class_score;
                        cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
                        max_class_score = sigmoid_x(max_class_score);
                        if(max_class_score >= this->class_threshold)
                        {
                            float x = (sigmoid_x(ptr_data[0]) * 2.f - 0.5f + j) * this->net_stride[stride];
                            float y = (sigmoid_x(ptr_data[1]) * 2.f - 0.5f + i) * this->net_stride[stride];
                            float w = powf(sigmoid_x(ptr_data[2]) * 2.f, 2.f) * anchor_w;
                            float h = powf(sigmoid_x(ptr_data[3]) * 2.f, 2.f) * anchor_h;
                            int left = (int)(x - 0.5 * w) * ratio_w + 0.5;
                            int top  = (int)(y - 0.5 * h) * ratio_h + 0.5;
                            key_points kpts;
                            for(size_t k = 0; k < this->key_points_num; k++)
                            {
                                cv::Point p;
                                if(sigmoid_x(ptr_data[5 + this->class_num + 3 * k + 2]) > kpt_threshold)
                                {
                                    float kpt_x = (ptr_data[5 + this->class_num + 3 * k] * 2.f - 0.5f + j) * this->net_stride[stride];
                                    float kpt_y = (ptr_data[5 + this->class_num + 3 * k + 1] * 2.f - 0.5f + i) * this->net_stride[stride];
                                    kpt_x -= pad_x;
                                    kpt_y -= pad_y;
                                    kpt_x *= ratio_w;
                                    kpt_y *= ratio_h;
                                    p = cv::Point(kpt_x, kpt_y);
                                    kpts.kpt[k] = p;
                                }
                                else
                                {
                                    p = cv::Point(0, 0);
                                    kpts.kpt[k] = p;
                                }
                            }
                            class_ids.push_back(class_id_point.x);
                            confidences.push_back(max_class_score*bbox_score);
                            bboxes.push_back(cv::Rect(left, top, int(w*ratio_w), int(h*ratio_h)));
                            key_points_all.push_back(kpts);
                        }
                    }
                    /* 下一行 */
                    ptr_data += net_channels;
                }
            }
        }
    }

    /* 执行非最大抑制以消除具有较低置信度的冗余重叠框（NMS）*/
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(bboxes, confidences, this->nms_score_threshold, this->nms_threshold, nms_result);
    for(size_t i = 0; i < nms_result.size(); i++)
    {
        int idx = nms_result[i];
        yolo_output result;
        result.id = class_ids[idx];
        result.confidence = confidences[idx];
        result.bbox = bboxes[idx];
        result.kpts = key_points_all[idx];
        this->results.push_back(result);
    }
}

/**
 * @brief 物体检测结果可视化
 * @param src 待可视化图片
 */
void YOLO::YOLO_OBJECT_DRAW_PRED(cv::Mat &src)
{
    for(size_t i = 0; i < this->results.size(); i++)
    {
        cv::rectangle(src, this->results[i].bbox, this->color[this->results[i].id], 2, 8);
        /* 添加标签 */
        std::string label = this->class_name[this->results[i].id] + ":" + std::to_string(this->results[i].confidence);
        int base_line;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
        int label_top = cv::max(this->results[i].bbox.y, label_size.height);
        int label_left = this->results[i].bbox.x;
        cv::putText(src, label, cv::Point(label_left, label_top), cv::FONT_HERSHEY_SIMPLEX, 1, this->color[this->results[i].id] ,2);
    }
}

/**
  * @brief 物体检测的接口
  * @param src
  */
void YOLO::YOLO_GET_OBJECT_DETECTION(cv::Mat &src)
{
    this->YOLO_OBJECT_DETECT(src);
    this->YOLO_OBJECT_DRAW_PRED(src);
}

/**
 * @brief 物体检测 + 关键点检测可视化
 * @param src 待可视化图片
 */
void YOLO::YOLO_KEY_POINTS_DRAW_PRED(cv::Mat &src)
{
    for(size_t i = 0; i < this->results.size(); i++)
    {
        cv::rectangle(src, this->results[i].bbox, this->color[this->results[i].id], 2, 8);
        /* 添加标签 */
        std::string label = this->class_name[this->results[i].id] + ":" + std::to_string(this->results[i].confidence);
        int base_line;
        cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base_line);
        int label_top = cv::max(this->results[i].bbox.y, label_size.height);
        int label_left = this->results[i].bbox.x;
        cv::putText(src, label, cv::Point(label_left, label_top), cv::FONT_HERSHEY_SIMPLEX, 1, this->color[this->results[i].id] ,3);
        for(size_t k = 0; k < this->key_points_num; k++)
        {
            if(this->results[i].kpts.kpt[k].x == 0 &&  this->results[i].kpts.kpt[k].y == 0)
            {
                continue;
            }
            else
            {
                cv::circle(src, cv::Point(this->results[i].kpts.kpt[k].x, this->results[i].kpts.kpt[k].y), 2, this->color[this->class_num+k], 3);
            }
        }
    }
}

/**
 * @brief 物体检测 + 关键点检测的接口
 * @param src
 */
void YOLO::YOLO_GET_KEY_POINTS(cv::Mat &src)
{
    this->YOLO_KEY_POINTS_DETECT(src);
    this->YOLO_KEY_POINTS_DRAW_PRED(src);
}
