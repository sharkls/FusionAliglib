//
// Created by root on 7/18/23.
//
#include <iostream>
#include "string"
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xview.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xstrided_view.hpp"
// #include "xtensor-blas/xlinalg.hpp"
#include "xtensor/xtensor_forward.hpp"
#include "xtensor/xio.hpp"
#include <algorithm>
#include "dbscan.h"
#include "StandardScaler.h"
#include "yaml-cpp/yaml.h"
#include "CSelfAlgParam.h"
#include "types_v5.h"

using namespace std;

class spills_detection{
public:
    spills_detection(CSelfAlgParam *m_AlgParams);
    ~spills_detection();
    void RUN(std::vector<cv::Mat> &l_vecVideoSrcData, std::vector<std::vector<yolov5Detection>> &video_detect_res);

private:
    //*****************************外设可参数********************************
    // int _oriInputW = 640;  //输入尺寸
    // int _oriInputH = 640;
    int _afterW = 640;  //输出尺寸
    int _afterH = 640;
    int _channel_num;        //通道数
    int _min_spills_pixel;      //轮廓面积最小值
    int _max_spills_pixel;  //轮廓面积最大值
    float _judge_rate;   //判断像素
    vector<int> _cls_list;  //除小目标之外的目标框进行放大，这是对应的不需要放大的类别
    float _spill_cls;        //输出的抛洒物类别
    int _x_range1;   //x,y放大的范围
    int _y_range1;
    int _x_range2;
    int _y_range2;
    int _DB_min_Pts;              //DBSCAN参数，最小点数
    float _DB_eps;              //DBSCAN参数
    int _clusters_num;            //簇个数阈值
    int _update_back_cnt;  //超过5帧，更新背景图像
    float _iou_thresh;     //抛洒物检测框和原来视频检测框计算的iou阈值

    //*****************************内部固定参数********************************
    vector<cv::Mat> _mask_list;
    queue <std::vector<cv::Mat>> _vecHistoryFrame;   //保存历史帧
    // 创建一个椭圆形的形态学操作的结构元素, 通过指定cv::MORPH_ELLIPSE作为形状参数，定义一个5x5的椭圆形结构元素，并将结果存储在kernel变量中
    cv::Mat _kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    bool _use_fix_background = false;
    cv::Mat _background_det;
    cv::Mat _background_gmm;
    cv::Mat _background;
    cv::Mat _backtmp;
    int _frame_cnt = 1;
    int _frame_cnt_gmm = 1;
    int n_clusters_ = 0;
    void mask_process(cv::Mat &mask_img);
    void determine_pixels(cv::Mat &det_frame, cv::Mat &current_frame_mask);
    cv::Mat draw_cnts(cv::Mat &img, std::vector<yolov5Detection> &count);
    void get_background(std::vector<yolov5Detection> &detections, cv::Mat &det_frame, cv::Mat &mask);
    bool judge_spills(float &x, float &y, float &right, float &bottom);
    xt::xarray<float> cluster_to_filter(vector<xt::xarray<float>> &res_contour, int &n_clusters_);
    xt::xarray<float> calcu_cnts(cv::Mat &img, int &n_clusters_); 
    xt::xarray<float> process(cv::Mat &frame, std::vector<yolov5Detection> &detections, cv::Mat &det_frame, cv::Mat &mask);
};
