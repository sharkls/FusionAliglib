//
// Created by root on 7/18/23.
//
# include "SpillsDetectAlg.h"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xslice.hpp"
#include "xtensor/xtensor_forward.hpp"
#include <cmath>
#include <cstddef>
#include <limits>
#include <opencv2/core/mat.hpp>

using namespace std;

spills_detection::spills_detection(CSelfAlgParam *m_AlgParams){
    YAML::Node video_cfg = YAML::LoadFile(m_AlgParams->m_strRootPath + m_AlgParams->m_strVideoCfgPath);
    _channel_num = video_cfg["CHANNEL_NUM"].as<int>();      //通道数
    _afterW = video_cfg["AFTER_W"].as<int>();          //输出尺寸
    _afterH = video_cfg["AFTER_H"].as<int>();          //输出尺寸
    _min_spills_pixel = video_cfg["MIN_SPILLS_PIXEL"].as<int>();      //轮廓面积最小值
    _max_spills_pixel = video_cfg["MAX_SPILLS_PIXEL"].as<int>();      //轮廓面积最大值
    _judge_rate = video_cfg["JUDGE_RATE"].as<float>();   //判断像素
    _cls_list = video_cfg["CLS_LIST"].as<vector<int>>();  //除小目标之外的目标框进行放大，这是对应的不需要放大的类别
    _spill_cls = video_cfg["SPILLS_CLS"].as<float>();        //输出的抛洒物类别
    _x_range1 = video_cfg["X_RANGE1"].as<int>();   //x,y放大的范围
    _y_range1 = video_cfg["Y_RANGE1"].as<int>();
    _x_range2 = video_cfg["X_RANGE2"].as<int>();
    _y_range2 = video_cfg["Y_RANGE2"].as<int>();
    _DB_min_Pts = video_cfg["DB_MIN_PTS"].as<int>();              //DBSCAN参数，最小点数
    _DB_eps = video_cfg["DB_EPS"].as<float>();              //DBSCAN参数
    _clusters_num = video_cfg["CLUSTERS_NUM"].as<int>();           //簇个数阈值
    _update_back_cnt = video_cfg["UPDATE_BACK_CNT"].as<int>();  //超过5帧，更新背景图像
    _iou_thresh = video_cfg["IOU_THRESH"].as<float>();       //抛洒物检测框和原来视频检测框计算的iou阈值

    // read masks
    for (int i=0; i<_channel_num; i++){
        cv::Mat img_mask = cv::imread(m_AlgParams->m_strRootPath + video_cfg["SPILLS_MSAK_PATH"].as<std::string>() + "mask_" + to_string((i + 1)) + ".png", cv::IMREAD_GRAYSCALE);
        cv::Size targetSize(_afterW, _afterH);
        cv::Mat resizedMask;
        cv::resize(img_mask, resizedMask, targetSize);
        cv::Mat thresholded_img;
        cv::threshold(resizedMask, thresholded_img, 254, 255, cv::THRESH_BINARY); //thresholded_img输出的阈值化后的图像,254设定阈值,255设置阈值之上的最大值,cv::THRESH_BINARY使用二进制阈值化
        cv::Mat _mask;
        cv::threshold(thresholded_img, _mask, 0, 255, cv::THRESH_BINARY_INV); 
        _mask_list.push_back(_mask);
    }    
}

spills_detection::~spills_detection(){
}

cv::Mat spills_detection::draw_cnts(cv::Mat &img, std::vector<yolov5Detection> &count){
    for (size_t i = 0; i < count.size(); i++){
        // float x = cnt.m_x / _afterW * _oriInputW;
        // float y = cnt.m_y / _afterH * _oriInputH;
        // float z = cnt.m_w / _afterW * _oriInputW;
        // float q = cnt.m_h / _afterH * _oriInputH;
        float x = count[i].rect.left_top_x;
        float y = count(i, 1);
        float w = count(i, 2);
        float h = count(i, 3);
        float conf = count(i, 4);
        float cls = count(i, 5);
        // pixincheng, class: 13 roadblock 15 construction_sign_board 16 spillage
        // now baseline yolov5 cls
        // 0"person",1"bicycle",2"tricycle",3"car",4"truck",5"bus",6"dump_truck",7"road_cones",8"others_bicycle_per",9"MTfood_bicycle_per",10"ELMfood_bicycle_per",
        // 11"JD_tricycle",12"SF_tricycle",13"YZ_tricycle",14"YD_tricycle",15"slagcar",16"tanker",17"road_work_sign",18"parking_tripod"]   
        // vector<int> cls_list = {7, 17, 18};
        if (find(_cls_list.begin(), _cls_list.end(), cls) == _cls_list.end()) {
            int left, top, right, bottom;
            if ((y + h) < img.rows){
                left = max(static_cast<int>(round(x)) - _x_range1, 0);
                top = max(static_cast<int>(round(y)) - _y_range1, 0);
                right = min(static_cast<int>(round(w)) + _x_range1, img.cols);
                bottom = min(static_cast<int>(round(h)) + _y_range1, img.rows);
            }
            else{
                left = max(static_cast<int>(round(x)) - _x_range2, 0);
                top = max(static_cast<int>(round(y)) - _y_range2, 0);
                right = min(static_cast<int>(round(w)) + _x_range2, img.cols);
                bottom = min(static_cast<int>(round(h)) + _y_range2, img.rows);
            }
            // 将给定区域的像素值设为0
            cv::Mat region = img(cv::Range(top, bottom), cv::Range(left, right));
            region.setTo(cv::Scalar(0));
        } 
        
    }
    return img;
}

void spills_detection::determine_pixels(cv::Mat &det_frame, cv::Mat &current_frame_mask){
    // 遍历图像的每个像素，并检查每个通道的像素值是否为0
    for (int i = 0; i < det_frame.rows; i++) {
        for (int j = 0; j < det_frame.cols; j++) {
            // 获取像素的RGB通道值
            cv::Vec3b pixel = det_frame.at<cv::Vec3b>(i, j);
            // 判断所有通道的像素值是否都为0
            if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0) {
                current_frame_mask.at<uchar>(i, j) = 255; // 设置为白色（255）
            }
            else {
                current_frame_mask.at<uchar>(i, j) = 0; // 设置为黑色（0）
            }
        }
    }
}

void spills_detection::get_background(std::vector<yolov5Detection> &detections, cv::Mat &det_frame, cv::Mat &mask){
    if (!det_frame.empty()) {
        if (detections.size() > 2){
            if (_backtmp.empty()){
                _backtmp = det_frame;
            }
            else{
                det_frame = draw_cnts(det_frame, detections);

                // 创建一个与 det_frame 相同大小的掩码矩阵
                cv::Mat current_frame_mask(det_frame.rows, det_frame.cols, CV_8UC1);
                determine_pixels(det_frame, current_frame_mask);

                cv::Mat result = det_frame;
                // 将 _backtmp 中满足掩码条件的像素复制到 result 中
                _backtmp.copyTo(result, current_frame_mask);
                det_frame.copyTo(result, ~current_frame_mask );
                _backtmp = result;
                _frame_cnt += 1;
                if (_frame_cnt > _update_back_cnt){
                    _frame_cnt = 1;
                    _background_det = _backtmp;
                }       
            }

        }
        else {
            cv::Mat roi1;
            cv::bitwise_and(det_frame, det_frame, roi1, mask);       //det_frame输入待处理图像,det_frame输出结果图像，可以与待处理图像相同，表示直接在原图像上进行操作
                                                                                    //result是用于存储结果的图像对象, _mask是输入的掩膜图像对象
            _frame_cnt_gmm += 1;
            if (_frame_cnt_gmm > _update_back_cnt){
                _frame_cnt_gmm = 1;
                _background_gmm = roi1;
            }
        }
            
    }
    if (!_background_gmm.empty()){
        _background = _background_gmm;
        _use_fix_background = true;
        return;
    }
    if (!_background_det.empty()){
        _background = _background_det;
        _use_fix_background = true;
        return;
    }
}

bool spills_detection::judge_spills(float &x, float &y, float &right, float &bottom){
    float rate = _judge_rate;
    float area = (right - x) * (bottom - y);
    float thresh_area = x * rate * y * rate;
    if (area > thresh_area){
        return false;
    }
    else{
        return true;
    }
}

xt::xarray<float> standardize(const xt::xarray<float>& input)
{
    // 计算均值和标准差
    float mean = xt::mean(input)();
    float stddev = xt::stddev(input)();
    // 标准化处理
    xt::xarray<float> standardized = (input - mean) / stddev;
    return standardized;
}

xt::xarray<float> spills_detection::cluster_to_filter(vector<xt::xarray<float>> &res_contour, int &n_clusters_){
    xt::xarray<float> res_all = xt::empty<float>({0, 6});
    if (res_contour.size()){
        // 获取数组的形状, [x, y, right, bottom, 1, cls]
        vector<size_t> res_shape = {res_contour.size(), 6};
        // 将 vector<xt::xarray<int>> 转换为 xt::xarray<int>
        xt::xarray<float> res_pre = res_contour[0];
        for (size_t i = 1; i < res_contour.size(); ++i) {
            res_pre = xt::concatenate(xt::xtuple(res_pre, res_contour[i]), 0);
        }
        res_pre.reshape(res_shape); // n*6
        auto X = xt::view(res_pre, xt::all(), xt::range(0, 2));
        // 标准化处理
        StandardScaler scaler;
        scaler.fit(X);
        xt::xarray<float> normalized_X = scaler.transform(X);
        //将normalized_X处理成DBSCAN需要的输入格式
        vector<DB_Point> db_points;
        for (size_t i=0; i<normalized_X.shape(0); i++) {
            DB_Point db_point;
            db_point.x = normalized_X(i, 0);
            db_point.y = normalized_X(i, 1);
            db_points.push_back(db_point);
        }
        // DBSCAN, 聚类标签存储在 labels 变量中
        DBSCAN *db = new DBSCAN(_DB_min_Pts, _DB_eps, db_points); 
        db->run();
        //标签的值包括正整数（表示簇的索引）和-2（表示噪音点）。如果某个数据点的标签为-2，则表示该点被视为离群点或噪音点，不属于任何簇
        // // label等于-2说明是离散的单个目标，有可能是抛洒物要保留的, label不等于-2 说明已经成了簇，很可能是光影，或者车辆运动阴影产生的一堆误检框
        // // 所以这里保留label等于-2的目标，keep_idx是对应的行索引
        unordered_set<int> unique_labels;
        vector<int> keep_idx;
        for (size_t i = 0; i<db->m_points.size(); i++)
        {
            // cout << "********************db->m_points[i].clusterID: " << db->m_points[i].clusterID << endl;
            unique_labels.insert(db->m_points[i].clusterID);
            if (db->m_points[i].clusterID == -2)
            {
                keep_idx.push_back(i);
            }
        }
        // res_pre保留对应的行数
        res_all = xt::view(res_pre, xt::keep(keep_idx));

        // 计算簇数
        n_clusters_ = unique_labels.size();
        if (unique_labels.find(-2) != unique_labels.end())
        {
            n_clusters_ -= 1;
        }
    }
    return res_all;
}

xt::xarray<float> spills_detection::calcu_cnts(cv::Mat &img, int &n_clusters_){
    // contours来存储轮廓的点集，hierarchy来存储轮廓的层次结构信息
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // 查找图像中的轮廓。接受输入图像(img)、轮廓点集(contours)、层次结构信息(hierarchy)、轮廓检测模式(cv::RETR_EXTERNAL)以及轮廓的近似方法(cv::CHAIN_APPROX_NONE)等参数
    cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    // 对每个轮廓进行处理
    vector<xt::xarray<float>> res_contour;
    for (size_t i = 0; i < contours.size(); i++) {
        // 处理轮廓 contours[i]
        double area = cv::contourArea(contours[i]); // 计算轮廓面积
        cv::Rect boundingRect = cv::boundingRect(contours[i]); // 获得一个图像的最小矩形边框, 返回四个参数: 左上角坐标，矩形的宽高
        float x = boundingRect.x;
        float y = boundingRect.y;
        float w = boundingRect.width;
        float h = boundingRect.height;
        if (area < _min_spills_pixel or area > _max_spills_pixel){
            continue;
        }
        // top = y
        float bottom = min(float(img.rows), y + h);
        // left = x
        float right = min(float(img.cols), x + w);
        // x = x / _oriInputW * _afterW;
        // y = y / _oriInputH * _afterH;
        // right  = right / _oriInputW * _afterW;
        // bottom = bottom / _oriInputH * _afterH;
        if (judge_spills(x, y, right, bottom)){
            xt::xarray<float> _arr {{x, y, right, bottom, 1, _spill_cls}}; //添加抛洒物类别为19
            res_contour.push_back(_arr);
        }
    }
    xt::xarray<float> filter_res = cluster_to_filter(res_contour, n_clusters_);
    return filter_res;
}

xt::xarray<float> spills_detection::process(cv::Mat &frame, std::vector<yolov5Detection> &detections, cv::Mat &det_frame, cv::Mat &mask){
    xt::xarray<float> contour_aft = xt::empty<float>({0, 6});
    if (!_background.empty()) {
        cv::Mat roi2, back, diff, gray, fgmask;
        cv::bitwise_and(frame, frame, roi2, mask);
        cv::bitwise_and(_background, _background, back, mask);
        cv::absdiff(back, roi2, diff);  //图像差分操作
        cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY); // 颜色空间转换（BGR到灰度)
        cv::threshold(gray, fgmask, 20, 255, cv::THRESH_BINARY); 
        // 开运算，即先腐蚀后膨胀，用于消除噪声
        // cv::morphologyEx()开运算操作, 输入图像(fgmask)、输出图像(result)、形态学操作类型(cv::MORPH_OPEN)、结构元素(kernel)、锚点位置(cv::Point(-1,-1))以及迭代次数(1)等参数
        cv::morphologyEx(fgmask, fgmask, cv::MORPH_OPEN, _kernel, cv::Point(-1, -1), 1);
        // 闭运算，即先膨胀后腐蚀，用于消除小黑洞
        cv::morphologyEx(fgmask, fgmask, cv::MORPH_CLOSE, _kernel, cv::Point(-1, -1), 1);

        contour_aft = calcu_cnts(fgmask,  n_clusters_);
        // cout << "11111<<<<<<<<<<<<<<<contour_aft.shape: " << contour_aft.shape(0) << ", " << contour_aft.shape(1) << endl;
        // cout << "11111<<<<<<<<<<<<<<<contour_aft(0, ) " << contour_aft(0, 0) << ", " << contour_aft(0, 1) << ", " << contour_aft(0, 2) << ", " 
        //                 << contour_aft(0, 3) << ", " << contour_aft(0, 4) << ", " << contour_aft(0, 5) << ", "  << endl;

        if (n_clusters_ > _clusters_num){
            _background.setTo(cv::Scalar(0));
            _background_gmm.setTo(cv::Scalar(0));
            _background_det.setTo(cv::Scalar(0));
            // contour_aft = xt::empty<float>({0, 6});
            cout << "======update background= n_clusters_: " << n_clusters_ << endl;
        }
    }
    else {
        get_background(detections, det_frame, mask);
    }
    n_clusters_ = 0;
    // 输出n*6的抛洒物检测框，xt::xarray<float> contour_aft 
    // {{x1, y1, right1, bottom1, 1, 19}, {x2, y2, right2, bottom2, 1, 19}, ...}, 添加抛洒物类别为19
    return contour_aft;
}


// 计算两个矩形的交集面积
float calculateIntersectionArea(const xt::xarray<float>& rectA, const xt::xarray<float>& rectB) {
    float x1 = std::max(rectA(0), rectB(0));
    float y1 = std::max(rectA(1), rectB(1));
    float x2 = std::min(rectA(2), rectB(2));
    float y2 = std::min(rectA(3), rectB(3));

    if (x1 >= x2 || y1 >= y2) {
        return 0.0f;
    }
    else {
        float intersectionArea = (x2 - x1) * (y2 - y1);
        return intersectionArea;
    }
}
// 计算两个矩形的IOU
float calculateIOU(const xt::xarray<float>& rectA, const xt::xarray<float>& rectB) {
    float intersectionArea = calculateIntersectionArea(rectA, rectB);

    float rectAArea = (rectA(2) - rectA(0)) * (rectA(3) - rectA(1));
    float rectBArea = (rectB(2) - rectB(0)) * (rectB(3) - rectB(1));

    float iou = intersectionArea / (rectAArea + rectBArea - intersectionArea);
    return iou;
}

void spills_detection::RUN(std::vector<cv::Mat> &l_vecVideoSrcData, std::vector<std::vector<yolov5Detection>> &video_detect_res){

    if (_vecHistoryFrame.size()){
        for (int n=0; n<_channel_num; n++){
            xt::xarray<float> spills_res = process(l_vecVideoSrcData[n], video_detect_res[n], _vecHistoryFrame.front()[n], _mask_list[n]);

            //当抛洒物检测结果不为空时
            if (spills_res.shape(0)){
                //当视频检测结果不为空时,计算Iou
                if (video_detect_res[n].size()){
                    size_t num_spills = spills_res.shape()[0];
                    size_t num_video_detections = video_detect_res[n].size();
                    std::vector<xt::xarray<float>> new_spills_detect_res;
                    for (size_t i = 0; i < num_spills; i++) {
                        bool should_add = true;
                        for (size_t j = 0; j < num_video_detections; j++) {
                            float iou = calculateIOU(xt::view(spills_res, i, xt::all()), xt::view(video_detect_res[n], j, xt::all()));
                            // std::cout << "IoU of spills_res[" << i << "] and video_detect_res[n][" << j << "]: " << iou << std::endl;
                            if (iou > _iou_thresh) {
                                should_add = false;
                                break;
                            }
                        }
                        if (should_add) {
                            xt::xarray<float> spills_tmp = xt::view(spills_res, i, xt::all());
                            new_spills_detect_res.push_back(spills_tmp.reshape({1, 6}));
                        }
                    }
                    if (new_spills_detect_res.size()){
                        for (std::size_t x = 0; x < new_spills_detect_res.size(); x++){
                            video_detect_res[n] = xt::concatenate(xt::xtuple(video_detect_res[n], new_spills_detect_res[x]), 0);
                        }
                    }
                }
                //当视频检测结果为空时，直接添加进结果
                else{
                    video_detect_res[n] = xt::concatenate(xt::xtuple(video_detect_res[n], spills_res), 0);
                } 
            }
            //***************************************只保存抛洒物的检测结果*********************************
            // static int t2 = 1;
            // string save_file2 = "/data/tmp/spills/" + std::to_string(t2++) + ".txt";
            // if (spills_res.shape(0)){
            //     // 抛洒物检测框保存到txt文件中
            //     std::ofstream file2(save_file2);
            //     for (size_t i = 0; i < spills_res.shape(0); ++i) {
            //         for (size_t j = 0; j < spills_res.shape(1); ++j) {
            //             file2 << spills_res(i, j) << " ";
            //         }
            //         file2 << std::endl;
            //     }
            //     file2.close();
            // }

        }
        _vecHistoryFrame.pop();
    }
    _vecHistoryFrame.push(l_vecVideoSrcData);
}