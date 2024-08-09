/*******************************************************
 文件名：FunctionHub.h
 作者：
 描述：基本的通用函数库
 版本：v1.0
 日期：2023-4-4
 *******************************************************/

#ifndef FUNCTIONHUB_H
#define FUNCTIONHUB_H
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xrepeat.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include "json.hpp"
#include <set>

using namespace std;

#define None -10000.1
#define PI 3.1415926


namespace FunctionHub
{
    xt::xarray<float> corners_nd(xt::xarray<float> &dims, float origin = 0.5);
    xt::xarray<float> rotation_2d(xt::xarray<float> &corners, const xt::xarray<float> &angles);
    xt::xarray<float> center_to_corner_box2d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                             xt::xarray<float> &angles, float origin = 0.5);

    xt::xarray<float> center_to_corner_box3d(xt::xarray<float> &centers, xt::xarray<float> &dims,
                                             float &angles, float origin = 0.5, int axis = 2);
    void rotation_3d_in_axis(xt::xarray<float> &points, float &angles, int axis = 0);

    xt::xarray<float> corner_to_standup_nd(xt::xarray<float> &boxes_corner);

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc(xt::xarray<float> &dets,
                                                                  xt::xarray<float> &trackers);
                                                                  

    std::pair<xt::xarray<float>, xt::xarray<float>> rotate_nms_cc_diou(xt::xarray<float> &dets,
                                                                  xt::xarray<float> &trackers);
                                                                  
    float cal_angle(std::vector<xt::xarray<float>> &state_list, float &thresh);


    std::pair<xt::xarray<float>, xt::xarray<float>> iou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);

    std::pair<xt::xarray<float>, xt::xarray<float>> diou_jit_new(xt::xarray<float> &boxes,
                                                                xt::xarray<float> &query_boxes,
                                                                float eps = 0.0);

    float IOU_calc(xt::xarray<float> box1, xt::xarray<float> box2, bool wh = false);

    // 距离计算方法，传入引用，在被调用之前，先定三个距离矩阵
    void cal_distance(xt::xarray<float> &detections, xt::xarray<float> &trackers, xt::xarray<float> &head_dis_new, xt::xarray<float> &ano_dis_new, xt::xarray<float> &square_dis, xt::xarray<int> &quadrant, int flag);
    
    // nms
    void result_nms(xt::xarray<float> &result, std::vector<int> &delete_index, float nms_min_threshold, float nms_max_threshold, bool flag = true);

    // 计算目标对应在车道图上的索引
    std::pair<int, int> get_pixel_location_xc(float x, float y, int width, int height, float x_limit[], float y_limit[]);
    // 获得车道信息，包括车道号和车道角
    std::pair<int, float> get_lane_info(float x, float y, int width, int height, float x_limit[], float y_limit[], std::unordered_map<int, std::pair<int, float>>  *chedao);
}

#endif // FUSION_ALG_BOX_OPS_H
