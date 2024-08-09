/*******************************************************
 文件名：FusionFunction.h
 作者：
 描述：基本的通用函数库
 版本：v1.0
 日期：2023-4-4
 *******************************************************/

#ifndef FUSIONFUNCTION_H
#define FUSIONFUNCTION_H
#include <xtensor/xnpy.hpp>
#include <xtensor/xbuilder.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xrepeat.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include "FUKalmanBoxTracker.h"
#include "json.hpp"
#include <set>

using namespace std;

#define None -10000.1
#define PI 3.1415926

namespace FusionFunction
{
    std::pair<float, bool> cal_angle_new(FUKalmanBoxTracker *tracker,nlohmann::json &m_fusion_parameter);
    //  轨迹航向角平滑
    void smooth_angle(FUKalmanBoxTracker *tracker, std::vector<int> *motor_vehicle_labels);
    std::map<int, float> neighbor_search(xt::xarray<float> &trackers, int idx, float search_radius, nlohmann::json &m_fusion_parameter);
    std::pair<float [2], float [2]> load_chedao(std::unordered_map<int, std::pair<int, float>> &m_chedao);
}

#endif // FUSIONFUNCTION_H
