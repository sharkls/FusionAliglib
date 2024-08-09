/*******************************************************
 fusion_object.h
 作者：zly
 描述：基于ds推理的类别融合方法
 版本：v1.0
 日期：2023-5-18
 *******************************************************/

#ifndef FUSION_OBJECT_H
#define FUSION_OBJECT_H
#include <map>
#include <xtensor/xarray.hpp>
#include "json.hpp"

class TypeFusion{
public:
    TypeFusion(xt::xarray<float> bbox, nlohmann::json &fusion_parameter);
    TypeFusion(){};
    virtual ~TypeFusion(){};
    void updateType(xt::xarray<float> bbox, int src = 2);
    std::map<std::string,int> _framework;
    xt::xarray<float> mass_A;
    xt::xarray<float> mass_B;
    int fusion_label;
    float fusion_score;
    xt::xarray<float> _bbox;
    float _dis_thresh ;
    float _intersection_center[2];
};
#endif