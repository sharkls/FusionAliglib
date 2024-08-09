#include <iostream>
#include "fusion_object.h"
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xsort.hpp>

TypeFusion::TypeFusion(xt::xarray<float> bbox, nlohmann::json &fusion_parameter){
    int fusion_label_size = 0;
    for (int i = 0; i < fusion_parameter["fusion_param"]["m_strFusionClass"].size(); i++)
    {
        if(fusion_parameter["fusion_param"]["m_strFusionClass"][i] != "None"){
            _framework[fusion_parameter["fusion_param"]["m_strFusionClass"][i]] = i;
            fusion_label_size++;
        }
    }
    _framework["unknown"] = fusion_label_size;
    mass_A = xt::zeros<float>({int(_framework.size())});

    fusion_label = bbox[7];
    fusion_score = bbox[10];
    if (fusion_score >= 0.98){
        fusion_score = 0.98;
    }
    _bbox = bbox;
    mass_A = xt::full_like(mass_A,1-fusion_score);
    mass_A[int(fusion_label)] = 1;
    mass_A[int(mass_A.size()-1)] = 0;
    float intersection_center[2]={0,0};
    memcpy(&_intersection_center[0], &intersection_center[0], 2*sizeof(float));
}

void TypeFusion::updateType(xt::xarray<float> bbox, int src){
    float sensor_reduction_ratio;
    float detect_score;
    switch (src)
    {
        case 1:
            sensor_reduction_ratio = 1;
            break;
        case 2:
            sensor_reduction_ratio = 0.5;
            break;
        default:
            sensor_reduction_ratio = 0.5;
    }

    detect_score = bbox[10];
    detect_score *= sensor_reduction_ratio;
    bbox[10] = detect_score;
    mass_B = xt::zeros<float>({int(_framework.size())});
    mass_B = xt::full_like(mass_B,1-bbox[10]);

    mass_B[int(bbox[7])] = 1;
    mass_B[int(mass_B.size()-1)] = 0;
    xt::xarray<float> T = xt::empty<float>({int(_framework.size())});
    float fenmu_A;
    float fenmu_B;
    xt::xarray<float> fenzi_A;
    xt::xarray<float> fenzi_B;
    xt::xarray<float> bayes_mass_A;
    xt::xarray<float> bayes_mass_B;
    xt::xarray<float> k;
    float total_k;
    xt::xarray<float> new_total;
    for(int i = 0; i < _framework.size(); i++){
        T(i) = 1;
    }
    T( _framework.size() - 1) = _framework.size() - 1;
    fenmu_A = xt::linalg::dot(mass_A, T)[0];
    fenmu_B = xt::linalg::dot(mass_B, T)[0];
    fenzi_A = mass_A*T;
    fenzi_B = mass_B*T;
    fenzi_A[int(fenzi_A.size())-1] = 0;
    fenzi_B[int(fenzi_B.size())-1] = 0;
    bayes_mass_A = (fenzi_A / fenmu_A);
    bayes_mass_B = (fenzi_B / fenmu_B);
    k = bayes_mass_A*bayes_mass_B;
    k[int(k.size())-1] = 0;
    total_k = xt::sum(k)[0];
    xt::xarray<float> new_label_score;
    new_total = bayes_mass_A*bayes_mass_B;
    new_label_score = new_total / total_k;
    mass_A = new_label_score;
    float new_score = xt::amax(new_label_score)[0];
    int new_label = int(xt::argmax(new_label_score)[0]);

    if (new_score >= 1){
        new_score = 0.98;
        mass_A[new_label] = 0.98;
        mass_A[int(mass_A.size()-1)] = 0;
    }

    fusion_score = new_score;
    fusion_label = new_label;
}