#ifndef DISTANCEASSOCIATE_H
#define DISTANCEASSOCIATE_H

#include "json.hpp"
#include "BaseAssociate.h"
#include "FusionFunction.h"

class DistanceAssociate : public BaseAssociate
{
private:
    /* data */
    // m_detections: x,y,w,l,theta(角度),z,h,label,speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel  雷达距离匹配时
    // m_trackers: x, y,z,w,l,h,theta(角度),id                                                                        雷达距离匹配时
    xt::xarray<float> m_detections; // 关联匹配之前处理后的检测数据 
    xt::xarray<float> m_trackers;   // 关联匹配之前处理后的轨迹数据
    cal_dis_result* m_cal_dis_result = nullptr;
    int m_flag;  // 是雷达距离匹配，还是相机距离匹配
    xt::xarray<int> quadrant; //dets相对于trks的象限
    std::vector<int> motor_vehicle_labels;
    bool adjust_match(int flag, int det_index, int trk_index);
    void setout_matched(std::vector<int>& matched_det_index, std::vector<int>& matched_trk_index, fusion_match_out *associate_out);
    void setout_unmatched(std::vector<int>& unmatched_det_index, std::vector<int>& unmatched_trk_index, fusion_match_out *associate_out);

    void setout3rd_matched(std::vector<int>& matched_det_index, std::vector<int>& matched_trk_index, fusion_match_out *associate_out);
    void setout3rd_unmatched(std::vector<int>& unmatched_det_index, std::vector<int>& unmatched_trk_index, fusion_match_out *associate_out);

public:
    nlohmann::json m_parameter;
    std::vector<int>* m_vaild_elder;
    std::vector<int>* m_vaild_younger;
    DistanceAssociate(std::shared_ptr<cal_dis_result> _cal_dis_result, nlohmann::json &parameter);
    ~DistanceAssociate();
    virtual void pre_process_associate_data(xt::xarray<float> *_dets, xt::xarray<float> *_trks, int flag) override;
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) override;
    virtual void execute_match(xt::xarray<float> _cost_matrix, fusion_match_out *associate_out) override;
    virtual void post_process_associate_data(fusion_match_out *associate_out, xt::xarray<float> *_dets) override;
    virtual void post_process_associate_data_third(fusion_match_out *associate_out, std::vector<int> *vaild_elder, std::vector<int> *vaild_younger);
};
#endif