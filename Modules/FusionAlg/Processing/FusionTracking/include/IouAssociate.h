#ifndef IOUASSOCIATE_H
#define IOUASSOCIATE_H
#include "BaseAssociate.h"
class IouAssociate : public BaseAssociate
{
private:
    /* data */
    xt::xarray<float> m_detections; // 关联匹配之前处理后的检测数据  // x, y, w, l, theta(角度)； x, y, theta(角度) ---距离
    xt::xarray<float> m_trackers;   // 关联匹配之前处理后的目标数据  // x, y, w, l, theta(角度)； x, y, theta(角度) ---距离
    int m_flag;
    xt::xarray<float> * m_dets;

    void setout_matched(std::vector<int>& matched_det_index, std::vector<int>& matched_trk_index, fusion_match_out *associate_out);

    void setout_unmatched(std::vector<int>& unmatched_det_index, std::vector<int>& unmatched_trk_index, fusion_match_out *associate_out);
public:

    nlohmann::json m_parameter;

    IouAssociate(nlohmann::json &parameter);
    ~IouAssociate();

    virtual void pre_process_associate_data(xt::xarray<float> *_dets, xt::xarray<float> *_trks, int flag) override;

    // 计算代价矩阵
    virtual void cal_cost_matrix(xt::xarray<float> &cost_matrix) override;

    // 计算代价矩阵
    void cal_cost_matrix_new(xt::xarray<float> &cost_matrix);

    // 匈牙利匹配
    virtual void execute_match(xt::xarray<float> _cost_matrix, fusion_match_out *associate_out) override;

    // 后处理
    virtual void post_process_associate_data(fusion_match_out *associate_out, xt::xarray<float> *_dets) override;

private:
    // 角度转弧度，以及转换航向角坐标系
    void adjust_boxes_iou_bev_input(xt::xarray<float> &origin_box);
};
#endif