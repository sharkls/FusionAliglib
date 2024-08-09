/*******************************************************
 文件名：process_lidar.h
 作者：
 描述：雷达检测数据和轨迹关联匹配
 版本：v1.0
 日期：2023-4-4
 *******************************************************/
#ifndef PROCESS_LIDAR_H
#define PROCESS_LIDAR_H


#include "BaseAssociate.h"
#include "FUKalmanBoxTracker.h"
#include "IouAssociate.h"
#include "DistanceAssociate.h"
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include "json.hpp"

#define PI 3.1415926

class Process_lidar
{
private:
    /* data */
    xt::xarray<float> lidar_box; // 数据预处理后的检测信息：shape:(n, )  x, y, w, l, thetatheta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box; // 数据预处理后的目标信息：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> younger_track_box; // 数据预处理后的目标信息（新跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    xt::xarray<float> elder_track_box; // 数据预处理后的目标信息（旧跟踪目标）：shape:(m, )  x, y, z, w，   l,             h, theta(角度), speed
    std::vector<int> vaild_younger;
    std::vector<int> vaild_elder;
    // // 目标向量（需要外面传进来）
    nlohmann::json m_fusion_parameter;
    std::unordered_map<int, std::pair<int, float>> *m_chedao = nullptr;
    fusion_match_out *m_associate_out = nullptr;
    cal_dis_result *m_cal_dis_result = nullptr;
    void setout_det(std::vector<int>& matched_all_dets_index, std::vector<int>& unmatched_elder_trks_index, std::vector<int>& matched_elder_trks_index);
    void setout_trk(std::vector<int>& matched_all_dets_index, std::vector<int>& unmatched_elder_trks_index, std::vector<int>& matched_elder_trks_index);

    void process_iou(std::shared_ptr<IouAssociate> iou_associate);
    void process_second(std::shared_ptr<DistanceAssociate> dis_associate);
    void process_third(std::shared_ptr<DistanceAssociate> dis_associate);
public:
    Process_lidar(nlohmann::json &parameter, std::shared_ptr<fusion_match_out> match_out, std::shared_ptr<cal_dis_result> cal_dis_result);
    ~Process_lidar();

    // 雷达检测数据预处理
    void process_lidar_data(xt::xarray<float> *p_pPcResult);

    // 执行关联匹配
    void excute_match_lidar_stage_3rd_match(std::vector<FUKalmanBoxTracker> &_trackers, std::shared_ptr<IouAssociate> iou_associate, std::shared_ptr<DistanceAssociate> dis_associate);

    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long pc_timestamp);
};
#endif