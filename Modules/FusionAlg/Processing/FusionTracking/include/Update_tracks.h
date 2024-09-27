/*******************************************************
 文件名：update_tracks.h
 作者：
 描述：匹配关联之后，需要对轨迹进行位置更新和类别更新
 版本：v1.0
 日期：2023-4-4
 *******************************************************/

#ifndef UPDATE_TRACKS_H
#define UPDATE_TRACKS_H

#include "BaseAssociate.h"
#include "FunctionHub.h"
#include <numeric>
#include "json.hpp"
#include "FusionFunction.h"

class Update_tracks
{
private:
    /* data */
    nlohmann::json m_fusion_parameter;
    // std::unordered_map<int, std::pair<int, float>>  *m_chedao;
    std::vector<int> camera_create_trackers;
    std::vector<int> motor_vehicle_labels;

    int m_height;   // 车道角覆盖范围准换到像素大小
    int m_width;

    // float *m_x_limit;
    // float *m_y_limit;
    void get_state(FUKalmanBoxTracker& tracker);
    void get_angle(FUKalmanBoxTracker& tracker);

public:
    Update_tracks(nlohmann::json &parameter);
    ~Update_tracks();

    // 生成新的轨迹
    void create_tracks(std::shared_ptr<fusion_match_out> match_out, std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long timestamp, int flag);
    void create_onlyvideotracks(std::shared_ptr<fusion_match_out> match_out, std::vector<FUKalmanBoxTracker> &_m_trackers, unsigned long timestamp, int flag);

    // 轨迹更新（位置平滑和更新，航向角平滑，轨迹删除）
    void update(std::vector<FUKalmanBoxTracker> &_m_trackers, xt::xarray<float> &fusion_track_result, unsigned long timestamp, float dt);
    void update_onlyvideo(std::vector<FUKalmanBoxTracker> &_m_trackers, xt::xarray<float> &fusion_track_result, unsigned long timestamp, float dt);

private:
    void parse_json();
};
#endif