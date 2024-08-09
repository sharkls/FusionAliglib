/*******************************************************
 文件名：process_camera.h
 作者：
 描述：图像检测数据和轨迹关联匹配
 版本：v1.0
 日期：2023-4-6
 *******************************************************/

#ifndef PROCESS_CAMERA_H
#define PROCESS_CAMERA_H

#include "BaseAssociate.h"
#include "FUKalmanBoxTracker.h"
#include "FunctionHub.h"
#include "IouAssociate.h"
#include "DistanceAssociate.h"
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include "json.hpp"

class Process_camera
{
private:
    /* data */

    xt::xarray<float> camera_box; // 数据预处理后的检测信息：shape:(n, )  数据预处理后的检测信息：shape:(n, )  x, y, w, l, theta(角度), z,    h,       label, speed, id, score, x_min,y_min,x_max,y_max, data_source, data_channel
    xt::xarray<float> track_box; // 数据预处理后的目标信息：shape:(m, )   // x, y, z, w，   l,  h,   theta, id
    xt::xarray<float> camera_box_all_channel;
    nlohmann::json m_fusion_parameter;
    fusion_match_out *m_associate_out = nullptr;
    cal_dis_result *m_cal_dis_result = nullptr;
    void process_iou(std::shared_ptr<IouAssociate> iou_associate);
    void process_dis(std::shared_ptr<DistanceAssociate> dis_associate);
    void set_out();
public:
    Process_camera(nlohmann::json &parameter, std::shared_ptr<fusion_match_out> match_out, std::shared_ptr<cal_dis_result> cal_dis_result); // TSelfFusionAlgParam这里里面要包含每一路相机的内外参
    ~Process_camera();

    // 多路相机检测数据预处理，
    void process_camera_data(xt::xarray<float> *CameraResult);

    // 执行关联匹配
    void excute_match_camera_stage(std::vector<FUKalmanBoxTracker> &_trackers, std::shared_ptr<IouAssociate> iou_associate, std::shared_ptr<DistanceAssociate> dis_associate);

    // 更新类别
    void update_type(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long timestamp);
    void update_onlyvideo(std::vector<FUKalmanBoxTracker> &_trackers, unsigned long timestamp);

    // 得到相机跟踪后的图像检测信息,用以显示。
    void getCameraBoxInfo(std::vector<xt::xarray<float>> *_p_pCameraResult);

private:

};
#endif