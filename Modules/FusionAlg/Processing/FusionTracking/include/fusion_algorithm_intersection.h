#ifndef FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H
#define FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H

#include <iostream>
#include <algorithm>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>
#include "GlobalContext.h"
#include "CAlgResult.h"
#include "CSelfAlgParam.h"
#include "ICommonAlg.h"
#include "yaml-cpp/yaml.h"
#include "CGeneralTrans.h"
#include "FUKalmanBoxTracker.h"
#include "Process_lidar.h"
#include "Process_camera.h"
#include "Update_tracks.h"
#include "FusionException.h"
#include "IouAssociate.h"
#include "DistanceAssociate.h"

class Fusion_Algorithm_Intersection : public ICommonAlg ,CGeneralTrans
{
public:
    Fusion_Algorithm_Intersection();
    virtual ~Fusion_Algorithm_Intersection();

    void init(CSelfAlgParam *p_pAlgParam);
    void execute();
    void FusionPc();
    void FusionOnlyVideo();

    void setCommonData(CCommonDataPtr p_commonData) override
    {
        m_CommonData = p_commonData;
    }

    CCommonDataPtr getCommonData() override
    {
        return m_CommonData;
    }

private:
    // private variables
    CSelfAlgParam m_stFusionAlgParams;
    CAlgResult m_stFusionAlgSrcdata;

    unsigned long m_pc_timestamp;
    unsigned long m_video_timestamp;
    std::vector<unsigned long> m_camera_timestamp;

    // private functions
    CCommonDataPtr m_CommonData;
    std::vector<FUKalmanBoxTracker> m_trackers; // 存储所有的轨迹
    std::unordered_map<int, std::pair<int, float>> m_chedao; // 车道航向角
    std::shared_ptr<Process_camera> m_camera_handler; // process_camera对象指针
    std::shared_ptr<Process_lidar> m_lidar_handler;   // process_lidar对象指针
    std::shared_ptr<Update_tracks> m_update_handler;
    
    std::shared_ptr<fusion_match_out> m_match_out;
    std::shared_ptr<cal_dis_result> m_cal_dis_result;
    std::shared_ptr<IouAssociate> m_iouassociate_handler;
    std::shared_ptr<DistanceAssociate> m_disassociate_handler;

    unsigned long last_timestamp = 0;
    float dt;
    float m_xlimit[2], m_ylimit[2];   // 车道限制
    int time_since_pcinput = 0;
    int time_pcinput_last = 0;
    int ModelofFusion = 1; // 1:pc 2:only video
    int pcWait = 500;
    int pcLast = 5;
};

#endif //FUSION_ALG_FUSION_ALGORITHM_INTERSECTION_H
