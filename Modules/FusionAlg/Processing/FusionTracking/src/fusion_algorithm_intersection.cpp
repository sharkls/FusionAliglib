#include "fusion_algorithm_intersection.h"
#include "CGeneralTrans.h"
#include "FunctionHub.h"

int FUKalmanBoxTracker::_count = 0;

Fusion_Algorithm_Intersection::Fusion_Algorithm_Intersection()
{
    LOG(INFO) << "Fusion_Algorithm_Intersection processing  statuse :   Built.";
}

Fusion_Algorithm_Intersection::~Fusion_Algorithm_Intersection()
{
}

void Fusion_Algorithm_Intersection::init(CSelfAlgParam *p_pAlgParam)
{
    LOG(INFO) << "Fusion_Algorithm_Intersection processing status :   Initialize start.";
    m_stFusionAlgParams = static_cast<CSelfAlgParam >(*p_pAlgParam);

    // 获取点云等待帧数和最后帧数
    pcWait = m_stFusionAlgParams.m_fusion_parameter["fusion_param"]["PcWaitFrame"];
    pcLast = m_stFusionAlgParams.m_fusion_parameter["fusion_param"]["PcLastFrame"];

    m_match_out = make_shared<fusion_match_out>();
    m_cal_dis_result = make_shared<cal_dis_result>();
    m_lidar_handler = make_shared<Process_lidar>(m_stFusionAlgParams.m_fusion_parameter, m_match_out, m_cal_dis_result);
    m_update_handler = make_shared<Update_tracks>(m_stFusionAlgParams.m_fusion_parameter);
    m_iouassociate_handler = std::make_shared<IouAssociate>(m_stFusionAlgParams.m_fusion_parameter);
    m_disassociate_handler =  std::make_shared<DistanceAssociate>(m_cal_dis_result, m_stFusionAlgParams.m_fusion_parameter);
}

void Fusion_Algorithm_Intersection::execute()
{
    m_camera_timestamp.clear();
    auto t_start = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "Fusion_Algorithm_Intersection::execute  status :  started.";

    // 获取预处理完的数据
    m_stFusionAlgSrcdata = getCommonData()->FuTrAlgdata;
    time_since_pcinput ++;      //点云数据丢失帧数（当接收到点云数据后进行重置）
    for (int t = 0; t < m_stFusionAlgSrcdata.vecFrameResult().size(); t++)
    {
        if(m_stFusionAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_PC_RESULT)         // 雷达数据
        {
            m_pc_timestamp = m_stFusionAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
            time_since_pcinput = 0;
            time_pcinput_last ++;

        }else if(m_stFusionAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_VIDEO_RESULT)
        {
            m_camera_timestamp.push_back(m_stFusionAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB]);
            m_video_timestamp = m_stFusionAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
        }
    }

    // 如果点云数据存在， 且modelofFusion 为2（纯视频） ，且雷达输入数据帧数累计大于pcLast， 进行雷达跟踪
    if(time_since_pcinput == 0 && ModelofFusion == 2 && time_pcinput_last > pcLast){
        LOG(ERROR)<<"---------------------CHANGE PC MODEL---------------------";
        m_trackers.clear();
        m_trackers.shrink_to_fit();
        ModelofFusion = 1;
        last_timestamp = m_pc_timestamp;
        throw FusionException();
    }

    // 当前帧数据缺失点云数据 且 点云连续缺失数据帧数小于pcLast
    if(time_since_pcinput > 0 && time_since_pcinput < pcWait){
        LOG(ERROR)<<"---------------------NO PCDATA---------------------";
        time_pcinput_last = 0;
        throw FusionException();
    }

    // 点云连续丢失帧数等于pcWait（5）时， 切换到图像跟踪并抛出异常
    if(time_since_pcinput == pcWait){
        LOG(ERROR)<<"---------------------CHANGE ONLY VIDEO MODEL---------------------";
        m_trackers.clear();
        m_trackers.shrink_to_fit();
        ModelofFusion = 2;
        last_timestamp = m_video_timestamp;
        throw FusionException();
    }

    if(ModelofFusion == 1){
        FusionPc();
    }
    if(ModelofFusion == 2){
        time_since_pcinput = pcWait + 1;
        // FusionOnlyVideo();
        LOG(INFO) << "------------------------------Enter FusionOnlyVideo mode!--------------------";
    }

    LOG(INFO)<< "Fusion_Algorithm_Intersection::execute  status : finished! ";
    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
}

void Fusion_Algorithm_Intersection::FusionPc()
{   
    // 获取融合后的数据
    std::vector<xt::xarray<float>> &video = getCommonData()->m_vecVideoXarrayResult;
    xt::xarray<float> & pc = getCommonData()->m_fPcXarrayResult;

    // predict
    dt = float(m_pc_timestamp - last_timestamp) / 1000;
    LOG(INFO) << "FusionPc----------------------------- dt -----------------------------: " << dt;

    std::vector<FUKalmanBoxTracker> &trackers = m_trackers;
    int trackers_num = trackers.size();
    if (trackers_num > 0){
        for (int i = 0; i < trackers_num; i++)
        {
            trackers[i].predict(dt);
        }
    }
    last_timestamp = m_pc_timestamp;

    // lidar fusion
    if (int(m_stFusionAlgParams.m_fusion_parameter["fusion_param"]["Fusion_Lidar"]) ==  1){
        m_lidar_handler->process_lidar_data(&pc);
        m_lidar_handler->excute_match_lidar_stage_3rd_match(trackers, m_iouassociate_handler, m_disassociate_handler);
        m_lidar_handler->update_type(trackers, m_pc_timestamp);
        m_update_handler->create_tracks(m_match_out, trackers, m_pc_timestamp, 1);
    }

    // update
    xt::xarray<float> fusion_track_result = xt::empty<float>({0, 22});
    m_update_handler->update(trackers, fusion_track_result, m_pc_timestamp, dt);
    m_CommonData->m_fTrackXarrayResult = fusion_track_result;
    m_CommonData->sourceDataType = ModelofFusion;

    setCommonData(m_CommonData);
}