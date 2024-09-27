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
    m_AlgParams = static_cast<CSelfAlgParam >(*p_pAlgParam);
    pcWait = m_AlgParams.m_fusion_parameter["fusion_param"]["PcWaitFrame"];
    pcLast = m_AlgParams.m_fusion_parameter["fusion_param"]["PcLastFrame"];
    // 加载车道角
    // std::pair<float [2], float [2]> limit = FusionFunction::load_chedao(m_chedao);
    // m_xlimit[0] = limit.first[0];
    // m_xlimit[1] = limit.first[1];
    // m_ylimit[0] = limit.second[0];
    // m_ylimit[1] = limit.second[1];

    m_match_out = make_shared<fusion_match_out>();
    m_cal_dis_result = make_shared<cal_dis_result>();
    m_lidar_handler = make_shared<Process_lidar>(m_AlgParams.m_fusion_parameter, m_match_out, m_cal_dis_result);
    m_update_handler =make_shared<Update_tracks>(m_AlgParams.m_fusion_parameter, &m_chedao, m_xlimit, m_ylimit);
    m_camera_handler = make_shared<Process_camera>(m_AlgParams.m_fusion_parameter, m_match_out, m_cal_dis_result);
    m_iouassociate_handler = std::make_shared<IouAssociate>(m_AlgParams.m_fusion_parameter);
    m_disassociate_handler =  std::make_shared<DistanceAssociate>(m_cal_dis_result, m_AlgParams.m_fusion_parameter);
}

void Fusion_Algorithm_Intersection::execute()
{
    m_camera_timestamp.clear();
    auto t_start = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "Fusion_Algorithm_Intersection::execute  status :  started.";
    m_stFusionAlgSrcdata = getCommonData()->FuTrAlgdata;
    time_since_pcinput ++;
    for (int t = 0; t < m_stFusionAlgSrcdata.vecFrameResult().size(); t++)
    {
        if(m_stFusionAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_PC_RESULT)
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
    if(time_since_pcinput == 0 && ModelofFusion == 2 && time_pcinput_last > pcLast){
        LOG(ERROR)<<"---------------------CHANGE PC MODEL---------------------";
        m_trackers.clear();
        m_trackers.shrink_to_fit();
        ModelofFusion = 1;
        last_timestamp = m_pc_timestamp;
        throw FusionException();
    }
    if(time_since_pcinput > 0 && time_since_pcinput < pcWait){
        LOG(ERROR)<<"---------------------NO PCDATA---------------------";
        time_pcinput_last = 0;
        throw FusionException();
    }
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
        FusionOnlyVideo();
    }

    LOG(INFO)<< "Fusion_Algorithm_Intersection::execute  status : finished! ";
    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
}

void Fusion_Algorithm_Intersection::FusionPc()
{
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
    if (int(m_AlgParams.m_fusion_parameter["fusion_param"]["Fusion_Lidar"]) ==  1){
        m_lidar_handler->process_lidar_data(&pc);
        m_lidar_handler->excute_match_lidar_stage_3rd_match(trackers, m_iouassociate_handler, m_disassociate_handler);

        m_lidar_handler->update_type(trackers, m_pc_timestamp);
        m_update_handler->create_tracks(m_match_out, trackers, m_pc_timestamp, 1);
    }

    // camera fusion
    if (int(m_AlgParams.m_fusion_parameter["fusion_param"]["Fusion_Video"]) ==  1){
        for (auto it = video.begin(); it != video.end(); it++){
            m_camera_handler->process_camera_data(&(*it));
            m_camera_handler->excute_match_camera_stage(trackers, m_iouassociate_handler, m_disassociate_handler);

            m_camera_handler->update_type(trackers, m_video_timestamp);
            m_update_handler->create_tracks(m_match_out, trackers, m_video_timestamp, 0);
        }

        m_camera_handler->getCameraBoxInfo(&video);
    }

    // update
    xt::xarray<float> fusion_track_result = xt::empty<float>({0, 22});
    m_update_handler->update(trackers, fusion_track_result, m_pc_timestamp, dt);
    m_CommonData->m_fTrackXarrayResult = fusion_track_result;
    m_CommonData->sourceDataType = ModelofFusion;

    setCommonData(m_CommonData);
}

void Fusion_Algorithm_Intersection::FusionOnlyVideo()
{
    std::vector<xt::xarray<float>> &video = getCommonData()->m_vecVideoXarrayResult;
    // predict
    dt = float(m_video_timestamp - last_timestamp) / 1000;
    LOG(INFO) << "FusionOnlyVideo----------------------------- dt -----------------: " << dt;

    std::vector<FUKalmanBoxTracker> &trackers = m_trackers;
    int trackers_num = trackers.size();
    if (trackers_num > 0){
        for (int i = 0; i < trackers_num; i++)
        {
            trackers[i].predict(dt);
        }
    }
    last_timestamp = m_video_timestamp;
    // camera fusion
    if (int(m_AlgParams.m_fusion_parameter["fusion_param"]["Fusion_Video"]) ==  1){
        for (auto it = video.begin(); it != video.end(); it++){
            m_camera_handler->process_camera_data(&(*it));
            m_camera_handler->excute_match_camera_stage(trackers, m_iouassociate_handler, m_disassociate_handler);

            m_camera_handler->update_onlyvideo(trackers, m_video_timestamp);
            m_update_handler->create_onlyvideotracks(m_match_out, trackers, m_video_timestamp, 0);
        }
        m_camera_handler->getCameraBoxInfo(&video);
    }else{
        LOG(ERROR) << " ONLY VIDEO BUT Fusion_Video != 1";
    }

    // update
    xt::xarray<float> fusion_track_result = xt::empty<float>({0, 22});
    m_update_handler->update_onlyvideo(trackers, fusion_track_result, m_video_timestamp, dt);
    m_CommonData->m_fTrackXarrayResult = fusion_track_result;
    m_CommonData->sourceDataType = ModelofFusion;

    setCommonData(m_CommonData);
}