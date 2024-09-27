#include "FusionPostprocess.h"
#include "Log/glog/logging.h"

namespace fs = std::experimental::filesystem;

CFusionPostprocess::CFusionPostprocess()
{
    LOG(INFO) << "Fusion_Tracking data Postprocess status :   start." << std::endl;
}


CFusionPostprocess::~CFusionPostprocess()
{

}

void CFusionPostprocess::init(CSelfAlgParam* p_pAlgParam)
{
    LOG(INFO) << "CFusionPostprocess::init status :   start";

    // 1. 预处理参数获取
    if(!p_pAlgParam){
        LOG(ERROR) << "CFusionPostprocess::init  ERROR:  CSelfAlgParam is empty";
    }
    p_PostFuTrAlgParam = *p_pAlgParam;
    if(p_PostFuTrAlgParam.m_tLidarParam.vecLidarDev().size() <= 0)
    {
        LOG(ERROR) << "PLEASE FILL IN LidarParam";
        return;
    }
    _dLidarLon = p_PostFuTrAlgParam.m_tLidarParam.vecLidarDev()[0].dLidarLon();
    _dLidarLat = p_PostFuTrAlgParam.m_tLidarParam.vecLidarDev()[0].dLidarLat();
    _fLidarNorthAngle = p_PostFuTrAlgParam.m_tLidarParam.vecLidarDev()[0].fLidarNorthAngle();
    
    LOG(INFO) << "_dLidarLon:" << _dLidarLon;
    LOG(INFO) << "_dLidarLat:" << _dLidarLat;
    LOG(INFO) << "_fLidarNorthAngle:" << _fLidarNorthAngle;

    LOG(INFO) << "CFusionPostprocess::init status :   finish!";
}

void CFusionPostprocess::execute()
{
    auto t_start = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "CFusionPostprocess::execute  status :  started.";

    // 1. 融合跟踪后处理数据获取
    CAlgResult FuTrAlgSrcdata = getCommonData()->FuTrAlgdata;
    xt::xarray<float> pc =  getCommonData()->m_fTrackXarrayResult;

    // 2. 融合跟踪后处理输出数据初始化
    CAlgResult FuTralgResult;
    CFrameResult PcResult;
    for (int i = 0 ; i < FuTrAlgSrcdata.vecFrameResult().size(); i++){
        if(m_CommonData->sourceDataType == 1){
            if (FuTrAlgSrcdata.vecFrameResult()[i].eDataType() == DATA_TYPE_PC_RESULT){                 // 点云数据信息获取并赋值给融合结果
                // 雷达附录信息
                PcResult.tLidarSupplement().dLidarLon() = FuTrAlgSrcdata.vecFrameResult()[i].tLidarSupplement().dLidarLon();
                PcResult.tLidarSupplement().dLidarLat() = FuTrAlgSrcdata.vecFrameResult()[i].tLidarSupplement().dLidarLat();
                PcResult.tLidarSupplement().fLidarNorthAngle() = FuTrAlgSrcdata.vecFrameResult()[i].tLidarSupplement().fLidarNorthAngle();
                PcResult.tLidarSupplement().ucLidarId(FuTrAlgSrcdata.vecFrameResult()[i].tLidarSupplement().ucLidarId());
                PcResult.unFrameId(FuTrAlgSrcdata.vecFrameResult()[i].unFrameId());
                PcResult.mapTimeStamp()[TIMESTAMP_PC_FIRST_PACKET] = FuTrAlgSrcdata.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PC_FIRST_PACKET];
                PcResult.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] = FuTrAlgSrcdata.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB]; // yyl- 增加时间戳为融合匹配使用
                PcResult.mapTimeStamp()[TIMESTAMP_PC_SUB] = FuTrAlgSrcdata.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PC_SUB];  //订阅单路点云数据包结构时的时间戳
                PcResult.mapTimeStamp()[TIMESTAMP_PC_FRAME] = FuTrAlgSrcdata.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PC_FRAME];  //点云pcap数据包组成一帧完整点云时的本机时间戳
                PcResult.mapFps()[FPS_PC_MATCH_INPUT] = FuTrAlgSrcdata.vecFrameResult()[i].mapFps()[FPS_PC_MATCH_INPUT];

                PcResult.mapDelay(FuTrAlgSrcdata.vecFrameResult()[i].mapDelay());                       // 耗时
            }
        }
        if(m_CommonData->sourceDataType == 2){
            if (FuTrAlgSrcdata.vecFrameResult()[i].eDataType() == DATA_TYPE_VIDEO_RESULT){                 // 点云数据信息获取并赋值给融合结果
                PcResult.tLidarSupplement().dLidarLon() = _dLidarLon;
                PcResult.tLidarSupplement().dLidarLat() = _dLidarLat;
                PcResult.tLidarSupplement().fLidarNorthAngle() = _fLidarNorthAngle;
                PcResult.tLidarSupplement().ucLidarId(0);
                PcResult.unFrameId(FuTrAlgSrcdata.vecFrameResult()[i].unFrameId());
                PcResult.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] = FuTrAlgSrcdata.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB]; // yyl- 增加时间戳为融合匹配使用
                PcResult.mapDelay(FuTrAlgSrcdata.vecFrameResult()[i].mapDelay());                       // 耗时
            }
        }
    }
    // 3. 遍历所有帧数据，构建点云结果数据和图像结果数据
    setFusionResult4Scene(&PcResult, p_PostFuTrAlgParam, pc);
    PcResult.eDataType(DATA_TYPE_TRACK_RESULT);
    FuTralgResult.vecFrameResult().push_back(PcResult);
    // SAVEDATA
    if(p_PostFuTrAlgParam.m_fusion_parameter["fusion_param"]["save_resultdata_flag"]){
        std::string l_sOutPutPath = p_PostFuTrAlgParam.m_strRootPath + "/SaveData/";
        save_fusion_result_to_csv(l_sOutPutPath, FuTralgResult);
    }

    // 4. 融合跟踪预处理数据输出
    m_CommonData->FuTrAlgdata = FuTralgResult;
    if(m_CommonData->sourceDataType == 1){
        m_CommonData->FuTrAlgdata.eDataSourceType(DATA_TYPE_PC_RESULT);
    }
    if(m_CommonData->sourceDataType == 2){
        m_CommonData->FuTrAlgdata.eDataSourceType(DATA_TYPE_VIDEO_RESULT);
    }
    setCommonData(m_CommonData);
    LOG(INFO) << "CFusionPostprocess::execute  status :  finish!";

    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_all = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
}

void CFusionPostprocess::setFusionResult4Scene(CFrameResult *FuTrackResult, CSelfAlgParam p_pAlgParam, const xt::xarray<float> &result)
{
    // 把融合跟踪结果装入结构体中
    LOG(INFO) << "TrackResult.nums:" << result.shape(0);
    if (result.shape(0) > 0)
    {
        // 跟踪目标信息获取
        for (int i = 0; i < result.shape(0); ++i)
        {
            // [0, 1, 2, 3,   4,   5, 6,   7,     8,    9,   10,     11,    12,    13,    14,     15,         16,     17,    18                19,         20,        21 ]
            // [x, y, w, l, theta, z, h, label, speed, id, scores, x_min, y_min, x_max, y_max, data_source, channel, lane, occur_time, time_sicne_update, hits, flag_delete_track]
            CObjectResult fu_cPcBoxInfo;
            fu_cPcBoxInfo.sXCoord(result(i, 0) * 100);                  // X轴坐标:m    0
            fu_cPcBoxInfo.sYCoord(result(i, 1) * 100);                  // Y轴坐标:m    1
            fu_cPcBoxInfo.usWidth(result(i, 2) * 100);                  // 宽度         2
            fu_cPcBoxInfo.usLength(result(i, 3) * 100);                 // 长度         3
            fu_cPcBoxInfo.sCourseAngle(result(i, 4));                   // 航向角 角度   4
            fu_cPcBoxInfo.sZCoord(result(i, 5) * 100);                  // Z轴坐标:m    5
            fu_cPcBoxInfo.usHeight(result(i, 6) * 100);                 // 高度         6
            fu_cPcBoxInfo.strClass(p_pAlgParam.m_fusion_parameter["fusion_param"]["m_strFusionClass"][int(result(i, 7))]); // 目标类型(点云算法传出的类型与视频传出的类型映射关系不同,所以此处采用字符串形式统一)
            fu_cPcBoxInfo.sSpeed(result(i, 8) * 100);                   // 速度：m/s
            fu_cPcBoxInfo.usTargetId(result(i, 9));                     // 单基站检测的ID
            fu_cPcBoxInfo.fPcConfidence(float(result(i, 10) * 100));      // 置信度
            fu_cPcBoxInfo.fBottomRightX((float)result(i, 11));          // x_min
            fu_cPcBoxInfo.fBottomRightY((float)result(i, 12));          // y_min
            fu_cPcBoxInfo.fTopLeftX((float)result(i, 13));              // x_max
            fu_cPcBoxInfo.fTopLeftY((float)result(i, 14));              // y_max
            fu_cPcBoxInfo.ucSource((unsigned char)(result(i, 15)));     // data_source
            fu_cPcBoxInfo.usVideoInfo()[4] = (unsigned char)(result(i, 16)); // channel
            fu_cPcBoxInfo.usLaneNum((uint32_t)result(i, 17));           // lane
            fu_cPcBoxInfo.ulFirstOccurTime(float(result(i, 18)));       // occur_time
            fu_cPcBoxInfo.uiTimeSinceUpdate(uint32_t(result(i, 19)));   // time_since_update
            fu_cPcBoxInfo.usHitNum(uint64_t(result(i, 20)));            // hits

            // 是否删除轨迹
            if (result(i, 21) == 0){
                fu_cPcBoxInfo.bDeleteFlag(0);
            }else{
                fu_cPcBoxInfo.bDeleteFlag(1);
            }
            // XYZ 转 经纬度
            double l_dBoxLongitude = 0, l_dBoxLatitude=0;
            XYZ_To_BLH( _dLidarLon, _dLidarLat, result(i,0), result(i, 1), _fLidarNorthAngle, l_dBoxLongitude, l_dBoxLatitude);
            fu_cPcBoxInfo.dLon(l_dBoxLongitude);
            fu_cPcBoxInfo.dLat(l_dBoxLatitude);

            FuTrackResult->vecObjectResult().push_back(fu_cPcBoxInfo);
        }
    }
    FuTrackResult->eDataSourceType(DATA_TYPE_TRACK_RESULT);
}

void CFusionPostprocess::save_fusion_result_to_csv(std::string path, CAlgResult outputAlgResult){
    for(int i = 0 ; i < outputAlgResult.vecFrameResult().size(); i++){
        std::ofstream file;
        char frameNum[7];
        snprintf(frameNum, sizeof(frameNum), "%06d", outputAlgResult.vecFrameResult()[i].unFrameId());
        std::string file_path = path + "FusionResults/Frame_" + std::to_string(outputAlgResult.vecFrameResult()[0].unFrameId()) +".csv";
        fs::path pathObj = (file_path);
        if(!fs::exists(pathObj.parent_path()))
        {
            fs::create_directories(pathObj.parent_path());
        }
        file.open(file_path);
        // 检查文件是否成功打开
        if (!file.is_open()) {
            std::cout << "无法打开文件" << file_path << std::endl;
        }
        else
        {
            for (int j = 0; j < outputAlgResult.vecFrameResult()[i].vecObjectResult().size(); j++)
            {
                int ID = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usTargetId();
                float x = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sXCoord()/100.0;

                float y = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sYCoord()/100.0 ;
                float z = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sZCoord()/100.0 ;
                float w = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usWidth()/100.0 ;
                float l = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usLength()/100.0 ;
                float h = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usHeight()/100.0 ;
                float s = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sSpeed()/100.0 ;
                float a = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].sCourseAngle() ;
                std::string cls1 = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].strClass() ;
                float cls = 0;
                for(int i = 0; i < p_PostFuTrAlgParam.m_fusion_parameter["fusion_param"]["m_strFusionClass"].size(); i++)
                {
                    if(cls1 == p_PostFuTrAlgParam.m_fusion_parameter["fusion_param"]["m_strFusionClass"][int(i)])
                    {
                        cls = i;
                    }
                }

                int src = int(outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].ucSource()) ;
                double dlon1 = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].dLon() ;
                double dlat1 =outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].dLat() ;
                float Conf = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fPcConfidence() ;
                int channel = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usVideoInfo()[4];
                float ltx = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fTopLeftX();
                float lty = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fTopLeftY();
                float rbx = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fBottomRightX();
                float rby = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].fBottomRightY();
                unsigned long long ti = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].ulFirstOccurTime();
                int lane = outputAlgResult.vecFrameResult()[i].vecObjectResult()[j].usLaneNum();
                long long timestamp = outputAlgResult.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PC_FIRST_PACKET];

                file << cls << "," << src << ","  << Conf << "," << x << "," << y << "," << z << "," << l << "," << w << "," << h << "," << a << "," << ltx << "," << lty << "," << rbx << "," << rby << "," << ID << "," << lane << "," << s << "," << ti << "," << channel << "," << timestamp << endl;
            }
        }
        file.close();
    }
}