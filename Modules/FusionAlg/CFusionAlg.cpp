#include "CFusionAlg.h"

std::string	CFusionAlg::GetVersion()
{
    return "AlgLib FusionTrackAlg V1.3";
}

CFusionAlg::CFusionAlg(const std::string& p_strExePath)
{
    m_strOutPath = p_strExePath;
}

CFusionAlg::~CFusionAlg()
{

}

bool CFusionAlg::InitAlgorithm(CSelfAlgParam* p_pAlgParam, const AlgCallback& alg_cb, void* hd)
{
    if (!p_pAlgParam){
        LOG(ERROR) << "CFusionAlg::InitAlgorithm ---- End >>> Failed : The incoming parameter is empty.";
        return false;
    }

    m_stSelfFuTrAlgParam = *p_pAlgParam;

    // 1.回调函数设置
    futr_callback = alg_cb;
    futr_hd = hd;

    // 2. 算法选择
    std::string s_AlgStratParamsFileName = m_strOutPath + "/Configs/Alg/Alg.yaml";
    p_makeAlgs = std::make_shared<CMakeFuTrAlgs_strategy>(s_AlgStratParamsFileName);
    p_makeAlgs->makeAlgs();
    p_CompositeAlgs = std::make_shared<CCompositeAlgs>(p_makeAlgs);

    // 3. 融合算法参数设置
    std::string filepath = m_strOutPath + "/Configs/Alg/fusion/fusion_param.json";
    bool isload = temp_paramgr.jsonload(filepath);
    if(!isload)
    {
        LOG(ERROR) << "DOWNLOAD fusion_param.json FAILED!!!";
    }
    bool flag = temp_paramgr.get(temp_param);
    if(!flag)
    {
        LOG(ERROR) << "GET fusion_param FAILED!!!";
    }
    m_stSelfFuTrAlgParam.m_fusion_parameter = temp_param;

    // 4.初始化
    FuTrdata = std::make_shared<ICommonData>();
    p_CompositeAlgs->setCommonData(FuTrdata);
    p_CompositeAlgs->init(&m_stSelfFuTrAlgParam);
    FuTrdata = p_CompositeAlgs->getCommonData();

    if(m_pRunThread)
    {
        m_pRunThread->join();
        m_pRunThread.reset();
    }

    m_pRunThread = std::make_unique<std::thread>(&CFusionAlg::RunFusionTrack, this);
    startTime = std::chrono::steady_clock::now();
    LOG(INFO) << "CFusionAlg::InitAlgorithm ---- End >>> Succeeded";
    return true;
}

void CFusionAlg::RunFusionTrack()
{
    while (1)
    {
        std::shared_ptr<CAlgResult> l_pFusionTrackSrcdata;
        if(!m_dqAlgResultQueue.PopFront(l_pFusionTrackSrcdata, 100))
        {
            continue;
        }
        LOG(INFO) << "CFusionAlg::RunFusionTrack ---- status : start.";
        FuTrdata->FuTrAlgdata = *l_pFusionTrackSrcdata;
        p_CompositeAlgs->setCommonData(FuTrdata);

        auto t_start = std::chrono::high_resolution_clock::now();
        m_stFuTrAlgResult.mapTimeStamp()[TIMESTAMP_FUSION_ALG_BEGIN] = t_start.time_since_epoch().count()/1000000;

        try{
            // 2. 融合算法执行
            p_CompositeAlgs->execute();

            // 3. 融合跟踪算法处理结果获取
            m_stFuTrAlgResult = p_CompositeAlgs->getCommonData()->FuTrAlgdata;
        }
        catch(const std::exception & e){
            LOG(ERROR) << e.what() << '\n';
            continue;
        }

        // 4. 计算融合耗时
        auto t_end = std::chrono::high_resolution_clock::now();
        m_stFuTrAlgResult.mapTimeStamp()[TIMESTAMP_FUSION_ALG_END] = t_end.time_since_epoch().count()/1000000;
        auto latency_alg = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
        LOG(INFO) <<"RunAlgorithm ---- End >>> Alg Time : " << latency_alg << " ms.";
        m_stFuTrAlgResult.mapDelay()[DELAY_TYPE_FUSION_ALG]= latency_alg;

        if(m_stFuTrAlgResult.eDataSourceType() == DATA_TYPE_PC_RESULT){
            auto latency_all = (t_end.time_since_epoch().count()/1000000 - m_stFuTrAlgResult.vecFrameResult()[0].mapTimeStamp()[TIMESTAMP_PC_FIRST_PACKET]);
            m_stFuTrAlgResult.mapDelay()[DELAY_TYPE_FUSION_ALL]= latency_all;
        }
        if(m_stFuTrAlgResult.eDataSourceType() == DATA_TYPE_VIDEO_RESULT){
            auto latency_all = (t_end.time_since_epoch().count()/1000000 - m_stFuTrAlgResult.vecFrameResult()[0].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB]);
            m_stFuTrAlgResult.mapDelay()[DELAY_TYPE_FUSION_ALL]= latency_all;
        }

        // 5. 通过回调函数返回结果
        futr_callback(m_stFuTrAlgResult, futr_hd);
        outputFrameCount ++;
        LOG(INFO) << "CFusionAlg::RunAlgorithm ---- status : over.";
    }
}

void CFusionAlg::RunAlgorithm(void* p_pSrcData)
{
    LOG(INFO) << "CFusionAlg::RunAlgorithm ---- start." ;
    if (!p_pSrcData)
    {
        LOG(ERROR) << "CFusionAlg::RunAlgorithm ---- No Fusion input data.";
        return;
    }

    // 1. 融合输入数据
    CAlgResult l_pFuTrAlgSrcdata = *(static_cast<CAlgResult *>(p_pSrcData));

    for (int t = 0; t < l_pFuTrAlgSrcdata.vecFrameResult().size(); t++)
    {
        if(l_pFuTrAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_PC_RESULT)
        {
            nowpc_timestamp = l_pFuTrAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
        }
        else if(l_pFuTrAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_VIDEO_RESULT)
        {
            nowvideo_timestamp = l_pFuTrAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
        }
    }
    now_timestamp = nowpc_timestamp;
    if(nowpc_timestamp == lastpc_timestamp){
        now_timestamp = nowvideo_timestamp;
        LOG(INFO) << "<<<<<<ONLY VIDEO DATA>>>>>>";
    }
    lastpc_timestamp = nowpc_timestamp;
    dt = float(now_timestamp - last_timestamp) / 1000;
    if(dt <= 0)
    {
        LOG(ERROR) << "<<<<<<dt ERROR>>>>>>dt:" << dt << " last_timestamp: " << last_timestamp << " now_timestamp: " << now_timestamp;
        last_timestamp = now_timestamp;
        return;
    }
    last_timestamp = now_timestamp;
    m_dqAlgResultQueue.PushBack(std::make_shared<CAlgResult>(l_pFuTrAlgSrcdata));
    inputFrameCount ++;

    auto currentTime = std::chrono::steady_clock::now();
    auto lastTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime).count();
    if(lastTime >= frameRateInterval)
    {
        inputFrameRate = static_cast<double> (inputFrameCount) / static_cast<double>(lastTime);
        LOG(INFO) << "<<<<<<FUSIONALG INPUT FRAMERATE : >>>>>>" << inputFrameRate;
        outputFrameRate = static_cast<double> (outputFrameCount) / static_cast<double>(lastTime);
        LOG(INFO) << "<<<<<<FUSIONALG OUTPUT FRAMERATE : >>>>>>" << outputFrameRate;
        inputFrameCount = 0;
        outputFrameCount = 0;
        startTime = currentTime;
    }
} 




