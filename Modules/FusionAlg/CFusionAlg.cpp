#include "CFusionAlg.h"

std::string	CFusionAlg::GetVersion()
{
    return "AlgLib FusionAlg V1.3";
}

CFusionAlg::CFusionAlg(const std::string& p_strExePath)
{
    m_strOutPath = p_strExePath;
}

CFusionAlg::~CFusionAlg()
{

}

bool CFusionAlg::InitAlgorithm(CSelfAlgParam* l_pAlgParam, const AlgCallback& alg_cb, void* hd)
{
    if (!l_pAlgParam){
        LOG(ERROR) << "CFusionAlg::InitAlgorithm ---- End >>> Failed : The incoming parameter is empty.";
        return false;
    }

    m_stSelfFusionAlgParam = *l_pAlgParam;

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
        LOG(ERROR) << "LOAD fusion_param.json FAILED!!!";
    }
    bool flag = temp_paramgr.get(temp_param);
    if(!flag)
    {
        LOG(ERROR) << "GET fusion_param FAILED!!!";
    }
    m_stSelfFusionAlgParam.m_fusion_parameter = temp_param;

    // 4.初始化
    FuTrdata = std::make_shared<ICommonData>();
    p_CompositeAlgs->setCommonData(FuTrdata);
    p_CompositeAlgs->init(&m_stSelfFusionAlgParam);
    FuTrdata = p_CompositeAlgs->getCommonData();

    if(m_pRunThread)
    {
        m_pRunThread->join();
        m_pRunThread.reset();
    }

    m_pRunThread = std::make_unique<std::thread>(&CFusionAlg::RunFusion, this);
    startTime = std::chrono::steady_clock::now();
    LOG(INFO) << "CFusionAlg::InitAlgorithm ---- End >>> Succeeded";
    return true;
}

void CFusionAlg::RunFusion()
{
    while (1)
    {   
        // 1. 获取融合数据
        std::shared_ptr<CAlgResult> l_pFusionSrcdata;
        if(!m_dqAlgResultQueue.PopFront(l_pFusionSrcdata, 100))
        {
            continue;
        }
        LOG(INFO) << "CFusionAlg::RunFusion ---- status : start.";

        // 2. 将融合原始数据传输至融合算法内部
        FuTrdata->FuTrAlgdata = *l_pFusionSrcdata;
        p_CompositeAlgs->setCommonData(FuTrdata);

        auto t_start = std::chrono::high_resolution_clock::now();
        m_stFuTrAlgResult.mapTimeStamp()[TIMESTAMP_FUSION_ALG_BEGIN] = t_start.time_since_epoch().count()/1000000;

        try{
            // 3.1. 融合算法执行
            p_CompositeAlgs->execute();

            // 3.2 融合跟踪算法处理结果获取
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
    CAlgResult l_pFuAlgSrcdata = *(static_cast<CAlgResult *>(p_pSrcData));

    for (int t = 0; t < l_pFuAlgSrcdata.vecFrameResult().size(); t++)
    {
        if(l_pFuAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_PC_RESULT)      // 点云数据
        {
            nowpc_timestamp = l_pFuAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
        }
        else if(l_pFuAlgSrcdata.vecFrameResult()[t].eDataType() == DATA_TYPE_VIDEO_RESULT)  // 视频数据
        {
            nowvideo_timestamp = l_pFuAlgSrcdata.vecFrameResult()[t].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];
        }
    }
    
    // 2. 更新时间戳信息
    if(nowpc_timestamp == lastpc_timestamp){        
        now_timestamp = nowvideo_timestamp;
        LOG(INFO) << "<<<<<<ONLY VIDEO DATA>>>>>>";
    }
    else
    {
        now_timestamp = nowpc_timestamp;
    }
    lastpc_timestamp = nowpc_timestamp;

    // 3. 计算帧间时间差
    dt = float(now_timestamp - last_timestamp) / 1000;  
    if(dt <= 0)
    {
        LOG(ERROR) << "<<<<<<dt ERROR>>>>>>dt:" << dt << " last_timestamp: " << last_timestamp << " now_timestamp: " << now_timestamp;
        last_timestamp = now_timestamp;
        return;
    }
    last_timestamp = now_timestamp;

    // 4. 融合原始数据添加至安全队列
    m_dqAlgResultQueue.PushBack(std::make_shared<CAlgResult>(l_pFuAlgSrcdata));
    inputFrameCount ++;

    // 5. 统计帧率
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
    LOG(INFO) << "CFusionAlg::RunAlgorithm ---- end." ;
} 




