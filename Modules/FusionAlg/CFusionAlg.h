/*******************************************************
 文件名：CFusionAlg.h
 作者：
 描述：融合算法接口实现，用于融合算法的运行及结果数据处理
 版本：v1.0
 日期：2020-10-22
 *******************************************************/

#pragma once

#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include <cstdio>
#include "ExportFusionAlgLib.h"
#include "Log/glog/logging.h"
#include "CommonFunctions.h"
#include "CAlgResult.h"
#include "GlobalContext.h"
#include "ParaMgr.h"
#include "CCompositeAlgs.h"
#include "CSelfAlgParam.h"
#include "CSafeDataDeque.h"

class CFusionAlg :public IFusionAlg
{
public:
    CFusionAlg(const std::string& p_strOutPath);
    virtual ~CFusionAlg();

    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    virtual bool  InitAlgorithm(CSelfAlgParam* p_pAlgParam, const AlgCallback& alg_cb, void* hd) override;

    //执行算法函数，传入融合前的结果，算法执行成功返回融合后的结果（由算法类型而定），失败返回nullptr
    virtual void RunAlgorithm(void* p_pSrcData) override;

    void RunFusionTrack();

    std::string	GetVersion();

private:
    std::string m_strOutPath;
    CSelfAlgParam m_stSelfFuTrAlgParam;
    AlgCallback futr_callback;
    void* futr_hd;
    IMakeAlgPtr p_makeAlgs;
    ICommonAlgPtr p_CompositeAlgs;
    CCommonDataPtr FuTrdata;
    CSafeDataDeque<std::shared_ptr<CAlgResult>> m_dqAlgResultQueue;
    std::unique_ptr<std::thread> m_pRunThread;
    CAlgResult m_stFuTrAlgResult;           // 融合跟踪模块输出数据

    ParaMgr temp_paramgr;
    nlohmann::json temp_param;              // 融合跟踪配置参数
    unsigned long last_timestamp = 0;
    unsigned long now_timestamp = 0;
    unsigned long nowpc_timestamp = 0;
    unsigned long nowvideo_timestamp = 0;
    unsigned long lastpc_timestamp = 0;
    float dt;

    const int frameRateInterval = 1;
    int inputFrameCount = 0;
    int outputFrameCount = 0;
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    double inputFrameRate;
    double outputFrameRate;
};

