/*******************************************************
 文件名：CTestAlg.h
 作者：
 描述：视频算法接口实现，用于视频算法的运行及结果数据处理
 版本：v1.0
 日期：2022-10-19
 *******************************************************/

#pragma once


#include "ExportTestAlgLib.h"
#include "CAlgResult.h"
#include "CVideoSrcData.h"
#include <iostream>
#include <vector>
#include <queue>
#include <thread>

#include <fstream>
#include "IMakeAlg.h"
#include "ICommonAlg.h"
#include "CMakeAlgs.h"
#include "CCompositeAlgs.h"
#include "GlobalContext.h"
#include "CSafeDataDeque.h"

class CTestAlg :public ITestAlg
{
public:
    CTestAlg(const std::string& p_strExePath);
    virtual ~CTestAlg();

    //设置初始化以及执行算法需要的各类参数，成功返回true，失败返回false
    // virtual bool  SetAlgParam(const CSelfVideoAlgParam* p_pAlgParam) override;

    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    bool InitAlgorithm(CSelfAlgParam* p_pAlgParam, const AlgCallback & alg_cb, void * hd);

    //执行算法函数，传入原始数据体，算法执行成功返回处理后的数据或者检测结果（由算法类型而定），失败返回nullptr
    //参数1：数据源，目前约定传入 TPcSrcData* 类型数据 
    //参数2：点云算法类型，默认 PC_DETECT_AND_TRACK
    //返回值：算法处理后的数据空间地址，根据算法类型不同有些差异，具体如下：
    //PC_PRETREATMENT类型：返回 TPcSrcData* 类型数据 
    //PC_DETECT类型：返回 TPcResult* 类型数据 
    //PC_DETECT_AND_TRACK类型：返回 TPcResult* 类型数据 
    void RunAlgorithm(void* p_pSrcData);

    std::string	GetVersion();
    int64_t GetTimeStamp();

    //  回调函数
    AlgCallback test_callback;
    void *test_hd;

    
    IMakeAlgPtr p_makeAlgs;
    CCompositeAlgsPtr p_CompositeAlgs;
    
    std::string m_strOutPath;
    CAlgResult m_stTestAlgResult;
    CSelfAlgParam m_stSelfTestAlgParam;

    CVideoSrcDataTimematch l_pTestSrcData;
    CCommonDataPtr test_Srcdata;
};