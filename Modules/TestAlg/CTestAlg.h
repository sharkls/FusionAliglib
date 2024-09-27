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

    //初始化算法接口对象，内部主要处理只需初始化一次的操作，比如模型加载之类的，成功返回true，失败返回false
    bool InitAlgorithm(CSelfAlgParam* p_pAlgParam, const AlgCallback & alg_cb, void * hd);

    //执行算法函数，传入原始数据体，算法执行成功通过回调函数将结果返回    
    void RunAlgorithm(void* p_pSrcData);

    std::string	GetVersion();

    int64_t GetTimeStamp();

    //  回调函数
    AlgCallback test_callback;
    void *test_hd;

    IMakeAlgPtr p_makeAlgs;
    CCompositeAlgsPtr p_CompositeAlgs;
    
    std::string m_strOutPath;               // Output路径（构造时根据该路径下的配置文件进行构建）
    CAlgResult m_stTestAlgResult;           // 回调函数返回数据
    CSelfAlgParam m_stSelfTestAlgParam;     // 测试算法参数

    CVideoSrcDataTimematch l_pTestSrcData;
    // CCommonDataPtr test_Srcdata;
};