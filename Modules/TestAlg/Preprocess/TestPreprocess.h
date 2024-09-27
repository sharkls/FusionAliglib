/*******************************************************
 文件名：TestPreProcess.h
 作者：
 描述：算法接口实现，用于算法预处理的运行及结果数据处理
 版本：v1.0
 日期：2024-1-11
 *******************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include "ICommonAlg.h"
#include "CAlgResult.h"
#include "CSelfAlgParam.h"

class TestPreProcess : public ICommonAlg
{
public:
    TestPreProcess();

    TestPreProcess(const CSelfAlgParam* p_pAlgParam);

    virtual ~TestPreProcess();

    // 初始化预处理部分中参数
    void init(CSelfAlgParam* p_pAlgParam);

    // 执行预处理算法
    void execute();

    
    void setCommonData(CCommonDataPtr p_commonData) override
    {
        m_CommonData = p_commonData;
    }

    CCommonDataPtr getCommonData() override
    {
        return m_CommonData;
    }

private:
    // CAlgResult * TestPreProcessResult;
};

