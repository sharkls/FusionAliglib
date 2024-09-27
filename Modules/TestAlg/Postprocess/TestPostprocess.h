/*******************************************************
 文件名：TestPostprocess.h
 作者：
 描述：算法接口实现，用于算法后处理的运行及结果数据处理
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

class TestPostProcess : public ICommonAlg
{
public:
    TestPostProcess();

    TestPostProcess(const CSelfAlgParam* p_pAlgParam);

    virtual ~TestPostProcess();

    // 初始化后处理部分中参数
    void init(CSelfAlgParam* p_pAlgParam);

    // 执行后处理算法
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
    // CAlgResult * TestPostProcessResult;
};

