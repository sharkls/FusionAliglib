/*******************************************************
 文件名：Video_detection_preprocess.h
 作者：
 描述：视频算法接口实现，用于视频算法的运行及结果数据处理
 版本：v1.0
 日期：2022-10-19
 *******************************************************/

#pragma once

#include "ExportVideoAlgLib.h"
#include "CAlgResult.h"
#include "CVideoSrcData.h"
#include "CSelfAlgParam.h"
#include "CCameraParam.h"
#include <iostream>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ICommonAlg.h"

class CVPre : public ICommonAlg
{
public:
    CVPre();

    CVPre(CSelfAlgParam* p_pAlgParam);

    virtual ~CVPre();

    // 初始化预处理部分参数
    void init(CSelfAlgParam * p_pAlgParam);

    // 执行数据预处理算法
    void execute();

    // xt::xarray<float> pc_execute(xt::xarray<float> &points_xt){ return points_xt;};

    // Resize 操作
    std::vector<cv::Mat> Resize(CVideoSrcDataTimematch * p_pSrcData);

    void setCommonData(CCommonDataPtr p_commonData) override
    {
        m_CommonData = p_commonData;
    }

    CCommonDataPtr getCommonData() override
    {
        return m_CommonData;
    }

private:

};

