#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>

#include <xtensor/xbuilder.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xtensor.hpp>
#include "xtensor/xview.hpp"
#include "xtensor/xarray.hpp"
#include "ICommonAlg.h"
#include "CSelfAlgParam.h"
#include "GlobalContext.h"
#include <experimental/filesystem>

using namespace std;

class CFusionPreprocess : public ICommonAlg{ // ,IFusionAlg{
    public:
        CFusionPreprocess();
        virtual ~CFusionPreprocess();

        void init(CSelfAlgParam* p_pAlgParam );
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
        xt::xarray<float> PcAlgResTransfer(CFrameResult *p_pPcResult);

        xt::xarray<float> VideoAlgResTransfer(CFrameResult *p_pVideoResult, int channel);

        int PcClass2label( std::string &p_strClass, nlohmann::json fusion_parameter);

        int VideoClass2label( std::string &p_strClass, nlohmann::json fusion_parameter);

        void save_input_to_npy(std::string path, CAlgResult outputAlgResult);

        // private parameters
        xt::xarray<int> m_camera_reflect_limit;     // 相机反映射限制
        CSelfAlgParam  p_PreFuTrAlgParam;           // 融合跟踪预处理参数
        CCommonDataPtr m_CommonData;

};