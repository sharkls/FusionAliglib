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
#include "CGeneralTrans.h"
#include <experimental/filesystem>

using namespace std;

class CFusionPostprocess : public ICommonAlg, CGeneralTrans{
    public:
        CFusionPostprocess();
        virtual ~CFusionPostprocess();

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
        void setFusionResult4Scene(CFrameResult *FuTrackResult, CSelfAlgParam p_pAlgParam, const xt::xarray<float> &result);

        void save_fusion_result_to_csv(std::string path, CAlgResult outputAlgResult);


    // private parameters
        CSelfAlgParam  p_PostFuTrAlgParam;              // 融合跟踪后处理参数
        CCommonDataPtr m_CommonData;
        double _dLidarLon;
        double _dLidarLat;
        float _fLidarNorthAngle;

};