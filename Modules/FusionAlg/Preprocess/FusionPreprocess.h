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

#include <opencv2/opencv.hpp>

#include "FunctionHub.h"
#include "hungarian_bigraph_matcher.h"

#include "ICommonAlg.h"
#include "CSelfAlgParam.h"
#include "GlobalContext.h"
#include "Log/glog/logging.h"
#include <experimental/filesystem>

using namespace std;

class CFusionPreprocess : public ICommonAlg{
    public:
        CFusionPreprocess();

        virtual ~CFusionPreprocess();

        void init(CSelfAlgParam* p_pAlgParam );

        void execute();

        // 数据写入
        void setCommonData(CCommonDataPtr p_commonData) override
        {
            m_CommonData = p_commonData;
        }
        // 数据获取
        CCommonDataPtr getCommonData() override
        {
            return m_CommonData;
        }

    private:
        // 点云检测结果转换
        void PcAlgResTransfer(CFrameResult *p_pPcResult, xt::xarray<float>& PcResult);

        xt::xarray<float> PcToVideoTransform(xt::xarray<float>& pc, std::vector<xt::xarray<float>> vecVideoResult, size_t index_cam);

        void MatchAndFuseTargets(xt::xarray<float>& pc, std::vector<xt::xarray<float>> vecVideoResult, size_t index_cam);
        // 视频检测结果转换
        xt::xarray<float> VideoAlgResTransfer(CFrameResult *p_pVideoResult, int channel);

        int PcClass2label( std::string &p_strClass, nlohmann::json fusion_parameter);

        int VideoClass2label( std::string &p_strClass, nlohmann::json fusion_parameter);

        void save_input_to_npy(std::string path, CAlgResult outputAlgResult);

        // void loadCameraParams(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec);

        // private parameters
        size_t  cam_num_;
        size_t  img_Width;                     // 图像宽度             
        size_t  img_High;                      // 图像高度
        vector<cv::Mat> m_vecCameraMatrix;     // 相机内参
        vector<cv::Mat> m_vecDistCoeffs;       // 畸变系数
        vector<cv::Mat> m_vecRotation;         // 旋转矩阵
        vector<cv::Mat> m_vecTranslation;      // 平移矩阵
        CCameraParam m_CameraParam;            // 相机参数
        

        xt::xarray<float>  m_PcResult;
        std::vector<xt::xarray<float>> m_vecVideoResult;

        xt::xarray<int> m_camera_reflect_limit;             // 相机反映射限制
        CSelfAlgParam  p_PreFuTrAlgParam;                   // 融合预处理参数
        CCommonDataPtr m_CommonData;

};