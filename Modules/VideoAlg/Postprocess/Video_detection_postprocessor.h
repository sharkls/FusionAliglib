/*******************************************************
 文件名：Video_detection_preprocess.h
 作者：
 描述：视频算法接口实现，用于视频算法的运行及结果数据处理
 版本：v1.0
 日期：2022-10-19
 *******************************************************/

#pragma once

// #include "IVideoAlg.h"
#include "ExportVideoAlgLib.h"
#include "CAlgResult.h"
#include "CVideoSrcData.h"
#include "CSelfAlgParam.h"
#include "CCameraParam.h"
#include "CGeneralTrans.h"
#include "cuda_utils_v5.h"
#include "postprocess_v5.h"
#include "utils_v5.h"
#include "config_v5.h"
#include "yolov5.h"
#include "CsvToXtensor.h"
#include <iostream>
#include <vector>
#include <xtensor/xarray.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor-blas/xlapack.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "ICommonAlg.h"
// #include "SpillsDetectAlg.h"

#define PI 3.1415926

class CVPost : public ICommonAlg
{
public:
    CVPost();

    CVPost(const CSelfAlgParam* p_pAlgParam);

    virtual ~CVPost();

    // 初始化视频检测预处理部分中参数
    void init(CSelfAlgParam* p_pAlgParam);

    // 执行视频检测后处理算法
    void execute();

    // 标准化目标检测框 操作
    void standard_box(std::vector<std::vector<yolov5Detection>> &input);

    // 视频结果处理
    void result_process(std::vector<std::vector<yolov5Detection>> &res_batch);

    // 加载单应性矩阵
    void loadWarpMatrix(CSelfAlgParam* p_pAlgParam);

    // 单应性映射
    void WarPersepctiveMatrix(xt::xarray<float> &corner_2d, xt::xarray<float> &corner_3d,
                                            xt::xarray<float> &warpMatrix);
    // 获取目标在雷达坐标系下的xyz
    bool get_xyz(CObjectResult &object, int cameraID, int class_id);

    void load_filter_image(std::string file_path, int cameraId);

    void read_videofusion_xml(const char* videofusion_xml_path);

    bool video_to_wa(CObjectResult &object, int cameraID, int class_id);

    // // 加载地图信息
    // void loadMapInformation(CSelfAlgParam &p_pAlgParam);

    void test(CAlgResult outputAlgResult);      // 感知结果输出（测试需要）

     void setCommonData(CCommonDataPtr p_commonData) override
    {
        m_CommonData = p_commonData;
    }

    CCommonDataPtr getCommonData() override
    {
        return m_CommonData;
    }
    
private:
    // spills_detection *m_SpillsDetector;                 // 抛洒物检测
    // std::vector<spills_detection *> m_arrSpillDetector;
    int Camera_num;
    std::vector<std::string> VideoClass;                    // 图像感知模型输出类别名
    CAlgResult algResult;                                   // 定义检测模块输出结果

    std::map<int, xt::xarray<double>> tiles_map;             // 高精度地图数据
    xt::xarray<double> fish_tile;                            // 鱼眼瓦片图
    xt::xarray<float> class_size;                           // 目标尺寸信息                   
    std::vector<xt::xarray<float>> warpMatrix_vec;          // 单应性矩阵
    float _GroundHeight;                                    // 地面点高度
    int _input_frame_W;                                     // 检测模型输入图像宽度             
    int _input_frame_H;                                     // 检测模型输入图像高度
    int _project_frame_W;                                   // 原始图像宽度
    int _project_frame_H;                                   // 原始图像高度
    std::vector<float> conf_thresholds;                     // 不同类别的置信度阈值
    std::map<int, cv::Mat> m_mapfilter_image_flag;                           // 过滤图像标志位  load_filter_image

    bool use_warpersepctivematrix;                          // 单应性矩阵
    bool use_tilemap;                                       // 瓦片图
    bool use_camerarelect;
    // bool use_spillsdetection;                               // 抛洒物

    double lidar_Lon;       // 主雷达经度
    double lidar_lat;       // 主雷达纬度
    double lidar_angle;     // 主雷达原点角度
    std::map<int, std::vector<std::vector<double>>> m_mapRmF;
    std::map<int, std::vector<std::vector<double>>> m_mapXrange;
    std::map<int, std::vector<std::vector<double>>> m_mapXadjust;
    std::map<int, std::vector<std::vector<double>>> m_mapYrange;
    std::map<int, std::vector<std::vector<double>>> m_mapYadjust;
};

