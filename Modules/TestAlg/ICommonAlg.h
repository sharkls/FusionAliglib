#ifndef ICOMMONALG_VIDEO_H
#define ICOMMONALG_VIDEO_H

#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include <opencv2/core/mat.hpp>
#include "Log/glog/logging.h"
#include "yaml-cpp/yaml.h"
#include "CSelfAlgParam.h"
#include "CVideoSrcData.h"

class ICommonData{
 public:
  ICommonData() = default;
  ~ICommonData() = default;

 public:
    // video
    int64_t VideoStartTime;
    CVideoSrcDataTimematch img_SrcData;     // 图像原始数据(软件传输到算法的原始数据)
    std::vector<cv::Mat> img_batch;         // 图像检测算法的输入（组成batch后图像数据）
    void* cpu_buffer;                       // 检测算法的输出，图像后处理算法的输入
    CAlgResult videoAlgResult;              // 图像后处理算法输出
};

using CCommonDataPtr = std::shared_ptr<ICommonData>;

class ICommonAlg{
 public:
  ICommonAlg(){};
  virtual ~ICommonAlg() = default;

  virtual void init(CSelfAlgParam* p_pAlgParam) = 0;
  virtual void execute() = 0;

  virtual void setCommonData(CCommonDataPtr p_commonData) = 0;
  virtual CCommonDataPtr getCommonData() = 0;

 public:
  CCommonDataPtr m_CommonData;

};

using ICommonAlgPtr = std::shared_ptr<ICommonAlg>;


#endif //ICOMMONALG_VIDEO_H