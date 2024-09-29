#ifndef ICOMMONALG_FUSION_H
#define ICOMMONALG_FUSION_H

#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xarray.hpp>
#include <xtensor-blas/xlinalg.hpp>
#include "Log/glog/logging.h"
#include "yaml-cpp/yaml.h"
#include "CSelfAlgParam.h"

class ICommonData{
  public:
    ICommonData() = default;
    ~ICommonData() = default;

  public:
    // Fusion
    CAlgResult FuTrAlgdata;
    std::vector<xt::xarray<float>> m_vecVideoXarrayResult;
    xt::xarray<float> m_fPcXarrayResult;
    xt::xarray<float> m_fTrackXarrayResult;
    int sourceDataType = 1;       // 1： 点云融合    2： 视频融合
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


#endif //ICOMMONALG_FUSION_H