#ifndef CCOMPOSITEALGS_VIDEO_H
#define CCOMPOSITEALGS_VIDEO_H

#include <iostream>
#include <memory>
#include <vector>
#include <xtensor/xview.hpp>
#include <xtensor/xnpy.hpp>
#include <xtensor/xsort.hpp>
#include <xtensor/xarray.hpp>
#include "ICommonAlg.h"
#include "IMakeAlg.h"
#include "CMakeAlgs.h"

class CCompositeAlgs final : public ICommonAlg{
 public:
  CCompositeAlgs() = delete;
  explicit CCompositeAlgs(IMakeAlgPtr& p_makeAlgs){
      m_vecAlgs = p_makeAlgs->getAlgs();// 將４個模塊待執行的部分放入m_vecAlgs中
  };

  void init(CSelfAlgParam* p_pAlgParam) override
  {
      for (const auto& command : m_vecAlgs) {
          command->setCommonData(m_CommonData);
          command->init(p_pAlgParam);
          m_CommonData = command->getCommonData();
      }
  }

  void execute() override {
      for (const auto& command : m_vecAlgs) {
          command->execute();
      }
  }
  void setCommonData(CCommonDataPtr p_commonData) override
  {
      m_CommonData = p_commonData;
  }

  CCommonDataPtr getCommonData() override
  {
      return m_CommonData;
  }
  
  void setCommonAllData(CCommonDataPtr p_commonData)
  {
    setCommonData(p_commonData);
    for (const auto& command : m_vecAlgs) {
        command->setCommonData(m_CommonData);
    }
  }

 private:
  std::vector<ICommonAlgPtr> m_vecAlgs;
};

using CCompositeAlgsPtr = std::shared_ptr<CCompositeAlgs>;
#endif //CCOMPOSITEALGS_VIDEO_H
