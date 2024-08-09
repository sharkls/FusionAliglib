#ifndef CMAKEALGS_FUSION_H
#define CMAKEALGS_FUSION_H

#include <iostream>
#include <memory>
#include "IMakeAlg.h"
#include "ICommonAlg.h"
#include "fusion_algorithm_intersection.h"
#include "FusionPreprocess.h"
#include "FusionPostprocess.h"

class CMakeFuTrAlgs_strategy final : public IMakeAlg {
public:
    CMakeFuTrAlgs_strategy();
    explicit CMakeFuTrAlgs_strategy(std::string &p_sfileName)
    {
        // 读取配置文件中点云算法选择参数
        YAML::Node futr_cfg = YAML::LoadFile(p_sfileName);
        b_preIsValid = futr_cfg["FUSION_TRACKING_ALG"]["PRE_ISVALID"].as<bool>();
        b_infIsValid = futr_cfg["FUSION_TRACKING_ALG"]["PROCESSING_ISVALID"].as<bool>();
        b_postIsValid = futr_cfg["FUSION_TRACKING_ALG"]["POST_ISVALID"].as<bool>();
        m_preName = futr_cfg["FUSION_TRACKING_ALG"]["PRE_ALG"].as<std::string>();
        m_infName = futr_cfg["FUSION_TRACKING_ALG"]["PROCESSING_ALG"].as<std::string>();
        m_postName = futr_cfg["FUSION_TRACKING_ALG"]["POST_ALG"].as<std::string>();
    };

    ~CMakeFuTrAlgs_strategy() = default;

    void makePre() override {
        if(b_preIsValid)
        {
            m_pPreAlg = m_mapPreAlgsLibs[m_preName];
            m_vecAlgs.push_back(m_pPreAlg);
        }
    }

    void makeInf() override {
        if(b_infIsValid)
        {
            m_pInfAlg = m_mapInfAlgsLibs[m_infName];
            m_vecAlgs.push_back(m_pInfAlg);
        }
    }

    void makePost() override {
        if(b_postIsValid)
        {
            m_pPostAlg = m_mapPostAlgsLibs[m_postName];
            m_vecAlgs.push_back(m_pPostAlg);
        }
        else
        {
            std::cout << "Fusion Tracking stratagy has no post process." << std::endl;
        }
    }

public:
    void makeAlgs() override {
        makePre();
        makeInf();
        makePost();
    }

private:
    //是否开启各模块算法
    bool b_preIsValid;
    bool b_infIsValid;
    bool b_postIsValid;

    //各模块对应算法选择
    std::string m_preName;
    std::string m_infName;
    std::string m_postName;

    //各模块中对应算法
    std::map<std::string, ICommonAlgPtr> m_mapPreAlgsLibs = {
                                                            {"Detection_resault_transfer", std::make_shared<CFusionPreprocess>()},
                                                            };
    std::map<std::string, ICommonAlgPtr> m_mapInfAlgsLibs = {
                                                            {"intersection", std::make_shared<Fusion_Algorithm_Intersection>()},
                                                            };
    std::map<std::string, ICommonAlgPtr> m_mapPostAlgsLibs = {
                                                            {"FuTrPost", std::make_shared<CFusionPostprocess>()},
                                                            };
};

#endif //CMAKEALGS_FUSION_H
