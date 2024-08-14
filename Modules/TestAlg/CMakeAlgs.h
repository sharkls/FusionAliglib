#ifndef CMAKEALGS_TEST_H
#define CMAKEALGS_TEST_H

#include <iostream>
#include <memory>
#include "IMakeAlg.h"
#include "ICommonAlg.h"

// test
#include "TestPreprocess.h"
#include "TestProcessing.h"
#include "TestPostprocess.h"

class CMakeTestAlgs_strategy final : public IMakeAlg {
public:
    CMakeTestAlgs_strategy();
    explicit CMakeTestAlgs_strategy(std::string &p_sfileName)
    {   
        LOG(INFO) <<   p_sfileName << std::endl;
        // 读取配置文件中算法选择参数
        YAML::Node video_cfg = YAML::LoadFile(p_sfileName);
        b_preIsValid = video_cfg["TEST_ALG"]["PRE_ISVALID"].as<bool>();
        b_infIsValid = video_cfg["TEST_ALG"]["PROCESSING_ISVALID"].as<bool>();
        b_postIsValid = video_cfg["TEST_ALG"]["POST_ISVALID"].as<bool>();
        m_preName = video_cfg["TEST_ALG"]["PRE_ALG"].as<std::string>();
        m_infName = video_cfg["TEST_ALG"]["PROCESSING_ALG"].as<std::string>();
        m_postName = video_cfg["TEST_ALG"]["POST_ALG"].as<std::string>();
    };

    ~CMakeTestAlgs_strategy() = default;

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
    std::map<std::string, ICommonAlgPtr> m_mapPreAlgsLibs = {{"pre_1", std::make_shared<TestPreProcess>()},};
    std::map<std::string, ICommonAlgPtr> m_mapInfAlgsLibs = {{"model_x", std::make_shared<TestProcessing>()},};
    std::map<std::string, ICommonAlgPtr> m_mapPostAlgsLibs = {{"post_1", std::make_shared<TestPostProcess>()},};
};

#endif //CMAKEALGS_TEST_H