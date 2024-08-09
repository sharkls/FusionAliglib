#ifndef CMAKEALGS_VIDEO_H
#define CMAKEALGS_VIDEO_H

#include <iostream>
#include <memory>
#include "IMakeAlg.h"
#include "ICommonAlg.h"

// video
#include "Video_detection_preprocessor.h"
#include "yolov5.h"
#include "Video_detection_postprocessor.h"

class CMakeVideoAlgs_strategy final : public IMakeAlg {
public:
    CMakeVideoAlgs_strategy();
    explicit CMakeVideoAlgs_strategy(std::string &p_sfileName)
    {   
        LOG(INFO) <<   p_sfileName << std::endl;
        // 读取配置文件中算法选择参数
        YAML::Node video_cfg = YAML::LoadFile(p_sfileName);
        b_preIsValid = video_cfg["VIDEO_ALG"]["PRE_ISVALID"].as<bool>();
        b_infIsValid = video_cfg["VIDEO_ALG"]["PROCESSING_ISVALID"].as<bool>();
        b_postIsValid = video_cfg["VIDEO_ALG"]["POST_ISVALID"].as<bool>();
        m_preName = video_cfg["VIDEO_ALG"]["PRE_ALG"].as<std::string>();
        m_infName = video_cfg["VIDEO_ALG"]["PROCESSING_ALG"].as<std::string>();
        m_postName = video_cfg["VIDEO_ALG"]["POST_ALG"].as<std::string>();
    };

    ~CMakeVideoAlgs_strategy() = default;

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
    std::map<std::string, ICommonAlgPtr> m_mapPreAlgsLibs = {{"video_pre_1", std::make_shared<CVPre>()},};
    std::map<std::string, ICommonAlgPtr> m_mapInfAlgsLibs = {{"yolov5", std::make_shared<yolov5>()},};
    std::map<std::string, ICommonAlgPtr> m_mapPostAlgsLibs = {{"video_post_1", std::make_shared<CVPost>()},};
};

#endif //CMAKEALGS_VIDEO_H