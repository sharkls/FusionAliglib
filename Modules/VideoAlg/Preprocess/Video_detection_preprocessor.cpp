#include "Video_detection_preprocessor.h"
#include "Log/glog/logging.h"
#include "CSelfAlgParam.h"
#include <cstdio>

/*
*    功能：构造函数，初始化视频预处理器并记录日志
*/
CVPre :: CVPre()
{   
    LOG(INFO) << "CVPre :: CVPre  status:    Started." << std::endl;
}


/*
*    功能：析构函数，用于清理视频预处理器
*/
CVPre :: ~CVPre()
{

};


/*
*    功能：初始化视频预处理器，设置算法参数
*    参数:
*       p_pAlgParam : 指向算法参数的指针
*/
void CVPre::init(CSelfAlgParam * p_pAlgParam)
{   
    // 算法参数转换
    LOG(INFO) << "CVPre :: init status : finish!" << std::endl;
}


/*
*    功能：执行视频预处理操作，包括获取数据、调整大小和设置公共数据
*/
void CVPre::execute()
{   
    LOG(INFO)  << "CVPre::execute  status :  start" << std::endl;
    auto t_start = std::chrono::high_resolution_clock::now();

    // 1. 图像预处理模块数据输出
    CVideoSrcDataTimematch video_Srcdata = getCommonData()->img_SrcData;
    
    // 2. Resize 处理
    if(video_Srcdata.vecVideoSrcData().size() <= 0){
        LOG(ERROR)  << "CVPre::execute ----  video_SrcData is None " << std::endl;
        m_CommonData->img_batch.clear();            // 清空上次数据
        return;                                     // 跳出前处理
    }
    std::vector<cv::Mat> img_batch = Resize(&video_Srcdata);

    // 3. 图像预处理模块数据输出
    m_CommonData->img_batch = img_batch;
    auto t_end = std::chrono::high_resolution_clock::now();
    auto latency_sp = (std::chrono::duration_cast<std::chrono::duration<int, std::micro>>(t_end - t_start).count())/1000;
    LOG(INFO)  << "CVPre::execute status :   finish >>> All Time :"<< latency_sp << " ms"  << std::endl;
}


/*
*    功能：调整视频源数据的大小
*    参数:
*       p_pSrcData : 指向视频源数据的指针
*    返回:
*       包含调整大小后的图像的向量
*/
std::vector<cv::Mat> CVPre::Resize(CVideoSrcDataTimematch * p_pSrcData)
{   
    auto t_start = std::chrono::high_resolution_clock::now();
    if (!p_pSrcData)                            // 核验图像数据合法性
    {
       LOG(ERROR) << "CVPre::Resize ---- p_pSrcData is None." << std::endl;
    }

    //视频数据处理
    int l_nVideoNumber = p_pSrcData->vecVideoSrcData().size();                                       // 获取图像包中帧数
    
    std::vector<cv::Mat> l_vecVideoSrcData;
    for(int i = 0; i < l_nVideoNumber; ++i)
    {   
        cv::Mat l_matData = cv::Mat(p_pSrcData->vecVideoSrcData()[i].vecImageBuf());
        l_matData = l_matData.reshape(3, p_pSrcData->vecVideoSrcData()[i].usBmpLength()).clone();    //  将图像数据转换为指定格式
        l_vecVideoSrcData.push_back(l_matData);
    }

    return l_vecVideoSrcData;
}


