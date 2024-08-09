#include "CVideoAlg.h"
#include "Log/glog/logging.h"
#include "CSelfAlgParam.h"
#include <cstdio>
using namespace std;
/**
 * 获取当前ms UTC时间
 * 参数：
 * 返回值：ms UTC时间
 */
int64_t CVideoAlg::GetTimeStamp()
{
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());

    auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
    return tmp.count();
}

std::string	CVideoAlg::GetVersion()
{
    return "AlgLib VideoAlg V1.3";
}

/**
 * 构造函数
 * 参数：
 *   p_strExePath - 算法库执行路径
 * 功能：
 *   初始化输出路径和日志记录版本信息
 */
CVideoAlg::CVideoAlg(const std::string& p_strExePath):m_strOutPath(p_strExePath)
{   
    m_strOutPath = p_strExePath;
    LOG(INFO)<<"CVideoAlg Version: "<< GetVersion() <<std::endl; 
}

/**
 * 析构函数
 * 功能：
 *   用于清理在CVideoAlg生命周期中分配的资源
 */
CVideoAlg::~CVideoAlg()
{

}

/**
 * 初始化算法
 * 参数：
 *   p_pAlgParam - 算法参数
 *   alg_cb - 算法回调函数
 *   hd - 回调函数的用户数据
 * 返回值：
 *   bool - 初始化成功返回true，否则返回false
 * 功能：
 *   设置算法参数，初始化算法选择和数据类型
 */
bool CVideoAlg::InitAlgorithm(CSelfAlgParam* p_pAlgParam, const AlgCallback & alg_cb, void * hd)
{   
    if (!p_pAlgParam){
        LOG(ERROR)<< "CVideoAlg::InitAlgorithm ---- End >>> Failed : No Video Data."<<std::endl;
        return false;
    }

    m_stSelfVideoAlgParam = *p_pAlgParam;
    // 回调函数相关参数
    cv_callback = alg_cb;
    cv_hd = hd;

    // 算法选择
    std::string s_fileName = m_strOutPath + "/Configs/Alg/Alg.yaml";
    p_makeAlgs = std::make_shared<CMakeVideoAlgs_strategy>(s_fileName);
    p_makeAlgs->makeAlgs();

    // 初始化数据指针及初始化各子模块
    p_CompositeAlgs = std::make_shared<CCompositeAlgs>(p_makeAlgs);
    p_CompositeAlgs->init(&m_stSelfVideoAlgParam);

    LOG(INFO)<< "CVideoAlg::InitAlgorithm ---- finish!"<<std::endl;
    return true;    
}

/**
 * 运行算法
 * 参数：
 *   p_pSrcData - 源数据指针
 * 返回值：
 *   void* - 算法处理结果
 * 功能：
 *   执行算法处理流程，包括图像数据处理、算法执行、结果处理和回调函数调用
 */
void CVideoAlg::RunAlgorithm(void* p_pSrcData)
{   
    LOG(INFO)<< "CVideoAlg::RunAlgorithm ---- start."<<std::endl;

    if (!p_pSrcData){
        LOG(ERROR)<< "CVideoAlg::RunAlgorithm ---- The Video processing incoming data is empty"<<std::endl;
        return;                         // 如果图像输入数据为空 ，直接跳过图像算法
    }
    int64_t VideoSubTimeStamp = GetTimeStamp();

    // 1.图像数据处理
    l_pVideoSrcData = *(static_cast<CVideoSrcDataTimematch *>(p_pSrcData));   

    // 2. 数据传输至算法内部
    CCommonDataPtr CommonData = std::make_shared<ICommonData>();
    // CommonData->VideoStartTime = VideoSubTimeStamp;
    CommonData->img_SrcData = l_pVideoSrcData;
    p_CompositeAlgs->setCommonAllData(CommonData);

    // 3.执行算法操作
    p_CompositeAlgs->execute();

    // 4.对算法输出结果进行处理
    m_stVideoAlgResult = static_cast<CAlgResult> (CommonData->videoAlgResult);
    
    // 5.图像感知结果信息设置
    int64_t VideoResultPubTimeStamp = GetTimeStamp();       
    auto latency_all = VideoResultPubTimeStamp - VideoSubTimeStamp;     // 图像检测算法耗时
    for (int i = 0 ; i < m_stVideoAlgResult.vecFrameResult().size() ; i++){
        m_stVideoAlgResult.vecFrameResult()[i].unFrameId(CommonData->img_SrcData.vecVideoSrcData()[i].unFrameId());                            // 传递帧ID
        m_stVideoAlgResult.vecFrameResult()[i].tCameraSupplement().ucCameraId(CommonData->img_SrcData.vecVideoSrcData()[i].ucCameraId());      // 传递相机ID
        m_stVideoAlgResult.vecFrameResult()[i].eDataType(DATA_TYPE_VIDEO_RESULT);   // 视频结果数据结构
        m_stVideoAlgResult.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_VIDEO_SUB] = CommonData->VideoStartTime;                 // 视频算法活动获取到原始视频数据时的时间戳
        m_stVideoAlgResult.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_VIDEO_RESULT_PUB] = VideoResultPubTimeStamp;    // 视频算法活动发布到检测结果数据时的时间戳
        m_stVideoAlgResult.vecFrameResult()[i].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] = CommonData->img_SrcData.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB];   // 视频算法活动获取到匹配的点云数据时间戳
        m_stVideoAlgResult.vecFrameResult()[i].mapDelay()[DELAY_TYPE_VIDEO_DETECTION_ALG] = latency_all;       //检测算法耗时
    }
    
    LOG(INFO) <<"RunAlgorithm ---- End >>> All Time : " << latency_all << " ms." << std::endl;
    std::cout << "m_stVideoAlgResult.mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] : " << m_stVideoAlgResult.vecFrameResult()[0].mapTimeStamp()[TIMESTAMP_PCSRCINFO_SUB] << std::endl;
    LOG(INFO) <<"m_stVideoAlgResult.vecFrameResult()[i].eDataType(DATA_TYPE_VIDEO_RESULT) : " << (int)m_stVideoAlgResult.vecFrameResult()[0].eDataType() << std::endl;
    
    // 5. 通过回调函数返回结果
    cv_callback(m_stVideoAlgResult, cv_hd); 
    LOG(INFO)<< "CVideoAlg::CvRun once status:  over."<<std::endl;
}
